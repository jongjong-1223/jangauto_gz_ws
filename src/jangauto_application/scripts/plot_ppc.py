#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String, Int32, Float64
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
from tf_transformations import euler_from_quaternion
import json
from datetime import datetime
import os
import math

class PlotPPC2(Node):
    def __init__(self):
        super().__init__('ppc_plotter_v2')
        
        # Parameters
        self.declare_parameter('motor_script', 'motor_cmd_vel_sim_2.py')
        self.declare_parameter('lookahead_distance', 1.5)
        
        # Get parameter values
        self.motor_script = self.get_parameter('motor_script').get_parameter_value().string_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        
        # motor_script → strategy_name mapping
        self.strategy_map = {
            # Sim
            'motor_cmd_vel_sim.py': 'Baseline',
            'motor_cmd_vel_sim_1.py': 'Proportional',
            'motor_cmd_vel_sim_2.py': 'Angular_Priority',
            'motor_cmd_vel_sim_3.py': 'Linear_Priority',
            # Real
            'motor_cmd_vel_real.py': 'Baseline',
            'motor_cmd_vel_real_proportional.py': 'Proportional',
            'motor_cmd_vel_real_linear.py': 'Linear_Priority',
        }
        self.strategy_name = self.strategy_map.get(self.motor_script, 'Unknown')
        
        self.get_logger().info(
            f'[PLOT_PPC_V2] Line Following Mode | Strategy={self.strategy_name}, Ld={self.lookahead_distance}m'
        )
        
        # Subscriptions
        self.create_subscription(Path, '/waypoints_path', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_ppc', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/ppc/enable', self.enable_callback, 10)
        self.create_subscription(PoseStamped, '/ppc/lookahead_point', self.lookahead_callback, 10)
        self.create_subscription(String, '/ppc/state', self.state_callback, 10)
        self.create_subscription(Int32, '/ppc/waypoint_idx', self.waypoint_idx_callback, 10)
        self.create_subscription(Float64, '/ppc/heading_error', self.heading_error_callback, 10)
        self.create_subscription(Float64, '/ppc/cte', self.cte_callback, 10)
        
        # Data Storage
        self.waypoints = []
        self.actual_path = deque() 
        self.current_pose = None
        self.current_yaw = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.is_enabled = False
        self.was_mission_completed = False
        self.lookahead_point = None
        self.current_state = "move"
        self.current_waypoint_idx = 1
        
        # Time-series data
        self.time_stamps = deque()
        self.cte_values = deque()
        self.heading_errors = deque()
        self.start_time = None
        self.run_start_time = None
        
        # TTR (Time To Recover)
        # Thresholds
        self.CTE_OUT = 0.20      # Out-of-bounds threshold (m)
        self.CTE_IN = 0.08       # Recovery threshold (m)
        self.HE_OUT = 20.0       # Heading error out-of-bounds threshold (degrees)
        self.HE_IN = 12.0        # Heading error recovery threshold (degrees)
        self.T_HOLD = 0.5        # Minimum hold time (s)
        self.TTR_TIMEOUT = 10.0  # Recovery timeout (s)
        self.TTR_COOLDOWN = 0.5  # Cooldown to prevent consecutive events (s)
        
        # State variables
        self.ttr_state = "normal"  # "normal", "off track", "recovering"
        self.t_out = None          # Out-of-bounds start time
        self.recovery_start = None # Recovery condition start time
        self.last_ttr_event = None # Last TTR event time (for cooldown)
        
        # TTR position tracking
        self.pose_at_t_out = None  # (x, y) coordinates at out-of-bounds time
        
        # TTR records
        self.ttr_events = []
        
        # Moving average filters (to reduce sensor noise)
        self.cte_filter = deque(maxlen=3)     # 3-sample moving average
        self.he_filter = deque(maxlen=3)      # 3-sample moving average
        
        # Plot setup
        self.setup_plot()

        self.get_logger().info('[PLOT_PPC_V2] Line Following PlotPPC initialized')

    def waypoints_callback(self, msg: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) 
                         for pose in msg.poses]
        self.get_logger().info(f'[PLOT_PPC_V2] Received {len(self.waypoints)} waypoints')
    
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_pose = (x, y)
        self.current_yaw = yaw
        
        if self.is_enabled:
            self.actual_path.append((x, y))
            
            if self.start_time is None:
                self.start_time = self.get_clock().now()
    
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def enable_callback(self, msg: Bool):
        if msg.data and not self.is_enabled:
            if self.was_mission_completed or self.start_time is None:
                self.actual_path.clear()
                self.time_stamps.clear()
                self.cte_values.clear()
                self.heading_errors.clear()
                self.start_time = None
                self.run_start_time = self.get_clock().now()
                
                self.ttr_state = "normal"
                self.t_out = None
                self.recovery_start = None
                self.last_ttr_event = None
                self.ttr_events.clear()
                self.cte_filter.clear()
                self.he_filter.clear()
                self.pose_at_t_out = None
                
                self.was_mission_completed = False
                
                self.get_logger().info('[PLOT_PPC_V2] PPC Enabled - NEW mission started')
            else:
                # Resumed after pause - data preserved
                self.get_logger().info('[PLOT_PPC_V2] PPC RESUMED - Continuing data recording')
        
        elif not msg.data and self.is_enabled:
            self.get_logger().info('[PLOT_PPC_V2] PPC PAUSED - Data preserved')
        
        self.is_enabled = msg.data

    # From PPC
    def lookahead_callback(self, msg: PoseStamped):
        self.lookahead_point = (msg.pose.position.x, msg.pose.position.y)
    
    def state_callback(self, msg: String):
        self.current_state = msg.data

    def waypoint_idx_callback(self, msg: Int32):
        old_idx = self.current_waypoint_idx
        self.current_waypoint_idx = msg.data
        
        # Mission completion detection: when exactly the last waypoint index is reached
        # (In ppc_2.py's check_arrival, mission_completed=True sets idx to len-1)
        if self.waypoints and self.current_waypoint_idx == len(self.waypoints) - 1:
            # Only save if the index just changed (just reached), not saved yet, and enabled
            if old_idx != self.current_waypoint_idx and not self.was_mission_completed and self.is_enabled:
                self.was_mission_completed = True
                self.save_run_data()
                self.get_logger().info('[PLOT_PPC_V2] Mission completed - Data saved')
    
    def heading_error_callback(self, msg: Float64):
        he_value = msg.data
        
        # Apply moving average filter
        self.he_filter.append(he_value)
        he_filtered = sum(self.he_filter) / len(self.he_filter)
        
        if self.is_enabled and self.start_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            self.time_stamps.append(elapsed)
            self.heading_errors.append(he_filtered)
            
            # Calculate TTR
            if self.current_state == "move":
                self._update_ttr(elapsed, he_filtered)

    def cte_callback(self, msg: Float64):
        cte_value = msg.data
        
        # Apply moving average filter
        self.cte_filter.append(cte_value)
        cte_filtered = sum(self.cte_filter) / len(self.cte_filter)
        
        if self.is_enabled and self.start_time is not None:
            self.cte_values.append(cte_filtered)

    def _update_ttr(self, current_time, heading_error):
        if not self.cte_values:
            return
        
        current_cte = self.cte_values[-1]
        
        # Cooldown check
        if self.last_ttr_event is not None:
            if current_time - self.last_ttr_event < self.TTR_COOLDOWN:
                return
        
        # State Machine
        
        if self.ttr_state == "normal":
            # Off-track detection
            if self.current_state == "move":
                if current_cte > self.CTE_OUT or abs(heading_error) > self.HE_OUT:
                    self.ttr_state = "off track"
                    self.t_out = current_time
                    self.recovery_start = None
                    self.pose_at_t_out = self.current_pose if self.current_pose else None
                    self.get_logger().info(
                        f'[TTR] off track detected at t={current_time:.2f}s, CTE={current_cte:.3f}m, HE={heading_error:.1f}°'
                    )
            else:
                if current_cte > self.CTE_OUT:
                    self.ttr_state = "off track"
                    self.t_out = current_time
                    self.recovery_start = None
                    self.pose_at_t_out = self.current_pose if self.current_pose else None
                    self.get_logger().info(
                        f'[TTR] off track detected at t={current_time:.2f}s, CTE={current_cte:.3f}m'
                    )
        
        elif self.ttr_state == "off track":
            # Timeout check
            if current_time - self.t_out > self.TTR_TIMEOUT:
                self.get_logger().warn(
                    f'[TTR] Recovery FAILED - Timeout after {self.TTR_TIMEOUT}s'
                )
                self.ttr_events.append({
                    't_out': self.t_out,
                    't_in': None,
                    'TTR': None,
                    'success': False,
                    'reason': 'timeout',
                    'pose_out': list(self.pose_at_t_out) if self.pose_at_t_out else None,
                    'pose_in': None
                })
                self.ttr_state = "normal"
                self.t_out = None
                self.recovery_start = None
                self.pose_at_t_out = None 
                self.last_ttr_event = current_time
                return
            
            # Recovery condition check
            if current_cte <= self.CTE_IN and abs(heading_error) <= self.HE_IN:
                if self.recovery_start is None:
                    self.recovery_start = current_time
                    self.ttr_state = "recovering"
                    self.get_logger().info(
                        f'[TTR] Recovery started at t={current_time:.2f}s'
                    )
        
        elif self.ttr_state == "recovering":
            # Recovery condition violation check
            if current_cte > self.CTE_IN or abs(heading_error) > self.HE_IN:
                self.get_logger().info(
                    f'[TTR] Recovery interrupted at t={current_time:.2f}s - Resetting hold timer'
                )
                self.recovery_start = None
                self.ttr_state = "off track"
                return
            
            # Hold duration check
            hold_duration = current_time - self.recovery_start
            if hold_duration >= self.T_HOLD:
                t_in = current_time
                ttr = t_in - self.t_out
                
                self.get_logger().info(
                    f'[TTR] Recovery SUCCESS - TTR={ttr:.2f}s'
                )
                
                self.ttr_events.append({
                    't_out': self.t_out,
                    't_in': t_in,
                    'TTR': ttr,
                    'success': True,
                    'cte_max': max([v for v in self.cte_values if v is not None], default=0),
                    'hold_duration': hold_duration,
                    'pose_out': list(self.pose_at_t_out) if self.pose_at_t_out else None,
                    'pose_in': list(self.current_pose) if self.current_pose else None
                })
                
                self.ttr_state = "normal"
                self.t_out = None
                self.recovery_start = None
                self.pose_at_t_out = None
                self.last_ttr_event = current_time
    
    # Plot Setup
    def setup_plot(self):
        self.fig = plt.figure(figsize=(18, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[3, 1], 
                                  hspace=0.45, wspace=0.3)
        
        # Path Plot (Left Full)
        self.ax_path = self.fig.add_subplot(gs[:, 0])
        self.ax_path.set_title('Line Following Pure Pursuit', fontsize=14, fontweight='bold')
        self.ax_path.set_xlabel('X (m)', fontsize=11)
        self.ax_path.set_ylabel('Y (m)', fontsize=11)
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.set_xlim(0, 0)
        self.ax_path.set_ylim(0, 0)
        self.ax_path.set_aspect('equal', adjustable='box')
        
        # Control Info (Top Right)
        self.ax_info = self.fig.add_subplot(gs[0, 1])   
        self.ax_info.set_title('Control Info', fontsize=11, fontweight='bold', pad=10)
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.5, 'Waiting...', fontsize=9,
                                            verticalalignment='center', 
                                            family='monospace',
                                            linespacing=1.3)

        # Cross-Track Error (Middle Right)
        self.ax_cte = self.fig.add_subplot(gs[1, 1])
        self.ax_cte.set_title('Cross-Track Error', fontsize=11, fontweight='bold', pad=10)
        self.ax_cte.set_xlabel('Time (s)', fontsize=9)
        self.ax_cte.set_ylabel('CTE (m)', fontsize=9)
        self.ax_cte.grid(True, alpha=0.3)
        self.ax_cte.tick_params(labelsize=8)

        # Heading Error (Bottom Right)
        self.ax_heading = self.fig.add_subplot(gs[2, 1])
        self.ax_heading.set_title('Heading Error', fontsize=11, fontweight='bold', pad=10)
        self.ax_heading.set_xlabel('Time (s)', fontsize=9)
        self.ax_heading.set_ylabel('Error (deg)', fontsize=9)
        self.ax_heading.grid(True, alpha=0.3)
        self.ax_heading.tick_params(labelsize=8)
        
        # Initialize Plot Elements
        self.waypoints_line, = self.ax_path.plot([], [], 'r--', linewidth=2, 
                                                  label='Waypoints', marker='o', markersize=6)
        
        # Highlight Active Segment
        self.active_segment_line, = self.ax_path.plot([], [], 'm-', linewidth=4, 
                                                       label='Active Line', alpha=0.7, zorder=3)
        
        self.robot_marker, = self.ax_path.plot([], [], 'go', markersize=10, label='Robot')

        self.ttr_deviate_markers, = self.ax_path.plot([], [], 'rx', markersize=12, 
                                                       markeredgewidth=2, label='Deviate', zorder=10)
        
        self.robot_arrow = None

        self.actual_line, = self.ax_path.plot([], [], 'b-', linewidth=2, 
                                               label='Actual Path', alpha=0.7)
        
        self.lookahead_marker, = self.ax_path.plot([], [], 'co', markersize=10, 
                                                    label='Lookahead', zorder=5)
        
        self.lookahead_line, = self.ax_path.plot([], [], 'c--', linewidth=2, alpha=0.5)
        
        # TTR Event Markers
        self.ttr_recover_markers, = self.ax_path.plot([], [], 'g+', markersize=14, 
                                                       markeredgewidth=2, label='Recover', zorder=10)
        
        self.cte_line, = self.ax_cte.plot([], [], 'r-', linewidth=2)
        self.heading_line, = self.ax_heading.plot([], [], 'b-', linewidth=2)
        
        self.ax_path.legend(loc='upper right', fontsize=7, ncol=2)
        
        # Animation Setup
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, 
                                blit=False, cache_frame_data=False)
    
    def update_plot(self, frame):
        if self.waypoints:
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            self.waypoints_line.set_data(wp_x, wp_y)
        
        # Highlight Active Segment (Previous WP → Target WP)
        if self.waypoints and 1 <= self.current_waypoint_idx < len(self.waypoints):
            p1 = self.waypoints[self.current_waypoint_idx - 1]
            p2 = self.waypoints[self.current_waypoint_idx]
            self.active_segment_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        else:
            self.active_segment_line.set_data([], [])
        
        # Actual Path
        if self.actual_path:
            actual_path_copy = list(self.actual_path)
            if actual_path_copy:
                path_x = [p[0] for p in actual_path_copy]
                path_y = [p[1] for p in actual_path_copy]
                self.actual_line.set_data(path_x, path_y)

        # Auto Adjust Limits
        self._auto_adjust_limits()
        
        # TTR Event Markers
        deviate_x, deviate_y = [], []
        recover_x, recover_y = [], []
        
        for event in self.ttr_events:
            pose_out = event.get('pose_out')
            pose_in = event.get('pose_in')
            
            if pose_out:
                deviate_x.append(pose_out[0])
                deviate_y.append(pose_out[1])
            
            if pose_in:
                recover_x.append(pose_in[0])
                recover_y.append(pose_in[1])
        
        if self.ttr_state in ["off track", "recovering"] and self.pose_at_t_out:
            deviate_x.append(self.pose_at_t_out[0])
            deviate_y.append(self.pose_at_t_out[1])

        self.ttr_deviate_markers.set_data(deviate_x, deviate_y)
        self.ttr_recover_markers.set_data(recover_x, recover_y)
        
        # Robot
        if self.current_pose:
            x, y = self.current_pose
            yaw = self.current_yaw
            self.robot_marker.set_data([x], [y])
            
            if self.robot_arrow:
                self.robot_arrow.remove()
            
            arrow_length = 0.5
            dx = arrow_length * np.cos(yaw)
            dy = arrow_length * np.sin(yaw)
            self.robot_arrow = self.ax_path.arrow(x, y, dx, dy, 
                                                  head_width=0.3, head_length=0.2,
                                                  fc='green', ec='green', alpha=0.7)
        
        # Lookahead Point
        if self.lookahead_point and self.current_pose:
            lx, ly = self.lookahead_point
            rx, ry = self.current_pose
            
            self.lookahead_marker.set_data([lx], [ly])
            self.lookahead_line.set_data([rx, lx], [ry, ly])
            
            ld = math.hypot(lx - rx, ly - ry)
            
            # Active Segment Info
            if self.waypoints and 1 <= self.current_waypoint_idx < len(self.waypoints):
                p1 = self.waypoints[self.current_waypoint_idx - 1]
                p2 = self.waypoints[self.current_waypoint_idx]
                segment_info = f'Seg: WP{self.current_waypoint_idx-1}→WP{self.current_waypoint_idx}'
            else:
                segment_info = 'Seg: N/A'

            # Info Panel Update
            info_lines = [
                '━' * 37,
                f'Mode:             LINE FOLLOWING',
                f'State:            {self.current_state.upper()}',
                f'Current Yaw:      {math.degrees(self.current_yaw):.1f}°',
                '━' * 37,
                segment_info,
                f'Lookahead Dist:   {ld:.2f} m',
                f'Target Point:     ({lx:.2f}, {ly:.2f})',
                '━' * 37,
            ]
            self.info_text.set_text('\n'.join(info_lines))
        else:
            self.lookahead_marker.set_data([], [])
            self.lookahead_line.set_data([], [])
            if not self.is_enabled:
                self.info_text.set_text('Waiting for PPC...')
        
        # CTE
        cte_values_copy = list(self.cte_values)
        time_stamps_copy = list(self.time_stamps)
        
        if cte_values_copy and time_stamps_copy:
            self.cte_line.set_data(time_stamps_copy, cte_values_copy)
            if len(time_stamps_copy) > 0:
                t_max = time_stamps_copy[-1]
                t_min = max(0, t_max - 30)
                if t_max - t_min < 5:
                    t_max = t_min + 5
                self.ax_cte.set_xlim(t_min, t_max + 1)

                if cte_values_copy:
                    cte_max = max(max(cte_values_copy), 0.1)
                    self.ax_cte.set_ylim(0, cte_max * 1.2)
        
        # Heading Error
        heading_errors_copy = list(self.heading_errors)

        if heading_errors_copy and time_stamps_copy:
            self.heading_line.set_data(time_stamps_copy, heading_errors_copy)
            if len(time_stamps_copy) > 0:
                t_max = time_stamps_copy[-1]
                t_min = max(0, t_max - 30)
                if t_max - t_min < 5:
                    t_max = t_min + 5
                self.ax_heading.set_xlim(t_min, t_max + 1)

                if heading_errors_copy:
                    h_max = max(abs(min(heading_errors_copy)), abs(max(heading_errors_copy)))
                    self.ax_heading.set_ylim(-h_max * 1.2, h_max * 1.2)
        
        # Title
        total_events = len(self.ttr_events)
        success_events = sum(1 for e in self.ttr_events if e['success'])
        
        status_text = f"PPC: {'ON' if self.is_enabled else 'OFF'} | State: {self.current_state.upper()}"
        
        ttr_state_map = {
            "normal": "NORMAL",
            "off track": "OFF-TRACK",
            "recovering": "RECOVERING"
        }
        status_text += f" | TTR: {ttr_state_map.get(self.ttr_state, 'UNKNOWN')}"
        
        if total_events > 0:
            status_text += f" | Events: {total_events} ({success_events} success)"
        
        if cte_values_copy:
            status_text += f" | CTE={cte_values_copy[-1]:.3f} m"
        if heading_errors_copy:
            status_text += f" | Heading Err={heading_errors_copy[-1]:.1f}°"
        
        title_color = 'green' if self.is_enabled else 'red'
        if self.ttr_state == "off track":
            title_color = 'red'
        elif self.ttr_state == "recovering":
            title_color = 'orange'

        self.fig.suptitle(status_text, fontsize=14, fontweight='bold', color=title_color)
        
        return [self.waypoints_line, self.active_segment_line, self.actual_line, 
                self.robot_marker, self.lookahead_marker, self.lookahead_line,
                self.ttr_deviate_markers, self.ttr_recover_markers,
                self.cte_line, self.heading_line]
    
    def _auto_adjust_limits(self):
        all_x = []
        all_y = []
        
        if self.waypoints:
            all_x.extend([wp[0] for wp in self.waypoints])
            all_y.extend([wp[1] for wp in self.waypoints])
        
        if self.actual_path:
            actual_path_copy = list(self.actual_path)
            all_x.extend([p[0] for p in actual_path_copy])
            all_y.extend([p[1] for p in actual_path_copy])
        
        if all_x and all_y:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            
            x_margin = (x_max - x_min) * 0.1 or 1.0
            y_margin = (y_max - y_min) * 0.1 or 1.0
            
            x_min -= x_margin
            x_max += x_margin
            y_min -= y_margin
            y_max += y_margin
            
            current_xlim = self.ax_path.get_xlim()
            current_ylim = self.ax_path.get_ylim()
            
            new_x_min = min(x_min, current_xlim[0])
            new_x_max = max(x_max, current_xlim[1])
            new_y_min = min(y_min, current_ylim[0])
            new_y_max = max(y_max, current_ylim[1])
            
            self.ax_path.set_xlim(new_x_min, new_x_max)
            self.ax_path.set_ylim(new_y_min, new_y_max)

    def _calculate_ttr_statistics(self):
        if not self.ttr_events:
            return {
                'total_events': 0,
                'successful_recoveries': 0,
                'failed_recoveries': 0,
                'avg_ttr': 0.0,
                'max_ttr': 0.0,
                'min_ttr': 0.0
            }
        
        successful = [e for e in self.ttr_events if e['success']]
        failed = [e for e in self.ttr_events if not e['success']]
        
        ttr_values = [e['TTR'] for e in successful if e['TTR'] is not None]
        
        return {
            'total_events': len(self.ttr_events),   
            'successful_recoveries': len(successful),
            'failed_recoveries': len(failed),
            'avg_ttr': np.mean(ttr_values) if ttr_values else 0.0,
            'max_ttr': max(ttr_values) if ttr_values else 0.0,
            'min_ttr': min(ttr_values) if ttr_values else 0.0
        }

    def save_run_data(self):
        if not self.actual_path:
            self.get_logger().warn('[PLOT_PPC_V2] No path data to save')
            return
        
        try:
            data_dir = os.path.expanduser('~/ppc_run_data_real_v2')
            os.makedirs(data_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(data_dir, f'ppc_run_v2_{timestamp}.json')
            
            ttr_stats = self._calculate_ttr_statistics()
            
            data = {
                'timestamp': timestamp,
                'mode': 'line_following',
                'metadata': {
                    'strategy_name': self.strategy_name,
                    'lookahead_distance': self.lookahead_distance,
                    'motor_script': self.motor_script
                },
                'waypoints': self.waypoints,
                'actual_path': list(self.actual_path),
                'cte_values': list(self.cte_values),
                'heading_errors': list(self.heading_errors),
                'avg_cte': np.mean(self.cte_values) if self.cte_values else 0.0,
                'max_cte': max(self.cte_values) if self.cte_values else 0.0,
                'total_points': len(self.actual_path),
                'run_duration': (self.get_clock().now() - self.run_start_time).nanoseconds * 1e-9 
                               if self.run_start_time else 0.0,
                'ttr_events': self.ttr_events,
                'ttr_statistics': ttr_stats
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)

            self.get_logger().info(
                f'[PLOT_PPC_V2] Run data saved: {filename}\n'
                f'  Mode: Line Following | Strategy: {self.strategy_name}, Ld: {self.lookahead_distance}m'
            )

            if ttr_stats['total_events'] > 0:
                self.get_logger().info(
                    f"[PLOT_PPC_V2] [TTR Summary] Total: {ttr_stats['total_events']}, "
                    f"Success: {ttr_stats['successful_recoveries']}, "
                    f"Failed: {ttr_stats['failed_recoveries']}, "
                    f"Avg TTR: {ttr_stats['avg_ttr']:.2f}s"
                )
            
        except Exception as e:
            self.get_logger().error(f'[PLOT_PPC_V2] Failed to save run data: {e}')

def main(args=None):
    rclpy.init(args=args)
    plotter = PlotPPC2()
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    spin_thread.start()
    
    plt.show()
    
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
