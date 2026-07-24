#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import os
import csv
from datetime import datetime
from collections import deque
import threading
import sys

# PyQtGraph imports
try:
    from pyqtgraph.Qt import QtCore, QtWidgets
    import pyqtgraph as pg
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False
    print("[WARN] pyqtgraph not available. Install with: pip3 install pyqtgraph PyQt5")


class UWBDataLogger(Node):
    def __init__(self):
        super().__init__('uwb_data_logger')
        
        self.get_logger().info('[UWB_DATA] Node starting...')
        
        # Files to save data
        self.base_dir = os.path.expanduser('~/uwb_data')
        self.uwb_dir = os.path.join(self.base_dir, 'uwb')
        self.kalman_dir = os.path.join(self.base_dir, 'kalman')
        
        os.makedirs(self.uwb_dir, exist_ok=True)
        os.makedirs(self.kalman_dir, exist_ok=True)
        
        self.get_logger().info(f'[UWB_DATA] Data directory: {self.base_dir}')
        
        # Files creation (timestamp-based)
        timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        
        self.uwb_file_path = os.path.join(self.uwb_dir, f'uwb_{timestamp}.csv')
        self.ekf_file_path = os.path.join(self.kalman_dir, f'ekf_{timestamp}.csv')
        
        # Open CSV files
        self.uwb_file = open(self.uwb_file_path, 'w', newline='')
        self.ekf_file = open(self.ekf_file_path, 'w', newline='')
        
        # Create CSV Writers
        self.uwb_writer = csv.writer(self.uwb_file)
        self.ekf_writer = csv.writer(self.ekf_file)
        
        # Write CSV headers
        self.uwb_writer.writerow(['timestamp', 'x', 'y'])
        self.ekf_writer.writerow(['timestamp', 'x', 'y'])
        
        self.get_logger().info(f'[UWB_DATA] UWB file: {self.uwb_file_path}')
        self.get_logger().info(f'[UWB_DATA] EKF file: {self.ekf_file_path}')
        
        # Data buffers (for visualization)
        max_points = 1000  # Show only the most recent 1000 points
        self.uwb_buffer = deque(maxlen=max_points)
        self.ekf_buffer = deque(maxlen=max_points)
        
        # Buffer lock (thread-safe)
        self.buffer_lock = threading.Lock()
        
        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped,'/abs_xy_fixed',self.uwb_callback,10)
        self.create_subscription(Odometry,'/odometry/ekf_single',self.ekf_callback,10)
        
        self.get_logger().info('[UWB_DATA] Subscribed to /abs_xy and /odometry/ekf_single')
        
        # Statistics
        self.uwb_count = 0
        self.ekf_count = 0
        self.start_time = self.get_clock().now()
        
        # PyQtGraph Settings
        # The QApplication must be created in the main thread. setup_plot()
        # will create windows / timers assuming QApplication already exists.
        self.app = None
        self.win = None
        self.timer = None
        
        if PYQTGRAPH_AVAILABLE:
            # setup_plot will be called from main() after QApplication has been created
            pass
        else:
            self.get_logger().warn('[UWB_DATA] Running in CSV-only mode (no visualization)')
        
        # Shutdown Handler
        # Don't use signal handler with Qt - let Qt handle it
        # signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('[UWB_DATA] Node initialized successfully')
    
    # Callbacks
    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        """Receive and save UWB data"""
        try:
            # Timestamp
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            
            # Position data
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # Write to CSV
            self.uwb_writer.writerow([timestamp, x, y])
            self.uwb_file.flush()  # Immediately write to disk
            
            # Add to buffer (for visualization)
            with self.buffer_lock:
                self.uwb_buffer.append((timestamp, x, y))
            
            self.uwb_count += 1
            
            # Periodic log (every 100 samples)
            if self.uwb_count % 100 == 0:
                self.get_logger().info(
                    f'[UWB_DATA] UWB: {self.uwb_count} samples | '
                    f'Latest: ({x:.3f}, {y:.3f})'
                )
                
        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error in uwb_callback: {e}')
    
    def ekf_callback(self, msg: Odometry):
        """Receive and save EKF data"""
        try:
            # Timestamp
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            
            # Position data
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # Calculate Yaw
            q = msg.pose.pose.orientation
            import math
            from tf_transformations import euler_from_quaternion
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Write to CSV
            self.ekf_writer.writerow([timestamp, x, y])
            self.ekf_file.flush()  # Immediately write to disk
            
            # Add to buffer (for visualization)
            with self.buffer_lock:
                self.ekf_buffer.append((timestamp, x, y))
            
            self.ekf_count += 1
            
            # Periodic log (every 100 samples)
            if self.ekf_count % 100 == 0:
                self.get_logger().info(
                    f'[UWB_DATA] EKF: {self.ekf_count} samples | '
                    f'Latest: ({x:.3f}, {y:.3f})'
                )
                
        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error in ekf_callback: {e}')
    
    # PyQtGraph Visualization
    def setup_plot(self):
        self.get_logger().info('[UWB_DATA] Initializing PyQtGraph window...')
        try:
            self.app = QtWidgets.QApplication.instance()
            if self.app is None:
                self.app = QtWidgets.QApplication(sys.argv)

            # Main window
            self.win = pg.GraphicsLayoutWidget(show=True, title="UWB vs EKF Data")
            self.win.resize(1200, 800)
            self.win.setWindowTitle('UWB Data Logger - Real-time Visualization')

            # Plot
            self.plot = self.win.addPlot(title="XY Position")
            self.plot.setLabel('left', 'Y Position', units='m')
            self.plot.setLabel('bottom', 'X Position', units='m')
            self.plot.setAspectLocked(True)
            self.plot.addLegend()
            self.plot.showGrid(x=True, y=True, alpha=0.3)

            # Curves
            self.uwb_curve = self.plot.plot(pen=pg.mkPen('r', width=2), name='UWB',
                                            symbol='o', symbolSize=5, symbolBrush='r')
            self.ekf_curve = self.plot.plot(pen=pg.mkPen('b', width=2), name='EKF (Kalman)',
                                            symbol='x', symbolSize=5, symbolBrush='b')

            # Timer for updates (runs in Qt event loop thread)
            self.timer = QtCore.QTimer()
            self.timer.timeout.connect(self._update_plot)
            self.timer.start(100)
            self.get_logger().info('[UWB_DATA] PyQtGraph window initialized')
        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error initializing plot: {e}')
    
    def _update_plot(self):
        """Update the plot"""
        try:
            with self.buffer_lock:
                # UWB data
                if self.uwb_buffer:
                    uwb_data = list(self.uwb_buffer)
                    uwb_x = [d[1] for d in uwb_data]
                    uwb_y = [d[2] for d in uwb_data]
                    self.uwb_curve.setData(uwb_x, uwb_y)
                
                # EKF data
                if self.ekf_buffer:
                    ekf_data = list(self.ekf_buffer)
                    ekf_x = [d[1] for d in ekf_data]
                    ekf_y = [d[2] for d in ekf_data]
                    self.ekf_curve.setData(ekf_x, ekf_y)
            
            # Update window title with stats
            if hasattr(self, 'win'):
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
                self.win.setWindowTitle(
                    f'UWB Data Logger - UWB: {self.uwb_count} | EKF: {self.ekf_count} | '
                    f'Time: {elapsed:.1f}s'
                )
                
        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error updating plot: {e}')
    
    # Shutdown
    def cleanup(self):
        """Safe shutdown procedure"""
        self.get_logger().info('[UWB_DATA] Cleaning up...')
        
        # Print statistics
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.get_logger().info(
            f'[UWB_DATA] Session statistics:\n'
            f'  - Duration: {elapsed:.1f} s\n'
            f'  - UWB samples: {self.uwb_count}\n'
            f'  - EKF samples: {self.ekf_count}\n'
            f'  - UWB rate: {self.uwb_count/elapsed:.1f} Hz\n'
            f'  - EKF rate: {self.ekf_count/elapsed:.1f} Hz'
        )
        
        # Close CSV files
        try:
            self.uwb_file.close()
            self.ekf_file.close()
            self.get_logger().info('[UWB_DATA] CSV files closed successfully')
        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error closing files: {e}')
        
        # Close PyQtGraph
        if PYQTGRAPH_AVAILABLE and self.app:
            try:
                self.app.quit()
                self.get_logger().info('[UWB_DATA] Plot window closed')
            except Exception as e:
                self.get_logger().error(f'[UWB_DATA] Error closing plot: {e}')
        
        self.get_logger().info('[UWB_DATA] Cleanup complete')


def main(args=None):
    # If PyQtGraph is available, create QApplication in the main thread first
    app = None
    if PYQTGRAPH_AVAILABLE:
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    rclpy.init(args=args)
    node = None
    
    try:
        node = UWBDataLogger()
        
        # If plotting requested, initialize widgets now (QApplication exists)
        if PYQTGRAPH_AVAILABLE:
            node.setup_plot()
            
            # Run rclpy in a background thread; keep Qt event loop in main thread
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()
            
            # Run Qt event loop (blocks until window closed)
            app.exec()
        else:
            # if no GUI, just run normally
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[UWB_DATA] Fatal error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
