#!/usr/bin/env python3
"""Pure Pursuit 라인 추종(line following) 주행을 실시간으로 시각화하고,
주행 품질 지표(CTE, heading error, TTR)를 계산·기록하는 노드.

## 역할
- `/waypoints_path`(경로)와 `/odometry/ekf_single`(현재 위치), PPC 컨트롤러가
  내는 `/ppc/*` 토픽들(lookahead point, state, cte, heading_error 등)을
  구독해 하나의 matplotlib 창에 실시간으로 그린다 — 경로/실제 주행 궤적,
  현재 로봇 위치·자세, lookahead 목표점, CTE·heading error 시계열 그래프.
- CTE(횡방향 오차)와 heading error(진행방향 오차)를 각각 이동평균 필터로
  스무딩한 뒤, 두 값이 임계치를 벗어났다가 회복하는 과정을 추적하는
  TTR(Time To Recover) 상태머신을 직접 굴린다 — 라인을 벗어난 뒤 다시
  기준 안으로 돌아오기까지 걸린 시간을 정량 지표로 남기기 위함.
- `/ppc/enable` 신호로 미션 시작/일시정지/재개를 구분한다: 새 미션
  시작 시에만 누적 데이터를 초기화하고, 일시정지 후 재개 시에는 기존
  데이터를 보존한다(같은 미션 안에서의 끊김을 데이터 유실로 취급하지
  않기 위함).
- `/ppc/waypoint_idx`가 마지막 waypoint에 도달하면 미션 완료로 간주해
  주행 궤적·오차 시계열·TTR 이벤트 전체를 JSON 파일로 저장한다
  (`~/ppc_run_data_real_v2/`) — 전략(strategy)별 성능 비교 분석 자료.

## 클래스 구성
- `PlotPPC2`: 구독, TTR 판정, matplotlib 플롯(`FuncAnimation`), 결과
  저장까지 전부 담당하는 단일 노드 클래스. 별도 보조 클래스는 없다.

## main()의 동작 순서
1. rclpy 초기화, `PlotPPC2` 노드 생성 → 구독 시작 및 플롯 창 구성
2. `rclpy.spin()`을 별도 데몬 스레드로 실행 — ROS 콜백은 백그라운드에서 처리
3. 메인 스레드에서 `plt.show()` 호출 — matplotlib GUI 이벤트 루프는
   메인 스레드에서 돌아야 하므로 스핀과 분리(창이 닫힐 때까지 블로킹)
4. 창이 닫히면 노드 파괴 및 rclpy 종료
"""

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
    """라인 추종 PPC 주행의 실시간 시각화 + CTE/heading/TTR 지표 산출 노드."""

    def __init__(self):
        super().__init__('ppc_plotter_v2')
        
        # 파라미터: 실행 중인 모터 제어 스크립트 이름과 lookahead 거리는
        # launch 인자로 주입됨 — 저장되는 run 데이터에 메타정보로 남겨
        # 전략(strategy)별 성능을 나중에 구분·비교할 수 있게 한다.
        self.declare_parameter('motor_script', 'motor_cmd_vel_sim_2.py')
        self.declare_parameter('lookahead_distance', 1.5)

        self.motor_script = self.get_parameter('motor_script').get_parameter_value().string_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value

        # 실행 파일명 -> 사람이 읽을 전략 이름. 시뮬/실기 스크립트가 이름은
        # 다르지만 같은 제어 전략을 구현하는 경우가 있어 여기서 통일한다.
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

        # 구독: 경로/현재위치/제어명령 + PPC 컨트롤러가 내는 상태·오차 신호들
        self.create_subscription(Path, '/waypoints_path', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_ppc', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/ppc/enable', self.enable_callback, 10)
        self.create_subscription(PoseStamped, '/ppc/lookahead_point', self.lookahead_callback, 10)
        self.create_subscription(String, '/ppc/state', self.state_callback, 10)
        self.create_subscription(Int32, '/ppc/waypoint_idx', self.waypoint_idx_callback, 10)
        self.create_subscription(Float64, '/ppc/heading_error', self.heading_error_callback, 10)
        self.create_subscription(Float64, '/ppc/cte', self.cte_callback, 10)

        # 현재 미션의 상태(경로/궤적/자세 등) — 플롯이 매 프레임 읽어가는 값들
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

        # CTE/heading error 시계열 — save_run_data()로 저장되고 우측 그래프에도 쓰임
        self.time_stamps = deque()
        self.cte_values = deque()
        self.heading_errors = deque()
        self.start_time = None      # 이 미션에서 첫 odom 수신 시각(경과시간 t=0 기준)
        self.run_start_time = None  # PPC가 enable된 시각(run_duration 계산용)

        # TTR(Time To Recover) 판정 임계치 — 값 두 쌍(OUT/IN)으로 히스테리시스를
        # 둬서, 경계선 근처에서 값이 미세하게 흔들릴 때 상태가 들쑥날쑥
        # 전이되는 것(chattering)을 막는다.
        self.CTE_OUT = 0.20      # 이 이상이면 "이탈"로 판정(m)
        self.CTE_IN = 0.08       # 이 이하로 돌아와야 "회복 시작"으로 인정(m)
        self.HE_OUT = 20.0       # 이 이상이면 "이탈"로 판정(도)
        self.HE_IN = 12.0        # 이 이하로 돌아와야 "회복 시작"으로 인정(도)
        self.T_HOLD = 0.5        # 회복 조건을 이 시간(초) 이상 유지해야 회복 확정
        self.TTR_TIMEOUT = 10.0  # 이탈 후 이 시간(초) 안에 회복 못하면 실패 처리
        self.TTR_COOLDOWN = 0.5  # 이벤트 종료 직후 이 시간(초)은 새 이벤트 판정을 쉼(연쇄 이벤트 방지)

        # TTR 상태머신 변수 — 자세한 전이 로직은 _update_ttr() 참고
        self.ttr_state = "normal"  # "normal"(정상) / "off track"(이탈) / "recovering"(회복중)
        self.t_out = None          # 이탈이 시작된 시각
        self.recovery_start = None # 회복 조건이 처음 충족된 시각
        self.last_ttr_event = None # 가장 최근 이벤트 종료 시각(쿨다운 기준점)

        # 이탈 시점의 로봇 위치 — 저장 후 지도 위에 이탈 지점을 표시하는 데 사용
        self.pose_at_t_out = None

        # 완료된(성공/실패) TTR 이벤트 기록 리스트 — save_run_data()로 그대로 저장됨
        self.ttr_events = []

        # 센서 노이즈로 인한 TTR 오탐을 줄이기 위한 이동평균 필터(3샘플)
        self.cte_filter = deque(maxlen=3)
        self.he_filter = deque(maxlen=3)

        self.setup_plot()

        self.get_logger().info('[PLOT_PPC_V2] Line Following PlotPPC initialized')

    def waypoints_callback(self, msg: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) 
                         for pose in msg.poses]
        self.get_logger().info(f'[PLOT_PPC_V2] Received {len(self.waypoints)} waypoints')

    def odom_callback(self, msg: Odometry):
        """EKF 위치 추정 콜백 — 현재 위치/자세 갱신 및 활성 상태일 때 궤적 기록.

        PPC가 꺼져 있으면(`is_enabled=False`) 로봇 표시 자체는 계속 갱신하되
        `actual_path`에는 쌓지 않는다 — 대기 중 흔들림까지 주행 궤적으로
        기록되는 것을 막기 위함. 이번 미션의 첫 유효 odom 시점을
        `start_time`(경과시간 t=0 기준)으로 잡는다.
        """
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
        """PPC가 낸 속도 명령을 저장 — 현재는 정보 패널에는 쓰이지 않고 보관만 함."""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def enable_callback(self, msg: Bool):
        """PPC 활성화 상태 변화 콜백 — 새 미션 시작과 일시정지 후 재개를 구분한다.

        - OFF -> ON 이면서 "이전 미션이 완료됐거나 아직 한 번도 시작 안 함"
          (`was_mission_completed` 또는 `start_time is None`)인 경우에만
          새 미션으로 보고 모든 누적 데이터(궤적/시계열/TTR 상태)를 초기화한다.
        - OFF -> ON인데 이전 미션이 완료되지 않았다면 "일시정지 후 재개"로
          보고 기존 데이터를 그대로 이어서 쌓는다.
        - ON -> OFF는 일시정지로만 취급하고 아무것도 지우지 않는다.
        """
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
                # 일시정지 후 재개 - 기존 데이터 보존
                self.get_logger().info('[PLOT_PPC_V2] PPC RESUMED - Continuing data recording')

        elif not msg.data and self.is_enabled:
            self.get_logger().info('[PLOT_PPC_V2] PPC PAUSED - Data preserved')

        self.is_enabled = msg.data

    # From PPC
    def lookahead_callback(self, msg: PoseStamped):
        """PPC가 현재 조준 중인 lookahead 목표점 좌표를 저장."""
        self.lookahead_point = (msg.pose.position.x, msg.pose.position.y)

    def state_callback(self, msg: String):
        """PPC 컨트롤러의 현재 상태 문자열(예: "move")을 저장.
        TTR 판정 시 이동 중인지 여부에 따라 heading error 기준 적용 여부가 달라진다.
        """
        self.current_state = msg.data

    def waypoint_idx_callback(self, msg: Int32):
        """현재 목표 waypoint 인덱스 갱신 + 미션 완료 감지.

        완료 판정 기준은 "인덱스가 마지막 waypoint(len-1)에 도달"이다
        (PPC 컨트롤러 쪽에서 도착 처리 시 인덱스를 마지막 값으로 못박아
        두기 때문 — 여기서는 그 값을 그대로 신뢰해 감지만 한다).
        인덱스가 실제로 "방금 바뀐" 순간에만, 그리고 아직 저장 안 했고
        PPC가 켜져 있을 때만 저장을 트리거한다 — 콜백이 여러 번 와도
        저장이 중복되지 않도록 하기 위함.
        """
        old_idx = self.current_waypoint_idx
        self.current_waypoint_idx = msg.data

        if self.waypoints and self.current_waypoint_idx == len(self.waypoints) - 1:
            if old_idx != self.current_waypoint_idx and not self.was_mission_completed and self.is_enabled:
                self.was_mission_completed = True
                self.save_run_data()
                self.get_logger().info('[PLOT_PPC_V2] Mission completed - Data saved')

    def heading_error_callback(self, msg: Float64):
        """heading error 콜백 — 이동평균 필터링 후 시계열에 적재하고 TTR 판정을 실행.

        시간축(`time_stamps`)도 이 콜백에서 함께 채운다 — heading error가
        (cte보다) 이 노드에 필요한 시계열의 기준 tick 역할을 한다.
        상태가 "move"일 때만 TTR을 갱신하는 이유: 정차/회전 등 다른 상태에서는
        경로 추종 이탈이라는 개념 자체가 성립하지 않기 때문
        (다만 CTE_OUT 단독 기준은 _update_ttr 내부에서 move 외 상태에도 적용됨).
        """
        he_value = msg.data

        # 이동평균 필터 적용(노이즈 스무딩)
        self.he_filter.append(he_value)
        he_filtered = sum(self.he_filter) / len(self.he_filter)

        if self.is_enabled and self.start_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            self.time_stamps.append(elapsed)
            self.heading_errors.append(he_filtered)

            if self.current_state == "move":
                self._update_ttr(elapsed, he_filtered)

    def cte_callback(self, msg: Float64):
        """CTE(횡방향 오차) 콜백 — 이동평균 필터링 후 값만 저장한다.

        TTR 판정은 이 콜백에서 직접 하지 않고 heading_error_callback이
        `_update_ttr` 호출 시 `cte_values[-1]`(가장 최근 값)을 읽어가는
        방식으로 간접 참조한다 — 두 값이 서로 다른 주기로 들어와도
        TTR 판정 시점을 하나로 통일하기 위함.
        """
        cte_value = msg.data

        self.cte_filter.append(cte_value)
        cte_filtered = sum(self.cte_filter) / len(self.cte_filter)

        if self.is_enabled and self.start_time is not None:
            self.cte_values.append(cte_filtered)

    def _update_ttr(self, current_time, heading_error):
        """TTR(Time To Recover) 상태머신의 한 스텝을 진행한다.

        "normal -> off track -> recovering -> normal" 3단계 순환 구조:
        - normal: 정상 추종 중. CTE/heading error가 OUT 임계치를 넘으면 이탈.
        - off track: 이탈 상태. TTR_TIMEOUT 안에 IN 임계치로 돌아오지 못하면
          실패 이벤트로 기록하고 normal로 복귀. IN 임계치 충족 즉시 recovering 진입.
        - recovering: 회복 조건이 T_HOLD(초) 이상 유지되면 성공 이벤트로 기록.
          유지 도중 다시 임계치를 벗어나면 off track으로 되돌아간다(재도전).

        Args:
            current_time: 미션 시작 기준 경과 시간(초) — 이벤트 타임스탬프로 기록.
            heading_error: 필터링된 heading error(도). CTE는 `self.cte_values[-1]`에서 읽음.
        """
        if not self.cte_values:
            return

        current_cte = self.cte_values[-1]

        # 쿨다운: 직전 이벤트 종료 직후엔 새 판정을 쉬어 연쇄 이벤트를 방지
        if self.last_ttr_event is not None:
            if current_time - self.last_ttr_event < self.TTR_COOLDOWN:
                return

        # 상태머신 본체
        
        if self.ttr_state == "normal":
            # 이탈 감지: "move" 상태에서는 CTE·heading error 둘 중 하나라도
            # 넘으면 이탈, 그 외 상태에서는 CTE만으로 판정
            # (heading error 기준은 이동 중 진행방향 정렬에만 의미가 있음).
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
            # 타임아웃 검사: 이탈 후 TTR_TIMEOUT초 안에 회복 조건을 못 만나면 실패로 종결
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

            # 회복 조건 검사: CTE/heading error 둘 다 IN 임계치 이하로
            # 들어오면 회복 시도가 시작된 것으로 보고 recovering으로 전이
            if current_cte <= self.CTE_IN and abs(heading_error) <= self.HE_IN:
                if self.recovery_start is None:
                    self.recovery_start = current_time
                    self.ttr_state = "recovering"
                    self.get_logger().info(
                        f'[TTR] Recovery started at t={current_time:.2f}s'
                    )

        elif self.ttr_state == "recovering":
            # 회복 조건 위반 검사: 유지 도중 다시 임계치를 벗어나면
            # off track으로 되돌리고 hold 타이머를 리셋(재도전)
            if current_cte > self.CTE_IN or abs(heading_error) > self.HE_IN:
                self.get_logger().info(
                    f'[TTR] Recovery interrupted at t={current_time:.2f}s - Resetting hold timer'
                )
                self.recovery_start = None
                self.ttr_state = "off track"
                return

            # 유지 시간 검사: T_HOLD초 이상 회복 조건을 유지해야 성공 확정
            # (짧은 순간 스치듯 조건을 만족한 것을 성공으로 오판하지 않기 위함)
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
        """matplotlib 창의 레이아웃(경로/정보패널/CTE/heading 4분할)과
        모든 그래프 요소(line, marker)를 미리 생성해둔다.

        여기서는 빈 데이터로 각 Artist(line2d)만 만들고, 실제 값 채우기는
        매 프레임 `update_plot()`이 담당한다 — `FuncAnimation`이 이 함수가
        아니라 `update_plot`을 주기적으로 호출하는 구조이기 때문에
        객체 생성은 한 번만, 갱신은 반복적으로 분리한 것.
        """
        self.fig = plt.figure(figsize=(18, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[3, 1],
                                  hspace=0.45, wspace=0.3)

        # 좌측 전체: 경로/궤적 지도
        self.ax_path = self.fig.add_subplot(gs[:, 0])
        self.ax_path.set_title('Line Following Pure Pursuit', fontsize=14, fontweight='bold')
        self.ax_path.set_xlabel('X (m)', fontsize=11)
        self.ax_path.set_ylabel('Y (m)', fontsize=11)
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.set_xlim(0, 0)
        self.ax_path.set_ylim(0, 0)
        self.ax_path.set_aspect('equal', adjustable='box')
        
        # 우측 상단: 텍스트 정보 패널(모드/상태/lookahead 등)
        self.ax_info = self.fig.add_subplot(gs[0, 1])
        self.ax_info.set_title('Control Info', fontsize=11, fontweight='bold', pad=10)
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.5, 'Waiting...', fontsize=9,
                                            verticalalignment='center',
                                            family='monospace',
                                            linespacing=1.3)

        # 우측 중단: CTE(횡방향 오차) 시계열
        self.ax_cte = self.fig.add_subplot(gs[1, 1])
        self.ax_cte.set_title('Cross-Track Error', fontsize=11, fontweight='bold', pad=10)
        self.ax_cte.set_xlabel('Time (s)', fontsize=9)
        self.ax_cte.set_ylabel('CTE (m)', fontsize=9)
        self.ax_cte.grid(True, alpha=0.3)
        self.ax_cte.tick_params(labelsize=8)

        # 우측 하단: heading error(진행방향 오차) 시계열
        self.ax_heading = self.fig.add_subplot(gs[2, 1])
        self.ax_heading.set_title('Heading Error', fontsize=11, fontweight='bold', pad=10)
        self.ax_heading.set_xlabel('Time (s)', fontsize=9)
        self.ax_heading.set_ylabel('Error (deg)', fontsize=9)
        self.ax_heading.grid(True, alpha=0.3)
        self.ax_heading.tick_params(labelsize=8)

        # 경로 지도 위 그래프 요소들 — 전부 빈 데이터로 생성 후 update_plot()에서 채움
        self.waypoints_line, = self.ax_path.plot([], [], 'r--', linewidth=2,
                                                  label='Waypoints', marker='o', markersize=6)

        # 현재 목표로 하는 구간(직전 WP -> 목표 WP)만 굵게 강조
        self.active_segment_line, = self.ax_path.plot([], [], 'm-', linewidth=4,
                                                       label='Active Line', alpha=0.7, zorder=3)

        self.robot_marker, = self.ax_path.plot([], [], 'go', markersize=10, label='Robot')

        self.ttr_deviate_markers, = self.ax_path.plot([], [], 'rx', markersize=12,
                                                       markeredgewidth=2, label='Deviate', zorder=10)

        self.robot_arrow = None  # 로봇 방향 화살표 — matplotlib arrow는 매 프레임 재생성/제거해야 함

        self.actual_line, = self.ax_path.plot([], [], 'b-', linewidth=2,
                                               label='Actual Path', alpha=0.7)

        self.lookahead_marker, = self.ax_path.plot([], [], 'co', markersize=10,
                                                    label='Lookahead', zorder=5)

        self.lookahead_line, = self.ax_path.plot([], [], 'c--', linewidth=2, alpha=0.5)

        # TTR 이벤트 지점 마커(이탈 지점 X, 회복 지점 +)
        self.ttr_recover_markers, = self.ax_path.plot([], [], 'g+', markersize=14,
                                                       markeredgewidth=2, label='Recover', zorder=10)

        self.cte_line, = self.ax_cte.plot([], [], 'r-', linewidth=2)
        self.heading_line, = self.ax_heading.plot([], [], 'b-', linewidth=2)

        self.ax_path.legend(loc='upper right', fontsize=7, ncol=2)

        # 100ms 주기로 update_plot을 호출하는 애니메이션 등록.
        # blit=False: 배경/화살표/텍스트 등 매 프레임 크게 바뀌는 요소가
        # 많아 partial redraw(blit) 최적화의 이득이 적고 오히려 버그 소지가 큼.
        # cache_frame_data=False: 데이터가 실시간으로 계속 늘어나므로
        # 프레임 캐싱은 무의미하고 메모리만 낭비됨.
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100,
                                blit=False, cache_frame_data=False)

    def update_plot(self, frame):
        """`FuncAnimation`이 100ms마다 호출하는 프레임 갱신 함수.

        경로/궤적/로봇 위치·자세/lookahead/TTR 마커/CTE·heading 그래프까지
        이 창의 모든 시각 요소를 이 한 함수에서 순서대로 갱신한다.
        `frame` 인자는 FuncAnimation 표준 콜백 시그니처상 필요하지만
        내부에서는 사용하지 않는다(매번 최신 상태를 그대로 다시 그리는 방식).
        """
        if self.waypoints:
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            self.waypoints_line.set_data(wp_x, wp_y)
        
        # 활성 구간 강조(직전 WP → 목표 WP)
        if self.waypoints and 1 <= self.current_waypoint_idx < len(self.waypoints):
            p1 = self.waypoints[self.current_waypoint_idx - 1]
            p2 = self.waypoints[self.current_waypoint_idx]
            self.active_segment_line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        else:
            self.active_segment_line.set_data([], [])

        # 실제 주행 궤적
        if self.actual_path:
            actual_path_copy = list(self.actual_path)
            if actual_path_copy:
                path_x = [p[0] for p in actual_path_copy]
                path_y = [p[1] for p in actual_path_copy]
                self.actual_line.set_data(path_x, path_y)

        # 지도 축 범위를 데이터에 맞춰 자동 확장
        self._auto_adjust_limits()

        # TTR 이탈/회복 지점 마커 좌표 수집(완료된 이벤트 전체 + 현재 진행 중인 이탈)
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

        # 로봇 위치/자세 — 화살표는 set_data로 갱신 불가능한 Patch라서
        # 매 프레임 이전 것을 지우고 새로 그려야 함
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

        # lookahead 목표점 및 로봇-목표점 연결선, 우측 정보 패널 텍스트 갱신
        if self.lookahead_point and self.current_pose:
            lx, ly = self.lookahead_point
            rx, ry = self.current_pose
            
            self.lookahead_marker.set_data([lx], [ly])
            self.lookahead_line.set_data([rx, lx], [ry, ly])
            
            ld = math.hypot(lx - rx, ly - ry)
            
            # 현재 활성 구간 텍스트
            if self.waypoints and 1 <= self.current_waypoint_idx < len(self.waypoints):
                p1 = self.waypoints[self.current_waypoint_idx - 1]
                p2 = self.waypoints[self.current_waypoint_idx]
                segment_info = f'Seg: WP{self.current_waypoint_idx-1}→WP{self.current_waypoint_idx}'
            else:
                segment_info = 'Seg: N/A'

            # 정보 패널 텍스트 갱신
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

        # CTE 그래프 — 최근 30초 구간만 x축에 표시(오래된 구간은 스크롤아웃),
        # 구간이 5초 미만이면 최소 폭 5초를 강제해 그래프가 지나치게 좁아지지 않게 함
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

        # heading error 그래프 — x축 범위 로직은 CTE와 동일, y축은 좌우
        # 대칭(±h_max)으로 잡아 부호 있는 오차의 변화가 한눈에 보이게 함
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

        # 창 제목: 현재 상태를 한눈에 요약(ON/OFF, PPC 상태, TTR 상태,
        # 누적 이벤트 수, 최신 CTE/heading). TTR 상태에 따라 제목 색도 바꿔
        # 이탈(빨강)/회복중(주황)을 시각적으로 바로 알아볼 수 있게 함.
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
        """경로 지도(`ax_path`)의 축 범위를 waypoint + 실제 궤적 데이터에 맞춰 넓힌다.

        기존 축 범위와 비교해 "더 넓히기만" 하고 절대 좁히지 않는다
        (new_min = min(계산값, 현재값), new_max = max(계산값, 현재값)) —
        로봇이 지나온 뒤에도 이미 본 구간이 시야에서 갑자기 잘려나가
        지도가 튀는(jarring) 현상을 막기 위함.
        """
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
        """누적된 TTR 이벤트 리스트로부터 요약 통계를 계산한다(저장 데이터에 포함됨).

        평균/최대/최소 TTR은 "성공한" 이벤트의 TTR 값만으로 계산한다 —
        실패(timeout) 이벤트는 회복 소요시간 자체가 정의되지 않으므로
        (TTR 필드가 None) 통계에 섞으면 의미가 왜곡되기 때문.
        """
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
        """미션 완료 시(`waypoint_idx_callback`에서 호출) 이번 run 전체를 JSON으로 저장.

        - 파일명에 타임스탬프를 넣어 실행마다 별도 파일로 남긴다.
        - `metadata`에 전략/lookahead 거리/모터 스크립트를 함께 저장해,
          여러 run 파일을 모아 전략별 성능(CTE/TTR)을 비교 분석할 수 있게 한다.
        - 저장 실패(디스크 오류 등)해도 노드 자체는 죽지 않도록 예외를 흡수하고 로그만 남긴다.
        """
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
    """진입점. 각 단계 의미는 모듈 docstring의 "main()의 동작 순서" 참고."""
    rclpy.init(args=args)
    plotter = PlotPPC2()

    # rclpy 콜백은 백그라운드 스레드로 돌리고, matplotlib GUI 이벤트 루프는
    # 메인 스레드에서 plt.show()로 실행한다 — matplotlib GUI 백엔드는
    # 메인 스레드 밖에서 돌리면 불안정하기 때문(플랫폼에 따라 크래시 가능).
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    spin_thread.start()

    plt.show()  # 창이 닫힐 때까지 블로킹

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
