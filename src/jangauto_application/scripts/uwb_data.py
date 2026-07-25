#!/usr/bin/env python3
"""UWB 측위값과 EKF(칼만필터) 추정값을 동시에 CSV로 로깅하고,
필요 시 PyQtGraph로 두 궤적을 실시간 비교 플로팅하는 노드.

## 역할
- `/abs_xy_fixed`(UWB 원시 측위, PoseWithCovarianceStamped)와
  `/odometry/ekf_single`(EKF로 융합된 추정치, Odometry)을 각각 구독해
  타임스탬프 있는 CSV 두 개(uwb_*.csv, ekf_*.csv)로 저장한다 —
  UWB 원신호와 필터링된 결과를 나란히 비교 분석하기 위한 목적.
- 실행 시점 타임스탬프로 매 실행마다 새 CSV 쌍을 만들어
  `~/uwb_data/uwb/`, `~/uwb_data/kalman/`에 각각 저장한다.
- pyqtgraph가 설치돼 있으면 최근 1000개 포인트를 슬라이딩 버퍼에 담아
  두 궤적(XY 평면)을 실시간 창으로 시각화한다. 미설치 시 시각화 없이
  CSV 로깅만 수행하는 fallback 모드로 동작한다(선택적 의존성).
- 콜백 스레드(rclpy)와 Qt 이벤트 루프 스레드가 공유하는 버퍼는
  `buffer_lock`으로 보호한다.

## 클래스 구성
- `UWBDataLogger`: 구독/CSV 기록/버퍼 관리/PyQtGraph 시각화를 모두
  담당하는 단일 노드 클래스. 시각화 위젯 생성은 `__init__`이 아니라
  별도 `setup_plot()`에서 이루어진다 — QApplication은 반드시 메인
  스레드에서 만들어야 하는데, 노드 생성 시점에는 아직 QApplication이
  없을 수 있기 때문(아래 main() 참고).

## main()의 동작 순서
1. pyqtgraph 사용 가능하면 QApplication을 메인 스레드에서 먼저 생성
2. rclpy 초기화, `UWBDataLogger` 노드 생성 → 두 토픽 구독 및 CSV 파일 오픈
3. pyqtgraph 사용 가능하면:
   a. `setup_plot()` 호출 — 이제 QApplication이 존재하므로 안전
   b. `rclpy.spin()`을 백그라운드 스레드로 돌려 ROS 콜백 처리
   c. 메인 스레드는 Qt 이벤트 루프(`app.exec()`)를 실행 — 창이 닫힐 때까지 블로킹
4. pyqtgraph 없으면 메인 스레드에서 바로 `rclpy.spin()`
5. 종료 시(Ctrl+C/예외 무관) `cleanup()`으로 통계 로그 출력, CSV 파일
   닫기, Qt 앱 종료 후 노드 파괴 및 rclpy 종료
"""

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
    """UWB/EKF 위치 데이터를 CSV로 기록하고 선택적으로 실시간 비교 플롯을 띄우는 노드."""

    def __init__(self):
        super().__init__('uwb_data_logger')
        
        self.get_logger().info('[UWB_DATA] Node starting...')

        # 저장 경로: UWB/EKF를 별도 하위 폴더로 분리 저장
        self.base_dir = os.path.expanduser('~/uwb_data')
        self.uwb_dir = os.path.join(self.base_dir, 'uwb')
        self.kalman_dir = os.path.join(self.base_dir, 'kalman')

        os.makedirs(self.uwb_dir, exist_ok=True)
        os.makedirs(self.kalman_dir, exist_ok=True)

        self.get_logger().info(f'[UWB_DATA] Data directory: {self.base_dir}')

        # 실행 시각을 파일명에 넣어 매 실행마다 새 CSV 쌍 생성(덮어쓰기 방지)
        timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')

        self.uwb_file_path = os.path.join(self.uwb_dir, f'uwb_{timestamp}.csv')
        self.ekf_file_path = os.path.join(self.kalman_dir, f'ekf_{timestamp}.csv')

        # CSV 파일은 노드 수명 내내 열어두고 콜백마다 append + flush한다
        # (아래 uwb_callback/ekf_callback 참고 — 중간에 죽어도 데이터 유실 최소화)
        self.uwb_file = open(self.uwb_file_path, 'w', newline='')
        self.ekf_file = open(self.ekf_file_path, 'w', newline='')

        self.uwb_writer = csv.writer(self.uwb_file)
        self.ekf_writer = csv.writer(self.ekf_file)

        self.uwb_writer.writerow(['timestamp', 'x', 'y'])
        self.ekf_writer.writerow(['timestamp', 'x', 'y'])

        self.get_logger().info(f'[UWB_DATA] UWB file: {self.uwb_file_path}')
        self.get_logger().info(f'[UWB_DATA] EKF file: {self.ekf_file_path}')

        # 시각화용 버퍼 — CSV에는 전체 이력을 다 쌓지만, 화면에는 최근
        # 1000개만 보여줘도 궤적 비교 목적엔 충분하고 렌더 부하도 억제됨
        max_points = 1000
        self.uwb_buffer = deque(maxlen=max_points)
        self.ekf_buffer = deque(maxlen=max_points)

        # rclpy 콜백 스레드와 Qt 타이머(_update_plot)가 버퍼를 동시에
        # 건드릴 수 있어 락으로 보호한다.
        self.buffer_lock = threading.Lock()

        self.create_subscription(PoseWithCovarianceStamped,'/abs_xy_fixed',self.uwb_callback,10)
        self.create_subscription(Odometry,'/odometry/ekf_single',self.ekf_callback,10)

        self.get_logger().info('[UWB_DATA] Subscribed to /abs_xy and /odometry/ekf_single')

        self.uwb_count = 0
        self.ekf_count = 0
        self.start_time = self.get_clock().now()

        # QApplication은 반드시 메인 스레드에서 생성해야 하므로, 여기서는
        # 위젯을 만들지 않고 자리만 잡아둔다 — 실제 생성은 main()이
        # QApplication을 만든 뒤 호출하는 setup_plot()에서 이루어진다.
        self.app = None
        self.win = None
        self.timer = None

        if PYQTGRAPH_AVAILABLE:
            # setup_plot()은 main()에서 QApplication 생성 후 호출됨
            pass
        else:
            self.get_logger().warn('[UWB_DATA] Running in CSV-only mode (no visualization)')

        # Qt와 함께 쓸 때는 signal 핸들러를 직접 걸지 않고 Qt/rclpy의
        # 기본 종료 처리에 맡긴다(충돌 방지).
        # signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info('[UWB_DATA] Node initialized successfully')
    
    # Callbacks
    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        """UWB 원시 측위 콜백 — CSV 기록 + 시각화 버퍼 적재를 함께 수행.

        z는 UWB 측위 특성상 참고용으로만 읽고 CSV/버퍼에는 XY만 남긴다
        (2D 실내/실외 측위 비교가 목적이라 고도는 저장 대상이 아님).
        예외를 여기서 잡아 로그만 남기는 이유는, 구독 콜백에서 예외가
        새어나가면 rclpy 스핀 자체가 죽을 수 있기 때문(노드 전체 다운 방지).
        """
        try:
            # ROS 헤더 타임스탬프 사용 — 콜백 실행 시각이 아니라 메시지가
            # 실제로 발행된 시각을 기록해야 UWB/EKF 두 CSV를 나중에 정확히 정렬 가능.
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            self.uwb_writer.writerow([timestamp, x, y])
            self.uwb_file.flush()  # 중간에 노드가 죽어도 이미 쓴 줄은 보존

            with self.buffer_lock:
                self.uwb_buffer.append((timestamp, x, y))

            self.uwb_count += 1

            # 매 샘플 로깅은 스팸이라 100개마다 한 번만 진행 상황 출력
            if self.uwb_count % 100 == 0:
                self.get_logger().info(
                    f'[UWB_DATA] UWB: {self.uwb_count} samples | '
                    f'Latest: ({x:.3f}, {y:.3f})'
                )

        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error in uwb_callback: {e}')

    def ekf_callback(self, msg: Odometry):
        """EKF(칼만필터) 추정 위치 콜백 — CSV 기록 + 시각화 버퍼 적재.

        구조는 `uwb_callback`과 동일(같은 이유로 예외를 콜백 내부에서 흡수).
        yaw는 계산만 하고 CSV/버퍼에는 쓰지 않는다 — 이 노드의 목적은
        UWB와 EKF의 XY 궤적 비교이며, 자세(방향)는 대상이 아니기 때문
        (다른 로깅 노드에서 필요 시 별도로 다룸).
        """
        try:
            stamp = msg.header.stamp
            timestamp = stamp.sec + stamp.nanosec * 1e-9

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            q = msg.pose.pose.orientation
            import math
            from tf_transformations import euler_from_quaternion
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.ekf_writer.writerow([timestamp, x, y])
            self.ekf_file.flush()

            with self.buffer_lock:
                self.ekf_buffer.append((timestamp, x, y))

            self.ekf_count += 1

            if self.ekf_count % 100 == 0:
                self.get_logger().info(
                    f'[UWB_DATA] EKF: {self.ekf_count} samples | '
                    f'Latest: ({x:.3f}, {y:.3f})'
                )

        except Exception as e:
            self.get_logger().error(f'[UWB_DATA] Error in ekf_callback: {e}')
    
    # PyQtGraph Visualization
    def setup_plot(self):
        """PyQtGraph 창/커브/갱신 타이머를 생성한다.

        `__init__`이 아니라 main()에서 QApplication 생성 후 별도로
        호출되는 이유는 클래스 설명 참고. QTimer가 100ms마다 `_update_plot`을
        호출하며 이게 곧 이 노드의 화면 갱신 주기(=플롯 프레임레이트)다.
        """
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
        """QTimer가 주기적으로 호출하는 화면 갱신 함수.

        버퍼 읽기(list 변환)와 커브 갱신(`setData`)을 같은 락 구간 안에서
        수행해, 콜백 스레드가 그리는 도중에 버퍼를 변경하지 못하게 한다
        (deque 자체는 append/iterate 동시성에 안전하지 않으므로 필요).
        """
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
        """정상/비정상 종료 시 공통으로 호출되는 정리 루틴(main()의 finally에서 호출).

        - 세션 통계(샘플 수, 수신율) 로그 출력
        - 열려 있던 CSV 파일 flush 없이 바로 close(콜백에서 이미 매번
          flush했으므로 close 시점엔 디스크 반영이 보장돼 있음)
        - PyQtGraph 사용 중이었다면 Qt 앱 종료
        각 단계를 개별 try/except로 감싸 한쪽이 실패해도 나머지 정리가
        계속 진행되게 한다.
        """
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
    """진입점. 각 단계 의미는 모듈 docstring의 "main()의 동작 순서" 참고."""
    # QApplication은 반드시 메인 스레드에서 먼저 생성해야 하므로 노드 생성보다 앞서 처리
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

            # rclpy 콜백은 백그라운드 스레드로, Qt 이벤트 루프는 메인 스레드로
            # 분리 — Qt는 자기 이벤트 루프가 메인 스레드에서 돌아야 하기 때문
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()

            # Qt 이벤트 루프 실행 — 창이 닫힐 때까지 여기서 블로킹
            app.exec()
        else:
            # GUI 없으면 별도 스레드 분리 없이 바로 스핀
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
