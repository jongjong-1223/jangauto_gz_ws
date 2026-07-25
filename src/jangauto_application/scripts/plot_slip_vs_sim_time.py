#!/usr/bin/env python3
"""좌우 바퀴 슬립비(slip ratio)를 시뮬레이션 시간 축으로 실시간 플로팅하는 노드.

## 역할
- `/slip_ratio`(Vector3: x=우측, y=좌측)를 구독해 최근 50개 샘플만
  유지하며 matplotlib 창에 실시간으로 그린다(오래된 샘플은 버림 —
  긴 시뮬레이션에서도 그래프가 무한히 무거워지지 않게 하기 위함).
- `/clock`(시뮬레이션 시계)을 별도 구독해 x축을 실제 시뮬레이션 시간으로
  맞춘다 — 콜백이 호출되는 wall-clock 시간이 아니라 시뮬레이터가 보고하는
  시간을 써야 시뮬레이션 배속/일시정지와 무관하게 그래프가 정확하다.

## 클래스 구성
- `SlipPlotter`: 슬립비/시계 구독과 실시간 플로팅을 모두 담당하는
  단일 클래스. 별도 발행이나 저장 로직은 없다(CSV 저장 코드는
  주석 처리된 디버그용 잔재).

## main()의 동작 순서
1. rclpy 초기화
2. matplotlib 인터랙티브 모드(`plt.ion()`) 켜기 — 콜백 안에서
   `plt.pause()`로 화면을 갱신하기 위해 필요
3. `SlipPlotter` 노드 생성 → 구독 시작
4. `rclpy.spin()`으로 블로킹 대기, Ctrl+C 시 정상 종료
5. 인터랙티브 모드 끄고 마지막 그래프를 `plt.show()`로 고정 표시
6. rclpy 종료
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
import csv

class SlipPlotter(Node):
    def __init__(self):
        super().__init__('slip_plotter')
        # 두 토픽 모두 depth=1: 최신 값만 중요하고 과거 값 큐잉은 불필요.
        self.slip_sub = self.create_subscription(Vector3, '/slip_ratio', self.slip_callback, 1)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 1)

        self.current_time = 0.0
        self.slip_r_data = []
        self.slip_l_data = []
        self.time_data = []

    def clock_callback(self, msg):
        """`/clock` 콜백 — 시뮬레이션 시간을 초 단위 float로 갱신만 한다.
        슬립비 콜백이 이 값을 읽어 x축 타임스탬프로 쓴다.
        """
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def slip_callback(self, msg):
        """`/slip_ratio` 콜백 — 데이터 적재와 그래프 갱신을 함께 수행.

        - msg.x = 우측 바퀴 슬립비, msg.y = 좌측 바퀴 슬립비.
        - 리스트 길이가 50을 넘으면 가장 오래된 샘플을 버려 슬라이딩
          윈도우처럼 최근 구간만 화면에 유지한다.
        """
        self.time_data.append(self.current_time)
        self.slip_r_data.append(msg.x)
        self.slip_l_data.append(msg.y)

        if len(self.slip_r_data) > 50:
            self.slip_r_data.pop(0)
            self.slip_l_data.pop(0)
            self.time_data.pop(0)

        # 특정 시점(t=15s) 데이터를 CSV로 한 번 저장하기 위한 디버그용 코드.
        # 현재는 비활성화 상태로 남겨둠(로직 변경 없이 유지).
        # if abs(self.current_time - 15.0) < 0.05 and not hasattr(self, 'csv_saved'):
        #     with open('/home/jungwoo/Downloads/slip_vs_time_slip.csv', 'w', newline='') as f:
        #         writer = csv.writer(f)
        #         writer.writerow(['time', 'slip_ratio_r', 'slip_ratio_l'])
        #         for t, r, l in zip(self.time_data, self.slip_r_data, self.slip_l_data):
        #             writer.writerow([t, r, l])
        #     self.csv_saved = True  # 중복 저장 방지

        # 매 콜백마다 전체를 다시 그림(clf 후 재플롯) — 데이터가 50개로
        # 작게 제한되어 있어 성능 부담이 크지 않다.
        plt.clf()
        plt.plot(self.time_data, self.slip_r_data, label='slip_ratio_r', linewidth=3)
        plt.plot(self.time_data, self.slip_l_data, label='slip_ratio_l', linewidth=3, linestyle='--')
        plt.xlabel("Simulation Time [s]", fontsize=20)
        plt.ylabel("Slip Ratio", fontsize=20)
        plt.ylim(-2, 2)
        plt.grid(True)
        plt.legend(fontsize=15)
        plt.tick_params(axis='both', labelsize=15)
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    plt.ion()
    node = SlipPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    plt.ioff()
    plt.show()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
