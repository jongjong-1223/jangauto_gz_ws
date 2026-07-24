#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
import csv

class SlipPlotter(Node):
    def __init__(self):
        super().__init__('slip_plotter')
        self.slip_sub = self.create_subscription(Vector3, '/slip_ratio', self.slip_callback, 1)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 1)

        self.current_time = 0.0
        self.slip_r_data = []
        self.slip_l_data = []
        self.time_data = []

    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def slip_callback(self, msg):
        self.time_data.append(self.current_time)
        self.slip_r_data.append(msg.x)
        self.slip_l_data.append(msg.y)

        if len(self.slip_r_data) > 50:
            self.slip_r_data.pop(0)
            self.slip_l_data.pop(0)
            self.time_data.pop(0)
        
        # if abs(self.current_time - 15.0) < 0.05 and not hasattr(self, 'csv_saved'):
        #     with open('/home/jungwoo/Downloads/slip_vs_time_slip.csv', 'w', newline='') as f:
        #         writer = csv.writer(f)
        #         writer.writerow(['time', 'slip_ratio_r', 'slip_ratio_l'])
        #         for t, r, l in zip(self.time_data, self.slip_r_data, self.slip_l_data):
        #             writer.writerow([t, r, l])
        #     self.csv_saved = True  # 중복 저장 방지

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
