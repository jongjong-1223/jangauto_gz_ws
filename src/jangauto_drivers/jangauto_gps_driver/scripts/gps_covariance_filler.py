#!/usr/bin/env python3
"""Republishes NavSatFix with a fixed position_covariance filled in.

The simulated navsat sensor (gz-sensors) does not populate
position_covariance regardless of any configured noise, so downstream
consumers (navsat_transform_node -> ekf_global) see it as 0.0 and treat
GPS as perfectly certain. This node copies the message through unchanged
except for the covariance fields.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

# GPS 데이터가 이 시간(초) 이상 안 오면 WARN(끊김)으로 판정
STALE_TIMEOUT_SEC = 3.0


class GpsCovarianceFiller(Node):

    def __init__(self):
        super().__init__('gps_covariance_filler')
        self.declare_parameter('position_stddev', 0.3)
        stddev = self.get_parameter('position_stddev').value
        variance = stddev * stddev
        self._covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        self._pub = self.create_publisher(NavSatFix, 'navsat_fixed', 10)
        self._sub = self.create_subscription(
            NavSatFix, 'navsat', self._callback, 10)

        self._last_msg_monotonic = None

        self._diag_updater = diagnostic_updater.Updater(self)
        self._diag_updater.setHardwareID('gps_covariance_filler')
        self._diag_updater.add('GPS reception', self._diagnostics_callback)

    def _callback(self, msg: NavSatFix) -> None:
        self._last_msg_monotonic = time.monotonic()
        msg.position_covariance = self._covariance
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub.publish(msg)

    def _diagnostics_callback(self, stat):
        if self._last_msg_monotonic is None:
            stat.summary(DiagnosticStatus.ERROR, 'No GPS data received yet')
        elif (time.monotonic() - self._last_msg_monotonic) > STALE_TIMEOUT_SEC:
            stat.summary(DiagnosticStatus.WARN, 'GPS data is stale')
        else:
            stat.summary(DiagnosticStatus.OK, 'Receiving GPS data')
        return stat


def main():
    rclpy.init()
    node = GpsCovarianceFiller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
