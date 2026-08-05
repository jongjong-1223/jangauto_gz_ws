#!/usr/bin/env python3
"""가상 UWB 맵 발행 노드.

## 역할
- 실제 UWB 앵커/태그 하드웨어와 위치 추정 알고리즘이 아직 없는 상태에서,
  nav2(`global_costmap`의 `static_layer`)가 참조할 `nav_msgs/OccupancyGrid`
  맵을 임시로 발행한다.
- 지금은 x:-10~10m, y:-10~10m(20m x 20m) 범위 안쪽은 전부 free(0), 가장
  바깥 1셀 테두리만 occupied(100)로 채운 placeholder 맵을 10Hz로 내보낸다
  (`mark_border_occupied` 파라미터로 끌 수 있음) — 실제 UWB 입력이 생기면
  이 노드 내부의 "맵 생성" 부분만 실제 변환 로직으로 교체하면 되도록, "가상/실
  UWB 입력 → OccupancyGrid 변환"이라는 노드 경계를 지금부터 고정해둔다.
  테두리는 두 가지 역할을 겸한다: (1) Nav2가 매핑 안 된 영역 밖으로 못
  나가게 막고, (2) 실 장애물 데이터 없이도 `app_websocket_bridge.py`의
  `/map` → 컨투어 추출 → 앱 전송 파이프라인을 눈으로 검증할 수 있게 함.
- 발행 토픽 이름은 nav2 기본 맵 토픽(`/map`)에 맞춰, `map_server` 없이도
  `static_layer`가 그대로 구독할 수 있게 한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

# 가상 맵의 물리적 범위(m)와 해상도 — x/y 각각 -10~10m, 셀 크기 0.1m -> 200x200 그리드.
MAP_HALF_EXTENT_M = 10.0
MAP_RESOLUTION_M = 0.1
MAP_FRAME_ID = "map"
PUBLISH_RATE_HZ = 10.0
OCCUPIED_VALUE = 100
FREE_VALUE = 0


class UwbVirtualMapPublisher(Node):
    """가상 UWB 데이터를 OccupancyGrid로 변환해 `/map`에 발행하는 노드."""

    def __init__(self):
        super().__init__('uwb_virtual_map_publisher')

        self.declare_parameter('mark_border_occupied', True)
        self._mark_border_occupied = self.get_parameter(
            'mark_border_occupied').get_parameter_value().bool_value

        # static_layer가 map_server 없이도 늦게 구독해서 즉시 최신 맵을 받을 수 있도록
        # latched(TRANSIENT_LOCAL) QoS 사용 — map_server의 기본 발행 방식과 동일한 계약.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self._grid_msg = self._build_placeholder_grid()

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_map)

    def _build_placeholder_grid(self) -> OccupancyGrid:
        """전부 free(0)로 채운 placeholder OccupancyGrid를 한 번만 생성해 재사용.

        - origin을 (-MAP_HALF_EXTENT_M, -MAP_HALF_EXTENT_M)에 둬서, 그리드가
          x/y 각각 -10~10m 범위를 정확히 덮게 한다(OccupancyGrid의 origin은
          셀 (0,0)의 world pose를 의미).
        - `mark_border_occupied`가 켜져 있으면 가장 바깥 1셀 링만 occupied로
          채운다 — 안쪽은 여전히 free라 Nav2 내비게이션엔 지장 없이, 지도
          경계를 컨투어로 추출 가능한 도형 하나로 만든다.
        """
        side_cells = int((2 * MAP_HALF_EXTENT_M) / MAP_RESOLUTION_M)

        grid = OccupancyGrid()
        grid.header.frame_id = MAP_FRAME_ID
        grid.info.resolution = MAP_RESOLUTION_M
        grid.info.width = side_cells
        grid.info.height = side_cells
        grid.info.origin.position.x = -MAP_HALF_EXTENT_M
        grid.info.origin.position.y = -MAP_HALF_EXTENT_M
        grid.info.origin.orientation.w = 1.0
        grid.data = [FREE_VALUE] * (side_cells * side_cells)

        if self._mark_border_occupied:
            last = side_cells - 1
            for row in range(side_cells):
                for col in range(side_cells):
                    if row == 0 or row == last or col == 0 or col == last:
                        grid.data[row * side_cells + col] = OCCUPIED_VALUE

        return grid

    def _publish_map(self) -> None:
        """타이머 콜백 — 매번 새 타임스탬프를 찍어 같은 grid 내용을 재발행."""
        self._grid_msg.header.stamp = self.get_clock().now().to_msg()
        self._grid_msg.info.map_load_time = self._grid_msg.header.stamp
        self._map_pub.publish(self._grid_msg)


def main():
    """노드 진입점 — `rclpy.spin()`으로 상주하며 타이머 콜백을 계속 처리한다."""
    rclpy.init()
    node = UwbVirtualMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
