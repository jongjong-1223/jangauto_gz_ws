"""`app_websocket_bridge` 노드 하나를 실행하는 launch 파일.

- 아래 launch argument는 전부 `app_websocket_bridge.py`의 ROS 파라미터로
  그대로 전달된다(호스트/포트, 하트비트 주기, mDNS 광고 여부/이름, 앱용
  상태 JSON 조립·브로드캐스트 주기).
- 기본값은 `app_websocket_bridge.py` 자체의 `declare_parameter` 기본값과
  같아야 한다 — 여기서 바꾸면 실제 동작도 그 값으로 바뀐다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument('host', default_value='0.0.0.0',
                                      description='WebSocket server bind address.')
    port_arg = DeclareLaunchArgument('port', default_value='8887',
                                      description='WebSocket server port (matches the app Config.WS_URL port).')
    heartbeat_period_arg = DeclareLaunchArgument('heartbeat_period_sec', default_value='0.5',
                                                  description='Heartbeat/link-liveness check period, in seconds.')
    heartbeat_timeout_arg = DeclareLaunchArgument('heartbeat_timeout_sec', default_value='1.5',
                                                   description='Time since last app message before link_alive goes false.')
    mdns_enabled_arg = DeclareLaunchArgument('mdns_enabled', default_value='true',
                                              description='Advertise a _robot._tcp.local. mDNS service for app NSD discovery.')
    mdns_instance_name_arg = DeclareLaunchArgument('mdns_instance_name', default_value='jangauto',
                                                    description='mDNS service instance name.')
    app_status_publish_period_arg = DeclareLaunchArgument(
        'app_status_publish_period_sec', default_value='0.2',
        description='Period for assembling /robot_status+/odometry/global+/odom into app JSON and broadcasting over WS.')
    map_data_retry_timeout_arg = DeclareLaunchArgument(
        'map_data_retry_timeout_sec', default_value='1.0',
        description='How long to wait for app_ack before resending map_data (matches the app\'s own RETRY_TIMEOUT_MS).')
    map_data_max_retries_arg = DeclareLaunchArgument(
        'map_data_max_retries', default_value='3',
        description='Max resend attempts for map_data before giving up (matches the app\'s own MAX_RETRIES).')
    map_data_retry_check_period_arg = DeclareLaunchArgument(
        'map_data_retry_check_period_sec', default_value='0.2',
        description='Polling period for checking whether a pending map_data delivery has timed out.')
    map_data_keepalive_period_arg = DeclareLaunchArgument(
        'map_data_keepalive_period_sec', default_value='5.0',
        description='Low-frequency period for re-broadcasting the last map_data even when unchanged, so a '
                    'client that missed the one-off change-triggered send (or connects later) still gets it.')
    map_occupied_threshold_arg = DeclareLaunchArgument(
        'map_occupied_threshold', default_value='99',
        description='Minimum OccupancyGrid cell value treated as occupied when extracting map_data vertices '
                    '(from /global_costmap/costmap: 99=lethal+inscribed-radius cells, excludes the softer '
                    'inflation gradient below that).')

    app_websocket_bridge_node = Node(
        package='jangauto_hmi',
        executable='app_websocket_bridge.py',
        name='app_websocket_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'heartbeat_period_sec': LaunchConfiguration('heartbeat_period_sec'),
            'heartbeat_timeout_sec': LaunchConfiguration('heartbeat_timeout_sec'),
            'mdns_enabled': LaunchConfiguration('mdns_enabled'),
            'mdns_instance_name': LaunchConfiguration('mdns_instance_name'),
            'app_status_publish_period_sec': LaunchConfiguration('app_status_publish_period_sec'),
            'map_data_retry_timeout_sec': LaunchConfiguration('map_data_retry_timeout_sec'),
            'map_data_max_retries': LaunchConfiguration('map_data_max_retries'),
            'map_data_retry_check_period_sec': LaunchConfiguration('map_data_retry_check_period_sec'),
            'map_data_keepalive_period_sec': LaunchConfiguration('map_data_keepalive_period_sec'),
            'map_occupied_threshold': LaunchConfiguration('map_occupied_threshold'),
        }],
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        heartbeat_period_arg,
        heartbeat_timeout_arg,
        mdns_enabled_arg,
        mdns_instance_name_arg,
        app_status_publish_period_arg,
        map_data_retry_timeout_arg,
        map_data_max_retries_arg,
        map_data_retry_check_period_arg,
        map_data_keepalive_period_arg,
        map_occupied_threshold_arg,
        app_websocket_bridge_node,
    ])
