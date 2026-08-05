"""로봇 URDF/SDF 기반 robot_state_publisher + 센서 프레임 static TF 발행(시뮬용).

description_real.launch.py와 static TF 값(물리적 장착 오프셋)은 동일 — 차이는
use_sim_time(Gazebo 클럭 사용 여부)뿐.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_description = get_package_share_directory('jangauto_description')

    # Load the SDF file from "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'tracked_v3', 'tracked_v3.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    static_base_prefix_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_prefix",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "tracked_v3/base_link"],
        output="screen",
    )
    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "base_link", "tracked_v3/base_link/imu_sensor"],
        output="screen",
    )
    static_gps_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_gps_to_base",
        arguments=["0.1", "0", "0", "0", "0", "0", "base_link", "tracked_v3/base_link/navsat_sensor"],
        output="screen",
    )
    # 뎁스카메라(front_depth_camera)의 PointCloud2 등이 이 프레임 이름으로
    # 오는데(gz-sim 센서 프레임 명명 규칙), imu/gps와 마찬가지로 이 TF가
    # 없으면 nav2 코스트맵의 메시지 필터가 계속 드롭한다(실행 확인됨).
    static_depth_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_depth_camera_to_base",
        arguments=["0.55", "0", "0.1", "0", "0", "0", "base_link",
                   "tracked_v3/base_link/front_depth_camera"],
        output="screen",
    )

    # base_link(차체 중심, 지면 0.35m 위) -> base_footprint(지면 투영, Z=0) — nav2 amcl/collision_monitor용
    static_base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_footprint",
        arguments=["0", "0", "-0.35", "0", "0", "0", "base_link", "base_footprint"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        static_base_prefix_tf,
        static_imu_tf,
        static_gps_tf,
        static_depth_camera_tf,
        static_base_footprint_tf,
    ])
