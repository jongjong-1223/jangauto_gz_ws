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
        static_base_footprint_tf,
    ])
