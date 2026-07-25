import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    analyzers_yaml = os.path.join(
        get_package_share_directory('jangauto_bringup'), 'config', 'diagnostic_analyzers.yaml')

    aggregator_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[analyzers_yaml],
    )

    return LaunchDescription([
        aggregator_node,
    ])
