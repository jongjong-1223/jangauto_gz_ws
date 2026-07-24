import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_application = get_package_share_directory('jangauto_application')

    pure_pursuit_node = Node(
        package='jangauto_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slip_plot_node = Node(
        package='jangauto_application',
        executable='plot_slip_vs_sim_time.py',
        name='slip_plot_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # twist_mux_node for priority of cmd_vel
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg_project_application, 'config', 'twist_mux.yaml')],
        output='screen',
    )

    return LaunchDescription([
        # pure_pursuit_node,
        # slip_plot_node,
        # twist_mux_node,
    ])
