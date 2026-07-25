from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_state_machine_node = Node(
        package="jangauto_mission",
        executable="mission_state_machine.py",
        name="mission_state_machine",
        output="screen",
    )

    return LaunchDescription([
        mission_state_machine_node,
    ])
