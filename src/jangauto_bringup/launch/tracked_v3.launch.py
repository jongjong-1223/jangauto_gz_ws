# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_project_gazebo = get_package_share_directory('jangauto_gazebo')
    pkg_project_description = get_package_share_directory('jangauto_description')
    pkg_project_perception = get_package_share_directory('jangauto_perception')
    pkg_project_gps_driver = get_package_share_directory('jangauto_gps_driver')
    pkg_project_application = get_package_share_directory('jangauto_application')
    pkg_project_navigation2 = get_package_share_directory('jangauto_navigation2')
    pkg_project_hmi = get_package_share_directory('jangauto_hmi')
    pkg_project_mission = get_package_share_directory('jangauto_mission')

    # Visualize in RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(get_package_share_directory('jangauto_bringup'), 'config', 'tracked_v1.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, 'launch', 'gazebo.launch.py')),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_description, 'launch', 'description.launch.py')),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_perception, 'launch', 'localization.launch.py')),
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gps_driver, 'launch', 'gps.launch.py')),
    )

    application = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_application, 'launch', 'application.launch.py')),
    )

    # 워크스페이스에 사전 제작된 정적 map 파일이 없으므로 AMCL 대신 slam_toolbox를 사용
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_navigation2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    hmi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_hmi, 'launch', 'hmi.launch.py')),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_mission, 'launch', 'mission.launch.py')),
    )

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('jangauto_bringup'), 'launch', 'diagnostics.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        description,
        localization,
        gps,
        application,
        # navigation2,
        hmi,
        mission,
        diagnostics,
        # rviz,
    ])
