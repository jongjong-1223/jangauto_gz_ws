cmake_minimum_required(VERSION 3.5)

project(jangauto_bringup)

find_package(ament_cmake REQUIRED)
find_package(jangauto_description REQUIRED)
find_package(jangauto_gazebo REQUIRED)
find_package(jangauto_gps_driver REQUIRED)
find_package(jangauto_uwb_driver REQUIRED)
find_package(jangauto_motor_driver REQUIRED)
find_package(jangauto_imu_driver REQUIRED)
find_package(jangauto_perception REQUIRED)
find_package(jangauto_navigation2 REQUIRED)
find_package(jangauto_hmi REQUIRED)
find_package(jangauto_mission REQUIRED)
find_package(jangauto_control REQUIRED)
find_package(diagnostic_aggregator REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
