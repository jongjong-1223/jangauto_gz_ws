# nav2_regulated_pure_pursuit_controller(nav2 1.3.12)를 워크스페이스로
# vendoring한 nav2_core::Controller 플러그인 빌드 설정.
# - apt 언더레이의 동명 패키지와 이름이 다르므로(오버레이 섀도잉 아님) 별도
#   패키지로 완전히 분리 빌드된다.
# - nav2_package() 매크로(nav2_common 제공)는 원본 RPP가 쓰던 그대로 유지 —
#   nav2 표준 컴파일 옵션/경고 설정을 그대로 물려받기 위함.
cmake_minimum_required(VERSION 3.5)
project(nav2_stanley_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  angles
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
)

set(library_name nav2_stanley_controller)

add_library(${library_name} SHARED
        src/stanley_controller.cpp
        src/collision_checker.cpp
        src/parameter_handler.cpp
        src/path_handler.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_stanley_controller.xml)

ament_package()
