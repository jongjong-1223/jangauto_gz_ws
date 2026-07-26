cmake_minimum_required(VERSION 3.5)
project(jangauto_uwb_driver)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/uwb_virtual_map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
