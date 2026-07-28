cmake_minimum_required(VERSION 3.5)
project(jangauto_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cmd_vel_arbiter.py
  scripts/key_manual_driver.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
