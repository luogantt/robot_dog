cmake_minimum_required(VERSION 3.8)
project(s10_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch route
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/preflight_check.sh
  scripts/apply_upstream_patches.sh
  scripts/record_bag.sh
  scripts/start_mapping.sh
  scripts/start_navigation.sh
  scripts/stop_navigation.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
