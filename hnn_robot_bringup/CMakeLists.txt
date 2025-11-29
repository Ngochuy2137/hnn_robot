cmake_minimum_required(VERSION 3.8)
project(hnn_robot_bringup)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  worlds
  rviz
  map
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
