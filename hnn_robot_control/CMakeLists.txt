cmake_minimum_required(VERSION 3.16)
project(hnn_robot_control LANGUAGES CXX)

# Use ros2_control_cmake for ROS2 Jazzy
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Check ros2_control version (ROS2 Jazzy requires >= 4.0.0)
find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required for ROS2 Jazzy. "
    "Are you using the correct ROS2 distribution?")
endif()

find_package(ros2_control REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Build hardware interface library
add_library(
  hnn_robot_control
  SHARED
  src/hnn_robot_hardware.cpp
  src/hnn_diff_robot_hardware.cpp
)

target_include_directories(hnn_robot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hnn_robot_control PUBLIC cxx_std_17)

target_link_libraries(hnn_robot_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export plugin description file
pluginlib_export_plugin_description_file(hardware_interface hnn_diff_robot_hardware.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS hnn_robot_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

