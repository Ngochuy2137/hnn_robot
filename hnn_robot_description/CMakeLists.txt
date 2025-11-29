cmake_minimum_required(VERSION 3.8)
project(hnn_robot_description)

find_package(ament_cmake REQUIRED)

# Install directories (only install if they exist)
foreach(dir xacro meshes)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
      USE_SOURCE_PERMISSIONS
    )
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

