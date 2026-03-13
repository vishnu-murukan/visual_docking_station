cmake_minimum_required(VERSION 3.8)
project(visual_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Install Python package (provides 'import visual_docking')
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/aruco_detector_node.py
  scripts/dock_controller.py
  scripts/generate_aruco_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directory resources
install(DIRECTORY launch   DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config   DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds   DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models   DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf     DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
