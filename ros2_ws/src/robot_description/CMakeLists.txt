cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the launch and description folders
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY description DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/raspi_sub/src/serial_interface.py
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/raspi_sub/src/odom.py
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/raspi_sub/src/base_to_tf.py
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/raspi_sub/src/joint_state_publisher.py
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/sllidar_ros2/src/sllidar_node.cpp
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/bno055/src/bno_node.py
    /home/uiabot/MAS514-UiABOT/ros2_ws/src/bno055/src/tf_bno.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

