cmake_minimum_required(VERSION 3.8)
project(raspi_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/forward_kinematic.py
  src/odom.py
  src/base_to_tf.py
  src/serial_interface.py
  src/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()

