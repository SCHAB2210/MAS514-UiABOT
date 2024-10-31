cmake_minimum_required(VERSION 3.8)
project(bno055)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python nodes directly
install(PROGRAMS
  src/bno_node.py
  src/tf_bno.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

