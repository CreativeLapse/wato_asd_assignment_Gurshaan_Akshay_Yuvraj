cmake_minimum_required(VERSION 3.10)
project(tf_throttle)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)     # ROS2 build tool
find_package(rclcpp REQUIRED)          # ROS2 C++ package
find_package(tf2_msgs REQUIRED)        # TFMessage
find_package(geometry_msgs REQUIRED)   # TransformStamped

# Create ROS2 node executable from source files
add_executable(tf_throttle_node src/tf_throttle_node.cpp)
ament_target_dependencies(tf_throttle_node rclcpp tf2_msgs geometry_msgs)

# Copy executable to installation location
install(TARGETS
  tf_throttle_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
