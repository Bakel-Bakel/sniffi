# sniffi_safety/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(sniffi_safety)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node rclcpp std_msgs geometry_msgs)

install(TARGETS safety_node DESTINATION lib/${PROJECT_NAME})

ament_package()
