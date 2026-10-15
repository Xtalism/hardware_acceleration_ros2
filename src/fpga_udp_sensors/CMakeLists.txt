cmake_minimum_required(VERSION 3.8)
project(fpga_udp_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# ADXL345 (port 1024) -> /imu/data
add_executable(udp_imu_node src/udp_imu_node.cpp)
ament_target_dependencies(udp_imu_node rclcpp sensor_msgs geometry_msgs tf2_ros)

# LD19 lidar (port 1025) -> /scan
add_executable(udp_ld19_node src/udp_ld19_node.cpp)
ament_target_dependencies(udp_ld19_node rclcpp sensor_msgs geometry_msgs tf2_ros)

# wheel encoders (port 1026) -> /odom, /wheel_states, /wheel_markers
add_executable(udp_mecanum_odom_node src/udp_mecanum_odom_node.cpp)
ament_target_dependencies(udp_mecanum_odom_node rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros visualization_msgs)

install(TARGETS udp_imu_node udp_ld19_node udp_mecanum_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
