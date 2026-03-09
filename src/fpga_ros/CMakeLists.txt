cmake_minimum_required(VERSION 3.8)
project(fpga_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PROGRAMS
serial_com
adxl345_fpga_pub
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
)

add_library(adxl345_fpga_lib lib/adxl345_fpga.c)
target_include_directories(adxl345_fpga_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

foreach(PROGRAM ${PROGRAMS})
    add_executable(${PROGRAM} src/${PROGRAM}.cpp)

    target_include_directories(${PROGRAM} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROGRAM} adxl345_fpga_lib)
    ament_target_dependencies(${PROGRAM} ${DEPENDENCIES})
    install(TARGETS ${PROGRAM} DESTINATION lib/${PROJECT_NAME})
endforeach()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
