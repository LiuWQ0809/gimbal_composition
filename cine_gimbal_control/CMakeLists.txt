cmake_minimum_required(VERSION 3.8)
project(cine_gimbal_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(recomo_controller REQUIRED)
find_package(jc2804_gimbal_driver REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tracking_node src/gimbal_tracking_node.cpp)
target_include_directories(tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tracking_node PUBLIC c_std_17)

ament_target_dependencies(tracking_node
  rclcpp
  sensor_msgs
  std_msgs
  recomo_controller
  jc2804_gimbal_driver
)

install(TARGETS tracking_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
