cmake_minimum_required(VERSION 3.8)
project(quad_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(quad_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(position_controller src/position_controller.cpp)
target_include_directories(position_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(position_controller rclcpp nav_msgs geometry_msgs std_srvs quad_interfaces)

install(TARGETS position_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
