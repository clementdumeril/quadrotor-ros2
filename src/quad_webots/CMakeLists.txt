cmake_minimum_required(VERSION 3.8)
project(quad_webots)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quad_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

pluginlib_export_plugin_description_file(webots_ros2_driver mavic_driver.xml)

add_library(${PROJECT_NAME} SHARED src/mavic_driver.cpp src/mixer.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp pluginlib webots_ros2_driver nav_msgs sensor_msgs std_msgs geometry_msgs tf2_ros quad_interfaces)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch worlds resource config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mixer test/test_mixer.cpp src/mixer.cpp)
  target_include_directories(test_mixer PRIVATE include ${EIGEN3_INCLUDE_DIRS})
endif()

ament_package()
