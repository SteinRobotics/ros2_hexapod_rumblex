cmake_minimum_required(VERSION 3.25)
project(rumblex_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rumblex_interfaces REQUIRED)

# --- Head sweep scan node ---
add_executable(node_head_scan src/node_head_scan.cpp)
target_link_libraries(node_head_scan
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rumblex_interfaces::rumblex_interfaces__rosidl_generator_cpp
  rumblex_interfaces::rumblex_interfaces__rosidl_typesupport_cpp
)

install(TARGETS node_head_scan DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
