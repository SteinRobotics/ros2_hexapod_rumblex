cmake_minimum_required(VERSION 3.8)
project(rumblex_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
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

# --- Navigation node ---
add_executable(node_navigation src/node_navigation.cpp)
target_link_libraries(node_navigation
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
)

install(TARGETS
  node_head_scan
  node_navigation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
