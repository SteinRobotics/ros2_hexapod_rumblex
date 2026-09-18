cmake_minimum_required(VERSION 3.8)
project(rumblex_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rumblex_interfaces REQUIRED)

## set sources
set(SOURCES
    src/LIDARLite_RaspberryPi_Library/src/lidarlite_v3.cpp
)

add_executable(node_lidar
  src/node_lidar.cpp
  ${SOURCES}
)
target_link_libraries(node_lidar
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_generator_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  rumblex_interfaces::rumblex_interfaces__rosidl_generator_cpp
  rumblex_interfaces::rumblex_interfaces__rosidl_typesupport_cpp
)

target_include_directories(node_lidar PRIVATE
  include/rumblex_lidar/
  src/LIDARLite_RaspberryPi_Library/include/
)

install(TARGETS
  node_lidar
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
