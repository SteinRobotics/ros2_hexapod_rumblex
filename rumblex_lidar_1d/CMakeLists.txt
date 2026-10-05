cmake_minimum_required(VERSION 3.25)
project(rumblex_lidar_1d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  # The vendor driver ignores an I2C write result; keep its warning non-fatal.
  set_source_files_properties(src/LIDARLite_RaspberryPi_Library/src/lidarlite_v3.cpp
    PROPERTIES COMPILE_OPTIONS -Wno-error=unused-result)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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
)

target_include_directories(node_lidar PRIVATE
  include/rumblex_lidar_1d/
  src/LIDARLite_RaspberryPi_Library/include/
)

install(TARGETS
  node_lidar
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch config
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
