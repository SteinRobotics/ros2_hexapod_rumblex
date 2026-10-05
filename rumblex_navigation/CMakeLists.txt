cmake_minimum_required(VERSION 3.25)
project(rumblex_navigation)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
  node_navigation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/test_map_walls.py scripts/offline_odometry.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/wall_geometry.py scripts/planar_motion.py scripts/offline_motion.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation test/test_navigation.cpp)
  target_link_libraries(test_navigation
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    tf2::tf2
  )
  ament_add_pytest_test(test_wall_geometry test/test_wall_geometry.py)
  ament_add_pytest_test(test_planar_motion test/test_planar_motion.py)
  ament_add_pytest_test(test_offline_odometry test/test_offline_odometry.py)
endif()

ament_package()
