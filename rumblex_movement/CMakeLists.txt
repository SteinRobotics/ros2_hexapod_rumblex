cmake_minimum_required(VERSION 3.25)
project(rumblex_movement)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rumblex_interfaces REQUIRED)
find_package(rumblex_utils REQUIRED)
find_package(yaml-cpp REQUIRED)
## set sources
set(SOURCES
  src/handler/servo_controller.cpp
  src/handler/hiwonder_protocol.cpp
  src/handler/feetech_protocol.cpp
  src/handler/feetech/HLSCL.cpp
  src/handler/feetech/SCS.cpp
  src/handler/feetech/SCSCL.cpp
  src/handler/feetech/SCSerial.cpp
  src/handler/feetech/SMS_STS.cpp
  src/handler/servo_handler.cpp
  src/requester/requester.cpp
)

add_library(rumblex_movement_core
  ${SOURCES}
)
target_compile_features(rumblex_movement_core PUBLIC cxx_std_23)
target_include_directories(rumblex_movement_core PUBLIC
  include/rumblex_movement
  include/rumblex_movement/handler/feetech
)
target_link_libraries(rumblex_movement_core PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  ${rumblex_interfaces_TARGETS}
  ${rumblex_utils_TARGETS}
  yaml-cpp::yaml-cpp
  rumblex_utils::rumblex_geometry
)

add_executable(node_movement
  src/node_movement.cpp
)
target_link_libraries(node_movement PRIVATE
  rumblex_movement_core
  rclcpp::rclcpp
  ${rumblex_utils_TARGETS}
)

add_executable(node_offline_visualization src/node_offline_visualization.cpp)
target_link_libraries(node_offline_visualization PRIVATE rumblex_movement_core)

install(TARGETS
  node_movement
  node_offline_visualization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scserial test/test_scserial.cpp)
  if(TARGET test_scserial)
    target_link_libraries(test_scserial rumblex_movement_core util)
  endif()

  ament_add_gtest(test_offline_visualization test/test_offline_visualization.cpp)
  if(TARGET test_offline_visualization)
    target_link_libraries(test_offline_visualization rumblex_movement_core)
    target_compile_definitions(test_offline_visualization PRIVATE
      RUMBLEX_ANATOMY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../rumblex_description/config")
  endif()

  ament_add_gtest(test_robot_configuration test/test_robot_configuration.cpp)
  if(TARGET test_robot_configuration)
    target_link_libraries(test_robot_configuration rumblex_movement_core)
    target_compile_definitions(test_robot_configuration PRIVATE
      RUMBLEX_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
      RUMBLEX_ANATOMY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../rumblex_description/config")
  endif()

  ament_add_gtest(test_types test/test_types.cpp)
  if(TARGET test_types)
    target_link_libraries(test_types rumblex_movement_core)
  endif()

  ament_add_gtest(test_units test/test_units.cpp)
  if(TARGET test_units)
    target_link_libraries(test_units rumblex_utils::rumblex_geometry)
  endif()

  ament_add_gtest(test_kinematics
    test/test_kinematics.cpp
  )
  if(TARGET test_kinematics)
    target_link_libraries(test_kinematics rumblex_movement_core rclcpp::rclcpp ament_index_cpp::ament_index_cpp ${rumblex_interfaces_TARGETS})
  endif()

  ament_add_gtest(test_requester
    test/test_requester.cpp
  )
  if(TARGET test_requester)
    target_link_libraries(test_requester rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp)
  endif()

  ament_add_gtest(test_leg_servo_conversion
    test/test_leg_servo_conversion.cpp
  )
  if(TARGET test_leg_servo_conversion)
    target_link_libraries(test_leg_servo_conversion rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS})
  endif()

endif()

ament_package()
