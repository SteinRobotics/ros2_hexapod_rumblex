cmake_minimum_required(VERSION 3.8)
project(rumblex_movement)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

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
find_package(ament_cmake_gtest REQUIRED)
find_package(magic_enum CONFIG REQUIRED)


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
  src/handler/servohandler.cpp
  src/requester/gait_bodyroll.cpp
  src/requester/gait_clap.cpp
  src/requester/gait_continuouspose.cpp
  src/requester/gait_highfive.cpp
  src/requester/gait_laydown.cpp
  src/requester/gait_legwave.cpp
  src/requester/gait_look.cpp
  src/requester/gait_move_combined.cpp
  src/requester/gait_running.cpp
  src/requester/gait_standup.cpp
  src/requester/gait_singlepose.cpp
  src/requester/gait_testlegs.cpp
  src/requester/gait_waiting.cpp
  src/requester/gait_watch.cpp
  src/requester/gaitcontroller.cpp
  src/requester/kinematics.cpp
  src/requester/requester.cpp
)

add_library(rumblex_movement_core
  ${SOURCES}
)
target_include_directories(rumblex_movement_core PUBLIC
  include/rumblex_movement
  include/rumblex_movement/requester
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
  magic_enum::magic_enum
)

add_executable(node_movement
  src/node_movement.cpp
)
target_link_libraries(node_movement PRIVATE
  rumblex_movement_core
  rclcpp::rclcpp
  ${rumblex_utils_TARGETS}
)

include_directories(
  include/rumblex_movement/
  include/rumblex_movement/handler/feetech/
)

install(TARGETS
  node_movement
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Unit tests
if(BUILD_TESTING)
  ament_add_gtest(test_kinematics
    test/test_kinematics.cpp
  )
  if(TARGET test_kinematics)
    target_include_directories(test_kinematics PRIVATE include)
    target_link_libraries(test_kinematics rumblex_movement_core rclcpp::rclcpp ament_index_cpp::ament_index_cpp ${rumblex_interfaces_TARGETS})
  endif()

  ament_add_gtest(test_requester
    test/test_requester.cpp
  )
  if(TARGET test_requester)
    target_link_libraries(test_requester rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp)
  endif()

  ament_add_gtest(test_gait_bodyroll
    test/test_gait_bodyroll.cpp
  )
  if(TARGET test_gait_bodyroll)
    target_link_libraries(test_gait_bodyroll rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp)
  endif()

  ament_add_gtest(test_gait_laydown
    test/test_gait_laydown.cpp
  )
  if(TARGET test_gait_laydown)
    target_link_libraries(test_gait_laydown rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp ${geometry_msgs_TARGETS})
  endif()

  ament_add_gtest(test_gait_standup
    test/test_gait_standup.cpp
  )
  if(TARGET test_gait_standup)
    target_link_libraries(test_gait_standup rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp ${geometry_msgs_TARGETS})
  endif()

  ament_add_gtest(test_gait_highfive
    test/test_gait_highfive.cpp
  )
  if(TARGET test_gait_highfive)
    target_link_libraries(test_gait_highfive rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp ${geometry_msgs_TARGETS})
  endif()

  ament_add_gtest(test_gait_clap
    test/test_gait_clap.cpp
  )
  if(TARGET test_gait_clap)
    target_link_libraries(test_gait_clap rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp ${geometry_msgs_TARGETS})
  endif()

  ament_add_gtest(test_leg_servo_conversion
    test/test_leg_servo_conversion.cpp
  )
  if(TARGET test_leg_servo_conversion)
    target_include_directories(test_leg_servo_conversion PRIVATE include)
    target_link_libraries(test_leg_servo_conversion rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS})
  endif()

  ament_add_gtest(test_gait_legwave
    test/test_gait_legwave.cpp
  )
  if(TARGET test_gait_legwave)
    target_link_libraries(test_gait_legwave rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp ${geometry_msgs_TARGETS})
  endif()

  ament_add_gtest(test_gait_controller
    test/test_gait_controller.cpp
  )
  if(TARGET test_gait_controller)
    target_link_libraries(test_gait_controller rumblex_movement_core rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ament_index_cpp::ament_index_cpp ${geometry_msgs_TARGETS})
  endif()


endif()

ament_package()
