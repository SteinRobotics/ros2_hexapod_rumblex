cmake_minimum_required(VERSION 3.25)
project(rumblex_brain)

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
find_package(std_srvs REQUIRED)
find_package(rumblex_interfaces REQUIRED)
find_package(rumblex_utils REQUIRED)
find_package(nlohmann_json REQUIRED) # nlohmann_json -> sudo apt install nlohmann-json3-dev

## set sources
set(SOURCES
src/gait/gait_clap.cpp
src/gait/gait_continuous_pose.cpp
src/gait/gait_controller.cpp
src/gait/gait_high_five.cpp
src/gait/gait_leg_wave.cpp
src/gait/gait_move_combined.cpp
src/gait/gait_running.cpp
src/gait/stride_planner.cpp
src/gait/gait_single_pose.cpp
src/gait/gait_test_legs.cpp
src/gait/gait_torso_roll.cpp
src/gait/gait_waiting.cpp
src/gait/gait_yaw_sequence.cpp
src/requester/coordinator.cpp
src/action/action_planner.cpp
src/requester/recognition.cpp
src/requester/error_management.cpp
src/requester/text_interpreter.cpp
src/handler/system.cpp
src/handler/communication.cpp
src/handler/movement.cpp
src/parser/behavior_parser.cpp
)
add_library(rumblex_brain_lib ${SOURCES})
target_compile_features(rumblex_brain_lib PUBLIC cxx_std_23)
target_include_directories(rumblex_brain_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rumblex_brain_lib PUBLIC
  rumblex_utils::rumblex_geometry
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${rumblex_interfaces_TARGETS}
  ${rumblex_utils_TARGETS}
  nlohmann_json::nlohmann_json
)

add_executable(node_brain
  src/node_brain.cpp
)
target_link_libraries(node_brain PRIVATE
  rumblex_brain_lib
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ament_index_cpp::ament_index_cpp
  ${rumblex_interfaces_TARGETS}
  nlohmann_json::nlohmann_json
)
target_include_directories(node_brain PRIVATE include)

# Test executable for behavior parser
add_executable(test_behavior_parser
  test/test_behavior_parser.cpp
)
target_link_libraries(test_behavior_parser PRIVATE
  rumblex_brain_lib
  rclcpp::rclcpp
  ${rumblex_interfaces_TARGETS}
  nlohmann_json::nlohmann_json
)
target_include_directories(test_behavior_parser PRIVATE include)

# Test executable for coordinator with behaviors
add_executable(test_coordinator_behaviors
  test/test_coordinator_behaviors.cpp
)
target_link_libraries(test_coordinator_behaviors PRIVATE
  rumblex_brain_lib
  rclcpp::rclcpp
  ${rumblex_interfaces_TARGETS}
  nlohmann_json::nlohmann_json
)
target_include_directories(test_coordinator_behaviors PRIVATE include)

include_directories(
  include/rumblex_brain/
)

install(TARGETS
  node_brain
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_movement_types
    test/test_movement_types.cpp
    test/test_movement_types_other_translation_unit.cpp
  )
  target_link_libraries(test_movement_types rumblex_brain_lib)
  ament_add_gtest(test_coordinator test/test_coordinator.cpp)
  target_link_libraries(test_coordinator rumblex_brain_lib)
  ament_add_gtest(test_trajectory_transitions test/test_trajectory_transitions.cpp)
  target_link_libraries(test_trajectory_transitions rumblex_brain_lib)
  ament_add_gtest(test_locomotion_transitions test/test_locomotion_transitions.cpp)
  target_link_libraries(test_locomotion_transitions rumblex_brain_lib)
  ament_add_gtest(test_robot_configuration test/test_robot_configuration.cpp)
  target_link_libraries(test_robot_configuration rumblex_brain_lib)
  target_compile_definitions(test_robot_configuration PRIVATE RUMBLEX_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
  ament_add_gtest(test_movement_handler test/test_movement_handler.cpp)
  target_link_libraries(test_movement_handler rumblex_brain_lib)
  ament_add_gtest(test_body_pose test/test_body_pose.cpp)
  target_link_libraries(test_body_pose rumblex_brain_lib)
  ament_add_gtest(test_gait_clap test/test_gait_clap.cpp)
  target_link_libraries(test_gait_clap rumblex_brain_lib)
  ament_add_gtest(test_gait_yaw_sequence test/test_gait_yaw_sequence.cpp)
  target_link_libraries(test_gait_yaw_sequence rumblex_brain_lib)
  ament_add_gtest(test_gait_controller test/test_gait_controller.cpp)
  target_link_libraries(test_gait_controller rumblex_brain_lib)
  ament_add_gtest(test_gait_high_five test/test_gait_high_five.cpp)
  target_link_libraries(test_gait_high_five rumblex_brain_lib)
  ament_add_gtest(test_gait_lay_down test/test_gait_lay_down.cpp)
  target_link_libraries(test_gait_lay_down rumblex_brain_lib)
  ament_add_gtest(test_gait_leg_wave test/test_gait_leg_wave.cpp)
  target_link_libraries(test_gait_leg_wave rumblex_brain_lib)
  ament_add_gtest(test_gait_stand_up test/test_gait_stand_up.cpp)
  target_link_libraries(test_gait_stand_up rumblex_brain_lib)
  ament_add_gtest(test_gait_torso_roll test/test_gait_torso_roll.cpp)
  target_link_libraries(test_gait_torso_roll rumblex_brain_lib)
  ament_add_gtest(test_text_interpreter 
    test/test_text_interpreter.cpp
  )
  if(TARGET test_text_interpreter)
    target_include_directories(test_text_interpreter PRIVATE include)
    target_link_libraries(test_text_interpreter rumblex_brain_lib rclcpp::rclcpp ament_index_cpp::ament_index_cpp ${rumblex_interfaces_TARGETS} nlohmann_json::nlohmann_json)
  endif()

  ament_add_gtest(test_error_management
    test/test_error_management.cpp
  )
  if(TARGET test_error_management)
    target_include_directories(test_error_management PRIVATE include)
    target_link_libraries(test_error_management rumblex_brain_lib rclcpp::rclcpp ${rumblex_interfaces_TARGETS} ${rumblex_utils_TARGETS})
  endif()


endif()

ament_package()
