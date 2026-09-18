cmake_minimum_required(VERSION 3.8)
project(rumblex_brain)

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
find_package(std_srvs REQUIRED)
find_package(rumblex_interfaces REQUIRED)
find_package(rumblex_utils REQUIRED)
find_package(nlohmann_json REQUIRED) # nlohmann_json -> sudo apt install nlohmann-json3-dev
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(GTest REQUIRED)

## set sources
set(SOURCES
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
target_include_directories(rumblex_brain_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rumblex_brain_lib PUBLIC
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