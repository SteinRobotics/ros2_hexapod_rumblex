cmake_minimum_required(VERSION 3.16)
project(rumblex_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rumblex_interfaces REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Exportes for downstream packages
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

# Unit tests
if(BUILD_TESTING)
  set(TEST_BASE_DEPS rumblex_utils rclcpp::rclcpp ament_index_cpp::ament_index_cpp)
  set(TEST_MSG_DEPS
    geometry_msgs::geometry_msgs__rosidl_generator_cpp geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    rumblex_interfaces::rumblex_interfaces__rosidl_generator_cpp rumblex_interfaces::rumblex_interfaces__rosidl_typesupport_cpp)

  ament_add_gtest(test_callback_timer 
    test/test_callback_timer.cpp
  )
  if(TARGET test_callback_timer)
    target_include_directories(test_callback_timer PRIVATE include)
    target_link_libraries(test_callback_timer ${TEST_BASE_DEPS})
  endif()

  ament_add_gtest(test_simple_timer 
    test/test_simpletimer.cpp
  )
  if(TARGET test_simple_timer)
    target_include_directories(test_simple_timer PRIVATE include)
    target_link_libraries(test_simple_timer ${TEST_BASE_DEPS})
  endif()

  ament_add_gtest(test_pid_controller
    test/test_pid_controller.cpp
  )
  if(TARGET test_pid_controller)
    target_include_directories(test_pid_controller PRIVATE include)
    target_link_libraries(test_pid_controller ${TEST_BASE_DEPS})
  endif()

  ament_add_gtest(test_filters
    test/test_filters.cpp
  )
  if(TARGET test_filters)
    target_include_directories(test_filters PRIVATE include)
    target_link_libraries(test_filters ${TEST_BASE_DEPS} ${TEST_MSG_DEPS})
  endif()

  ament_add_gtest(test_geometry
    test/test_geometry.cpp
  )
  if(TARGET test_geometry)
    target_include_directories(test_geometry PRIVATE include)
    target_link_libraries(test_geometry ${TEST_BASE_DEPS} ${TEST_MSG_DEPS})
  endif()
endif()


ament_package()
