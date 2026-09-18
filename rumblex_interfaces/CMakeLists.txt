cmake_minimum_required(VERSION 3.8)
project(rumblex_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CommunicationStatus.msg"
  "msg/ContinuousMovementUpdate.msg"
  "msg/MovementRequest.msg"
  "msg/ServoDirectRequest.msg"
  "msg/ServoIndex.msg"
  "msg/ServoAngle.msg"
  "msg/ServoAngles.msg"
  "msg/ServoStatus.msg"
  "msg/JoystickRequest.msg"
  "msg/Orientation.msg"
  "msg/Pose.msg"
  
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
