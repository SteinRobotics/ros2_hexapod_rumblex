cmake_minimum_required(VERSION 3.8)
project(rumblex_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/joint_state_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
