cmake_minimum_required(VERSION 3.25)
project(rumblex_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds models config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/joint_state_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_house_furniture test/test_house_furniture.py)
endif()

ament_package()
