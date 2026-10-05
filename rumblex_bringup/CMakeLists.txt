cmake_minimum_required(VERSION 3.25)
project(rumblex_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
launch config
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sensor_launch test/test_sensor_launch.py)
endif()

ament_package()
