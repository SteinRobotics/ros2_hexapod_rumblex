cmake_minimum_required(VERSION 3.8)
project(rumblex_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf launch rviz meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nira_urdf test/test_nira_urdf.py)
  ament_add_pytest_test(test_nox_urdf test/test_nox_urdf.py)
endif()

ament_package()
