cmake_minimum_required(VERSION 3.25)
project(rumblex_camera)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/desktop_camera.py scripts/face_preview.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_desktop_camera test/test_desktop_camera.py)
  ament_add_pytest_test(test_camera_launch test/test_camera_launch.py)
endif()

ament_package()
