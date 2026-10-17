cmake_minimum_required(VERSION 3.8)
project(maze_navigation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_flake8()
  ament_xmllint()
endif()

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/initialize_localization.py
  scripts/verify_navigation.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
