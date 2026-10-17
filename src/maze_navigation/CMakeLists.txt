cmake_minimum_required(VERSION 3.8)
project(maze_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(maze_navfn_planner SHARED src/maze_navfn_planner.cpp)
target_compile_features(maze_navfn_planner PUBLIC cxx_std_17)
target_include_directories(maze_navfn_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(maze_navfn_planner
  nav2_navfn_planner nav2_core nav2_costmap_2d pluginlib rclcpp nav_msgs geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)
install(TARGETS maze_navfn_planner LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_libraries(maze_navfn_planner)
ament_export_include_directories(include)
ament_export_dependencies(nav2_navfn_planner nav2_core nav2_costmap_2d
  rclcpp nav_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_long_route test/test_long_route.cpp)
  target_link_libraries(test_long_route maze_navfn_planner)
  target_compile_definitions(test_long_route PRIVATE
    COSTMAP_FIXTURE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/winding_costmap.pgm")
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
