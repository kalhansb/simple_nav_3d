# Moved comments: doc/simple_nav_3d_code_notes.md
cmake_minimum_required(VERSION 3.8)
project(simple_nav_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common segmentation filters)
find_package(scovox_msgs REQUIRED)

add_library(simple_nav_3d_core
  src/parameters.cpp
  src/controller_factory.cpp
  src/planner_factory.cpp
  src/mapping/local_mapper.cpp
  src/controllers/ugv_controller.cpp
  src/controllers/uav_controller.cpp
  src/planners/ugv_planner.cpp
  src/planners/uav_planner.cpp
)

target_include_directories(simple_nav_3d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(simple_nav_3d_core ${PCL_LIBRARIES})

ament_target_dependencies(simple_nav_3d_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  scovox_msgs
)

add_executable(simple_nav_costmap_node src/simple_nav_costmap_node.cpp)
target_link_libraries(simple_nav_costmap_node simple_nav_3d_core)
ament_target_dependencies(simple_nav_costmap_node
  rclcpp
  nav_msgs
  sensor_msgs
)

add_executable(simple_nav_planner_node src/simple_nav_planner_node.cpp)
target_link_libraries(simple_nav_planner_node simple_nav_3d_core)
ament_target_dependencies(simple_nav_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  scovox_msgs
)

add_executable(simple_nav_controller_node src/simple_nav_controller_node.cpp)
target_link_libraries(simple_nav_controller_node simple_nav_3d_core)
ament_target_dependencies(simple_nav_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
)

add_executable(simple_nav_navigator_node src/simple_nav_navigator_node.cpp)
target_link_libraries(simple_nav_navigator_node simple_nav_3d_core)
ament_target_dependencies(simple_nav_navigator_node
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  simple_nav_3d_core
  simple_nav_costmap_node
  simple_nav_planner_node
  simple_nav_controller_node
  simple_nav_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # No if(TARGET ...) guard, on purpose (as in explo_planner): without gtest the
  # guard yields a green build with zero tests; unguarded, it is a hard CMake
  # error. (notes: cmake-gtest-no-target-guard)
  ament_add_gtest(test_ugv_planner test/test_ugv_planner.cpp)
  target_link_libraries(test_ugv_planner simple_nav_3d_core)
  # D2. segment_free is header-only on purpose: its caller is the planner node,
  # an executable that cannot be linked into a test, so the guard was lifted out
  # to where it can be pinned. The link is still needed for the ROS message
  # types the fixtures build.
  ament_add_gtest(test_goal_snap test/test_goal_snap.cpp)
  target_link_libraries(test_goal_snap simple_nav_3d_core)
  # D2b. same_goal_pose lives in grid_utils.hpp for the same reason: its caller
  # is the navigator node, which is an executable and cannot be linked into a
  # test. The predicate decides whether an upstream goal revision reaches the
  # controller at all, so it is pinned where it can be.
  ament_add_gtest(test_goal_identity test/test_goal_identity.cpp)
  target_link_libraries(test_goal_identity simple_nav_3d_core)
  ament_target_dependencies(test_ugv_planner
    rclcpp
    geometry_msgs
    nav_msgs
  )
endif()

ament_export_include_directories(include)
ament_package()
