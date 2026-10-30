cmake_minimum_required(VERSION 3.14)
project(navmap_paper_bench)
set(CMAKE_CXX_STANDARD 23)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(navmap_core REQUIRED)
find_package(navmap_ros REQUIRED)
find_package(easynav_costmap_common REQUIRED)
find_package(easynav_costmap_maps_manager REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(bench_flat bench_flat.cpp)
target_link_libraries(bench_flat navmap_core::navmap_core navmap_ros::navmap_ros
  easynav_costmap_common::easynav_costmap_common
  easynav_costmap_maps_manager::easynav_costmap_maps_manager ${nav_msgs_TARGETS})

set(BONXAI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src/easynav_plugins/maps_managers/easynav_bonxai_maps_manager)
add_executable(bench_3d bench_3d.cpp ${BONXAI_DIR}/src/bonxai/probabilistic_map.cpp)
target_include_directories(bench_3d PRIVATE ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${BONXAI_DIR}/include)
target_link_libraries(bench_3d navmap_core::navmap_core navmap_ros::navmap_ros
  ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

# Planning benchmark: the planner plugins of EasyNav driven without a running system
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_costmap_planner REQUIRED)
find_package(easynav_navmap_planner REQUIRED)
find_package(easynav_navmap_maps_manager REQUIRED)
add_executable(bench_plan bench_plan.cpp)
target_link_libraries(bench_plan navmap_core::navmap_core navmap_ros::navmap_ros
  easynav_common::easynav_common
  easynav_costmap_common::easynav_costmap_common
  easynav_costmap_maps_manager::easynav_costmap_maps_manager
  easynav_navmap_maps_manager::easynav_navmap_maps_manager
  easynav_costmap_planner::easynav_costmap_planner
  easynav_navmap_planner::easynav_navmap_planner
  rclcpp_lifecycle::rclcpp_lifecycle ${nav_msgs_TARGETS})
