cmake_minimum_required(VERSION 3.8)
project(simple_pointcloud_octomap_updater)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hector_worldmodel_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
add_library(simple_pointcloud_octomap_updater_core SHARED
            src/simple_pointcloud_octomap_updater.cpp)

set_target_properties(simple_pointcloud_octomap_updater_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(
  simple_pointcloud_octomap_updater_core
  hector_worldmodel_msgs
  message_filters
  moveit_core
  moveit_ros_occupancy_map_monitor
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)

set_target_properties(
  simple_pointcloud_octomap_updater_core
  PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(
  simple_pointcloud_octomap_updater_core
  PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
target_include_directories(
  simple_pointcloud_octomap_updater_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

add_library(simple_pointcloud_octomap_updater SHARED src/plugin_init.cpp)
set_target_properties(simple_pointcloud_octomap_updater
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(
  simple_pointcloud_octomap_updater
  hector_worldmodel_msgs
  message_filters
  moveit_core
  moveit_ros_occupancy_map_monitor
  pluginlib
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs)
target_link_libraries(simple_pointcloud_octomap_updater
                      simple_pointcloud_octomap_updater_core)

target_include_directories(
  simple_pointcloud_octomap_updater
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS simple_pointcloud_octomap_updater
          simple_pointcloud_octomap_updater_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_perception)

pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  "simple_pointcloud_octomap_updater_plugin_description.xml")

ament_package()
