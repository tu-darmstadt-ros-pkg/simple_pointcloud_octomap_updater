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

# --- Dependencies List ---
# nav_msgs added for the optional 2D OccupancyGrid output
set(DEPENDENCIES
    Eigen3
    hector_worldmodel_msgs
    message_filters
    moveit_core
    moveit_msgs
    moveit_ros_occupancy_map_monitor
    nav_msgs
    OpenMP
    pluginlib
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    urdf
    visualization_msgs)

foreach(dep ${DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

# --- Core Library ---
add_library(${PROJECT_NAME}_core SHARED
            src/simple_pointcloud_octomap_updater.cpp)

target_link_libraries(${PROJECT_NAME}_core OpenMP::OpenMP_CXX)

target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_core ${DEPENDENCIES})

set_target_properties(${PROJECT_NAME}_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# --- Plugin Library ---
add_library(${PROJECT_NAME} SHARED src/plugin_init.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hector_testing_utils REQUIRED)

  ament_add_gtest(test_simple_pointcloud_updater
                  test/test_simple_octomap_updater.cpp)

  target_link_libraries(test_simple_pointcloud_updater ${PROJECT_NAME}_core)

  ament_target_dependencies(
    test_simple_pointcloud_updater
    hector_testing_utils
    hector_worldmodel_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
    moveit_ros_occupancy_map_monitor
    moveit_core)

  target_include_directories(test_simple_pointcloud_updater PRIVATE include)
endif()

# --- Installation & Export ---
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  "simple_pointcloud_octomap_updater_plugin_description.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_export_dependencies(${DEPENDENCIES})

ament_package()