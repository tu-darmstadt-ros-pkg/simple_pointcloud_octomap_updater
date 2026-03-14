cmake_minimum_required(VERSION 3.8)
project(simple_pointcloud_octomap_updater)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# --- Dependencies -----------------------------------------------------------
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

find_package(octomap REQUIRED)

# --- Core library -----------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
            src/simple_pointcloud_octomap_updater.cpp)

target_link_libraries(${PROJECT_NAME}_core
  OpenMP::OpenMP_CXX
  ${OCTOMAP_LIBRARIES})

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${OCTOMAP_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}_core ${DEPENDENCIES})
set_target_properties(${PROJECT_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

# --- Plugin library ---------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/plugin_init.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# --- Dataset generator (installed as a runnable tool) -----------------------
add_executable(generate_test_dataset
               test/generate_test_dataset.cpp)

target_include_directories(generate_test_dataset PRIVATE
  ${OCTOMAP_INCLUDE_DIRS})

target_link_libraries(generate_test_dataset
  ${OCTOMAP_LIBRARIES})

# --- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hector_testing_utils REQUIRED)

  ament_add_gtest(test_simple_pointcloud_updater
                  test/test_simple_octomap_updater.cpp)

  target_link_libraries(test_simple_pointcloud_updater ${PROJECT_NAME}_core)

  ament_target_dependencies(test_simple_pointcloud_updater
    hector_testing_utils
    hector_worldmodel_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
    moveit_ros_occupancy_map_monitor
    moveit_core)

  target_include_directories(test_simple_pointcloud_updater PRIVATE include)

  # Make the dataset generator a test fixture so the .ot file is always
  # freshly created before the file-based tests run.
  add_test(NAME generate_test_dataset
           COMMAND generate_test_dataset
                   ${CMAKE_CURRENT_BINARY_DIR}/test_octomap.ot)

  # Pass the generated path to the GTest binary so file-based tests can find it
  set_tests_properties(test_simple_pointcloud_updater PROPERTIES
    ENVIRONMENT "TEST_OCTOMAP_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_octomap.ot"
    DEPENDS generate_test_dataset)
endif()

# --- Installation & Export --------------------------------------------------
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core generate_test_dataset
  EXPORT  export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  "simple_pointcloud_octomap_updater_plugin_description.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_export_dependencies(${DEPENDENCIES})

ament_package()