cmake_minimum_required(VERSION 3.8)
project(radar_4_vox_map)

# C++17 Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Override Release flags to use O2 instead of O3 for numerical stability
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG -fno-fast-math")
  message(STATUS "Using custom Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

# Find dependencies
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
include(thrid_party_library/find_dependencies.cmake)

## Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

## Find other dependencies
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(OpenMP REQUIRED)

## build g2o
message("*** Information about build g2o ***")
message("Build g2o. It takes 1~5 minutes and require sudo permission")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/thrid_party_library/g2o/cmake_modules)
set(G2O_REBUILD OFF) # set to ON to rebuild g2o
if(G2O_REBUILD)
  execute_process(
    COMMAND bash rebuild_g2o.sh
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/thrid_party_library/g2o
    RESULT_VARIABLE result
  )
else()
  execute_process(
    COMMAND bash build_g2o.sh
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/thrid_party_library/g2o
    RESULT_VARIABLE result
  )
endif()
message("finish build g2o")

if (result)
  message(FATAL_ERROR "Error occurred while running the script: ${error_output}")
endif()

# For g2o
set(g2o_DIR "/usr/local/lib/cmake/g2o")
find_package(G2O REQUIRED) # g2o optimizer library
find_package(CSparse REQUIRED) # g2o's CSparse external dependency
find_package(Cholmod REQUIRED) # g2o's Cholmod external dependency
find_package(SuiteSparse REQUIRED) # g2o's SuiteSparse external dependency

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ros2/
)

## radar_4_vox_map library
# build radar_4_vox_map library as shared library
add_library(${PROJECT_NAME} STATIC
    src/utils.cpp
    src/radar_4_vox_map.cpp
    src/graph_optimizer/graph_optimizer.cpp
    src/graph_optimizer/vertex_PV_radar.cpp
    src/graph_optimizer/edge_PV_PV_prediction.cpp
    src/graph_optimizer/edge_PV_DICP.cpp
    src/graph_optimizer/edge_PV_doppler.cpp
    src/voxeled_rcs_mapper/voxeled_rcs_mapper.cpp
    ##### 2D implementations 
    src/voxeled_rcs_mapper/voxeled_rcs_mapper_2d.cpp
    src/graph_optimizer/graph_optimizer_2d.cpp
    src/graph_optimizer/edge_PV2_DICP.cpp
    src/graph_optimizer/edge_PV2_doppler.cpp
    src/graph_optimizer/edge_PV2_PV2_prediction.cpp
    src/graph_optimizer/vertex_PV2_radar.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${G2O_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
)

target_link_directories(${PROJECT_NAME}
    PRIVATE
    ${G2O_LIBRARY_DIR}
    ${CSPARSE_LIBRARY_DIR}
    ${CHOLMOD_LIBRARY_DIR}
)

target_link_libraries(${PROJECT_NAME}
    ${G2O_TYPES_DATA}
    ${G2O_CORE_LIBRARY}
    ${G2O_STUFF_LIBRARY}
    ${G2O_SOLVER_PCG}
    ${G2O_SOLVER_CSPARSE}  # be aware that CSPARSE is released under LGPL
    ${G2O_SOLVER_CSPARSE_EXTENSION}  # be aware that CSPARSE is released under LGPL
    ${G2O_SOLVER_CHOLMOD}  # be aware of that cholmod is released under GPL
    ${G2O_TYPES_SLAM3D}
    ${G2O_TYPES_SLAM3D_ADDONS}
    ${CSparse_LIBRARIES}  # Add SuiteSparse library
    ${Cholmod_LIBRARIES}  # Add SuiteSparse library
    ${SuiteSparse_LIBRARIES}  # Add SuiteSparse library
    TBB::tbb
)

# Add radar_4_vox_map_ros executable
add_executable(radar_4_vox_map_ros
  ros2/radar_4_vox_map_ros.cpp
)

target_compile_definitions(radar_4_vox_map_ros PRIVATE
  PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_include_directories(radar_4_vox_map_ros
  PRIVATE
  ros2
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

target_link_libraries(radar_4_vox_map_ros
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM3D}
  ${CSparse_LIBRARIES}
  ${Cholmod_LIBRARIES}
  ${SuiteSparse_LIBRARIES}
  cxsparse
)

# Use ament_target_dependencies for ROS2 dependencies
ament_target_dependencies(radar_4_vox_map_ros
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_msgs
  pcl_conversions
  ament_index_cpp
)

# Install targets
install(TARGETS ${PROJECT_NAME} radar_4_vox_map_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  ros2/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz files
install(DIRECTORY
  ros2/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()