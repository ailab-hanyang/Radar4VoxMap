cmake_minimum_required(VERSION 3.0.2)
project(radar_4_vox_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
include(thrid_party_library/find_dependencies.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(OpenMP REQUIRED) 

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    topic_tools
    roslib # ros::package for finding package path
    pcl_conversions
    rosbag
    visualization_msgs
    jsk_recognition_msgs
    tf2
    tf2_eigen
)

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
find_package(G2O REQUIRED) # g2o optimizer library
find_package(CSparse REQUIRED) # g2o's CSparse external dependency
find_package(Cholmod REQUIRED) # g2o's Cholmod external dependency
find_package(SuiteSparse REQUIRED) # g2o's SuiteSparse external dependency

# Call catkin_package() to generate package configuration files
catkin_package(
  INCLUDE_DIRS include ros
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs visualization_msgs jsk_recognition_msgs tf2 tf2_eigen
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ros/
)

## radar_4_vox_map library
# build radar_4_vox_map library as shared library
ADD_LIBRARY(${PROJECT_NAME} STATIC
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
  ros/radar_4_vox_map_ros.cpp
)

target_include_directories(radar_4_vox_map_ros
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ros
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

target_link_libraries(radar_4_vox_map_ros
  ${catkin_LIBRARIES}
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

# Install targets
install(TARGETS ${PROJECT_NAME} radar_4_vox_map_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)