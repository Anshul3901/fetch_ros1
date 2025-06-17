cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  rosbag
  sensor_msgs
  tf
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS pcl_ros rosbag roscpp rospy sensor_msgs tf
)

## Enable Python module support
catkin_python_setup()

## Include header directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare the demo executable
add_executable(perception_save_cloud_main src/save_cloud_main.cpp)
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "")
add_dependencies(perception_save_cloud_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_save_cloud_main
  ${catkin_LIBRARIES}
)


## Install any Python scripts from the scripts/ folder (optional if used)
# install(PROGRAMS
#   scripts/your_script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
