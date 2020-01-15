cmake_minimum_required(VERSION 2.8.3)
project(automap)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  ar_track_alvar_msgs
  rtabmap_ros
)

generate_messages(
  DEPENDENCIES
  nav_msgs
  sensor_msgs
  ar_track_alvar_msgs
  rtabmap_ros
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    nav_msgs
    ar_track_alvar_msgs
    sensor_msgs
    rtabmap_ros
)

## Include Directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
)


add_executable(marker2Map src/marker2Map.cpp)

add_dependencies(marker2Map ${catkin_EXPORTED_TARGETS})

target_link_libraries(marker2Map ${catkin_LIBRARIES})
