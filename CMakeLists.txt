cmake_minimum_required(VERSION 2.8.3)
project(blurr_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  freefloating_gazebo
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blurr_base
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_control src/joy_control.cpp)
target_link_libraries(joy_control   ${catkin_LIBRARIES})