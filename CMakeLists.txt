cmake_minimum_required(VERSION 2.8.3)
project(speed_control)

add_definitions(-std=gnu++11)

set(CMAKE_BUILD_TYPE "Debug")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mrtstar
  tf
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/config_tool.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs dynamic_reconfigure visualization_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(speed_controller src/waypoints_follower.cpp src/waypoints_follower_main.cpp src/abstract_controller.cpp)
target_link_libraries(speed_controller ${catkin_LIBRARIES})
add_dependencies(speed_controller ${PROJECT_NAME}_gencfg)

add_executable(test_speed_controller src/test_follower.cpp)
target_link_libraries(test_speed_controller ${catkin_LIBRARIES})