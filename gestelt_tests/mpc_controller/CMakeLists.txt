cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(ROS_LIBRARIES
  roscpp
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
#  INCLUDE_DIRS 
  # include
#  LIBRARIES gestelt_bringup
#  CATKIN_DEPENDS  
#  DEPENDS 
)

###########
## Build ##
###########