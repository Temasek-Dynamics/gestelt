cmake_minimum_required(VERSION 3.0.2)
project(sfc_generation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(ROS_LIBRARIES
  roscpp
  grid_map
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} # Exported libraries
#  CATKIN_DEPENDS  # Export grid_map as part of this package
#  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

## Declare a C++ library
add_library( ${PROJECT_NAME} 
  src/spherical_sfc.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
