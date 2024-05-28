cmake_minimum_required(VERSION 3.0.2)
project(dynamic_brushfire)

# set(CMAKE_BUILD_TYPE "Release")
# add_compile_options(-std=c++17)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  nav_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)
find_package(SDL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamic_brushfire # Exported libraries
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
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
add_library(static_brushfire 
  src/static_brushfire.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(static_brushfire
  ${catkin_LIBRARIES}
  SDL SDL_image
)


add_executable(static_brushfire_node
  src/dynamicvoronoi.cpp
  src/static_brushfire_node.cpp 
)
target_link_libraries(static_brushfire_node 
  ${catkin_LIBRARIES}
  static_brushfire
)



