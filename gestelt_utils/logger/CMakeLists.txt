cmake_minimum_required(VERSION 3.0.2)
project(logger)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_compile_options(-std=c++17)

# set(ROS_LIBRARIES
#   roscpp
# ) 

find_package(catkin REQUIRED COMPONENTS
  # ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS 
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library( ${PROJECT_NAME} 
  # include/logger/logger.h
  src/timer.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


