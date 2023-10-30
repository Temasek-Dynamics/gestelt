cmake_minimum_required(VERSION 3.0.2)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  grid_map
  # cv_bridge
)

set(ROS_DEPENDS
  grid_map
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_searching
  CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
)

add_library( path_searching 
  src/dyn_a_star.cpp
)
target_link_libraries( path_searching
  ${catkin_LIBRARIES} 
  # ${EIGEN3_LIBRARIES}
)  
