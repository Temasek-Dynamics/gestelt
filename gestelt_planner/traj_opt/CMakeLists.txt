cmake_minimum_required(VERSION 3.0.2)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
    roscpp
    std_msgs
    grid_map
    path_searching
    traj_utils
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_opt
 CATKIN_DEPENDS ${ROS_LIBRARIES} 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
)

add_library(traj_opt 
    src/poly_traj_optimizer.cpp 
)
target_link_libraries(traj_opt
    ${catkin_LIBRARIES} 
    ${EIGEN3_LIBRARIES}
)  
