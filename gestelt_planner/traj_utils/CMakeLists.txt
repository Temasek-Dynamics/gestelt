cmake_minimum_required(VERSION 3.0.2)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(ENABLE_PRECOMPILED_HEADERS "OFF")
add_compile_options(-std=c++14)

set(ROS_LIBRARIES
  roscpp
  message_generation
)

set(MSG_DEP
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES} ${MSG_DEP}
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  MINCOTraj.msg
)
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    ${MSG_DEP}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_utils 
  CATKIN_DEPENDS message_runtime
  #  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
)

add_library( traj_utils
    src/planning_visualization.cpp 
)
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    ${EIGEN3_LIBRARIES}
)  
