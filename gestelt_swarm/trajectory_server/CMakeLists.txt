cmake_minimum_required(VERSION 3.0.2)
project(trajectory_server)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  traj_opt
  # traj_utils
  trajectory_server_msgs
)

# set(ROS_DEPENDS
# )

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES trajectory_server
#  CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(traj_server_node 
  src/traj_server_node.cpp
  src/traj_server.cpp
)
target_link_libraries(traj_server_node 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


