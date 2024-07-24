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
  gestelt_msgs
  # controller_msgs
  mpc_ros_wrapper
)

# set(ROS_DEPENDS
# )

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
  mpc_ros_wrapper
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES trajectory_server
 CATKIN_DEPENDS mpc_ros_wrapper
 CATKIN_DEPENDS ${ROS_DEPENDS}
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  # ${mpc_ros_wrapper_INCLUDE_DIRS}
)

add_executable(traj_server_node 
  src/traj_server_node.cpp
  src/traj_server.cpp
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)
target_link_libraries(traj_server_node 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)


# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})




