cmake_minimum_required(VERSION 3.0.2)
project(swarm_collision_checker)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
) 

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES swarm_collision_checker # Exported libraries
#  CATKIN_DEPENDS  # Export grid_map as part of this package
#  DEPENDS 
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(swarm_collision_checker_node
  src/swarm_collision_checker.cpp 
  src/swarm_collision_checker_node.cpp 
)

target_link_libraries(swarm_collision_checker_node 
  ${catkin_LIBRARIES}
)



