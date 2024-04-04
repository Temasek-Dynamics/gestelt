cmake_minimum_required(VERSION 3.0.2)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traj_utils
  sensor_msgs
)

catkin_package(
#  CATKIN_DEPENDS traj_utils
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node_udp
  src/bridge_node_udp.cpp 
  )
target_link_libraries(bridge_node_udp 
  ${catkin_LIBRARIES}
  )

add_executable(swarm_bridge
  src/swarm_bridge.cpp 
  src/swarm_bridge_node.cpp 
)
target_link_libraries(swarm_bridge 
  ${catkin_LIBRARIES}
)


