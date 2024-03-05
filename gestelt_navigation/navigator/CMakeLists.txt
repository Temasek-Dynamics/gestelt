cmake_minimum_required(VERSION 3.0.2)
project(navigator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_compile_options(-std=c++17)

set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  gestelt_msgs
  global_planner
  sfc_generation
  traj_utils
  traj_opt
  gestelt_debug_msgs
) 

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS 
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  )

add_executable(navigator_node
  src/navigator_node.cpp 
  src/navigator.cpp 
)
target_link_libraries(navigator_node 
  ${catkin_LIBRARIES}
)


