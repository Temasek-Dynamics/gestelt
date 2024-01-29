cmake_minimum_required(VERSION 3.0.2)
project(front_end_planner)

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

add_executable(front_end_planner_node
  src/front_end_planner_node.cpp 
  src/front_end_planner.cpp 
)
target_link_libraries(front_end_planner_node 
  ${catkin_LIBRARIES}
)


