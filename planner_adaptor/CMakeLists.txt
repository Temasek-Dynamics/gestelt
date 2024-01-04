cmake_minimum_required(VERSION 3.0.2)
project(planner_adaptor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  gestelt_msgs
  traj_opt
  traj_utils
)

# set (ROS_DEPENDS
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES planner_adaptor
#  CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

add_executable(ego_planner_adaptor_node src/ego_planner_adaptor_node.cpp)
add_dependencies(ego_planner_adaptor_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ego_planner_adaptor_node
  ${catkin_LIBRARIES}
)
