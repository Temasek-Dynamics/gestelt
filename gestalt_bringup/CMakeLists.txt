cmake_minimum_required(VERSION 3.0.2)
project(gestalt_bringup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set (ROS_LIBRARIES
  roscpp
  rospy
  geometry_msgs
  visualization_msgs
  tf2_ros
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
#  INCLUDE_DIRS 
  # include
#  LIBRARIES gestalt_bringup
 CATKIN_DEPENDS ${ROS_LIBRARIES}
 DEPENDS Eigen Boost
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
)

add_executable(uav_tf_broadcaster src/uav_tf_broadcaster.cpp)
add_dependencies(uav_tf_broadcaster 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(uav_tf_broadcaster
  ${catkin_LIBRARIES}
)

add_executable(swarm_collision_checker src/swarm_collision_checker.cpp)
add_dependencies(swarm_collision_checker 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(swarm_collision_checker
  ${catkin_LIBRARIES}
)
