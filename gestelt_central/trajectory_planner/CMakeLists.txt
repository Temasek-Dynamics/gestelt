cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

add_compile_options(-std=c++11)

set (ROS_PKGS
  roscpp
  mav_trajectory_generation
  mav_trajectory_generation_ros
  nav_msgs
  eigen_conversions
  trajectory_server_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${ROS_PKGS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${ROS_PKGS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory_planner.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(trajectory_generator
  src/example_planner.cc
  src/example_planner_node.cc
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_generator
  ${catkin_LIBRARIES}
)

add_executable(trajectory_sampler
  src/trajectory_sampler.cpp
)
target_link_libraries(trajectory_sampler ${catkin_LIBRARIES})
