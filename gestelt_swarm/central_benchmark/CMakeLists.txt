cmake_minimum_required(VERSION 3.0.2)
project(central_benchmark)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
  roscpp
  rospy
  gazebo_msgs
  geometry_msgs
  visualization_msgs
  gestelt_msgs
  tf2_ros
)

# set(ROS_DEPENDS
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
#  INCLUDE_DIRS 
  # include
#  LIBRARIES central_benchmark
#  CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS Eigen 
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(swarm_collision_checker src/swarm_collision_checker.cpp)
target_link_libraries(swarm_collision_checker
  ${catkin_LIBRARIES}
)

# add_executable(benchmark_aggregator src/benchmark_aggregator.cpp)
# target_link_libraries(benchmark_aggregator
#   ${catkin_LIBRARIES}
# )
# add_dependencies(benchmark_aggregator ${catkin_EXPORTED_TARGETS})

add_executable(cloud_downsampler src/cloud_downsampler.cpp)
target_link_libraries(cloud_downsampler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

