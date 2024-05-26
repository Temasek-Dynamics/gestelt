cmake_minimum_required(VERSION 3.0.2)
project(perception_tools)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(ROS_LIBRARIES
  roscpp
  sensor_msgs
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
#  LIBRARIES perception_tools
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

add_executable(cloud_downsampler src/cloud_downsampler.cpp)
target_link_libraries(cloud_downsampler
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

