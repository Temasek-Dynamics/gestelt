cmake_minimum_required(VERSION 3.0.2)
project(swarm_benchmark)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set (ROS_LIBRARIES
  roscpp
)

# set (ROS_DEPENDS
# )

find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS 
  include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS Eigen Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/timebenchmark.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(hardware_monitor src/hardware_monitor.cpp)
target_link_libraries(hardware_monitor
  ${catkin_LIBRARIES}
)