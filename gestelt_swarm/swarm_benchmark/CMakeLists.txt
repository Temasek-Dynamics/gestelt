cmake_minimum_required(VERSION 3.0.2)
project(swarm_benchmark)

add_compile_options(-std=c++14)

set (ROS_LIBRARIES
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS 
  include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${ROS_LIBRARIES}
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