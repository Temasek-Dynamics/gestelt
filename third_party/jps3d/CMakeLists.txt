cmake_minimum_required(VERSION 3.8)
# cmake_minimum_required(VERSION 3.0.2)
project(jps3d)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_compile_options(-std=c++17)

set(ROS_LIBRARIES
)

find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES jps3d # Exported libraries
#  CATKIN_DEPENDS # Export grid_map as part of this package
#  DEPENDS system_lib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
)

add_library(jps3d 
  src/distance_map_planner/distance_map_planner.cpp
  src/distance_map_planner/graph_search.cpp

  src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(jps3d
  ${catkin_LIBRARIES}
)
