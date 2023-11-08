cmake_minimum_required(VERSION 3.0.2)
project(fake_map)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set (ROS_LIBRARIES
  roscpp
  geometry_msgs
  pcl_ros
  sensor_msgs
  tf2_ros
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
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  CATKIN_DEPENDS ${ROS_DEPENDS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(fake_laser 
  src/fake_laser.cpp
)

target_include_directories(fake_laser PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(fake_laser
  Eigen3::Eigen
)

add_executable (fake_map_node
  src/fake_map_node.cpp
  src/fake_map.cpp
)
target_link_libraries(fake_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  fake_laser
)

add_executable (fake_map_publisher
  src/fake_map_publisher.cpp
)
target_link_libraries(fake_map_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable (fake_map_generator
  src/fake_map_generator.cpp
)
target_link_libraries(fake_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)