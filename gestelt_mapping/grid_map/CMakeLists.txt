cmake_minimum_required(VERSION 3.0.2)
project(grid_map)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(ROS_LIBRARIES
    roscpp
    tf2_ros
    std_msgs
    sensor_msgs
    message_filters
    # cv_bridge
    # swarm_benchmark
    trajectory_server_msgs
)

# set(ROS_DEPENDS
#     roscpp
# )

find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    ${ROS_LIBRARIES}
)
# find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES grid_map
#  CATKIN_DEPENDS ${ROS_DEPENDS}
 DEPENDS PCL
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    # ${OpenCV_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

# add_definitions(${PCL_DEFINITIONS})
# link_directories(${PCL_LIBRARY_DIRS})

add_library(grid_map 
    src/grid_map.cpp 
)

target_link_libraries(grid_map
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)  
