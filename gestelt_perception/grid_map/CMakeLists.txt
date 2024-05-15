cmake_minimum_required(VERSION 3.0.2)
project(grid_map)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


set(ROS_LIBRARIES
    roscpp
    tf2_ros
    std_msgs
    sensor_msgs
    message_filters
    bonxai
    ikd_tree
    logger
)

find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS tf2_ros bonxai ikd_tree logger
 DEPENDS PCL 
)

include_directories( 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
    src/grid_map.cpp 
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)  
