cmake_minimum_required(VERSION 3.0.2)
project(dynamic_voronoi)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS 
#  DEPENDS  
)

include_directories( 
    include 
    ${EIGEN3_INCLUDE_DIRS} 
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
    include/dynamic_voronoi/dynamicvoronoi.h
    include/dynamic_voronoi/bucketedqueue.hpp
    include/dynamic_voronoi/bucketedqueue.h
    src/dynamicvoronoi.cpp
)
