cmake_minimum_required(VERSION 3.0.2)
project(ikd_tree)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(PCL REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories( 
    include 
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
    include/ikd_tree/ikd_tree.h
    src/ikd_tree.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
)  
