cmake_minimum_required(VERSION 3.0.2)
project(bonxai)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL REQUIRED COMPONENTS common io)
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
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} 
    include/bonxai/bonxai.hpp
    include/bonxai/pcl_utils.hpp
    include/bonxai/probabilistic_map.hpp
    src/pcl_utils.cpp
    src/probabilistic_map.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
)  
