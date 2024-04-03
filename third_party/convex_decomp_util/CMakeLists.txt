cmake_minimum_required(VERSION 3.8.0)
project(convex_decomp_util)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS 
#  DEPENDS  
)

include_directories(
  include 
  ${DECOMP_UTIL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}  
  src/convex_decomp.cpp)

install(FILES "${PROJECT_NAME}-config.cmake" "${PROJECT_NAME}-config-version.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS convex_decomp_util
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/convex_decomp_util)
