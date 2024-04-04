cmake_minimum_required(VERSION 3.0.2)
project(decomp_util)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} # Exported libraries
#  CATKIN_DEPENDS # 
#  DEPENDS system_lib
)
  
include_directories(
  include 
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}  
  src/decomp_util.cpp
)

# include(CTest)

# add_executable(test_seed_decomp test/test_seed_decomp.cpp)
# target_link_libraries(test_seed_decomp ${Boost_LIBRARIES})
# add_test(test_seed_decomp test_seed_decomp)

# add_executable(test_line_segment test/test_line_segment.cpp)
# target_link_libraries(test_line_segment ${Boost_LIBRARIES})
# add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
# target_link_libraries(test_ellipsoid_decomp ${Boost_LIBRARIES})
# add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
# target_link_libraries(test_iterative_decomp ${Boost_LIBRARIES})
# add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/decomp_basis include/decomp_geometry include/decomp_util
  DESTINATION include)
