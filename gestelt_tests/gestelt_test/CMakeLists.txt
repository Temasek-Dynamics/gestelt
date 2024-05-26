cmake_minimum_required(VERSION 3.0.2)
project(gestelt_test)

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_compile_options(-std=c++17)

set(ROS_LIBRARIES
  roscpp
  traj_opt
  global_planner
  sfc_generation
  grid_map
) 

find_package(benchmark)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS 
)

include_directories(
  # include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

############################################

function(compile_benchmark TARGET)
    target_compile_definitions(${TARGET} PUBLIC
        DATA_PATH="${PROJECT_SOURCE_DIR}/data/")
    target_include_directories(${TARGET} PUBLIC
        ${PCL_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PUBLIC
        benchmark::benchmark
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES})
endfunction()

############################################

set(BENCHMARK_SRC src/benchmark_front_end.cpp)

add_executable(benchmark_front_end ${BENCHMARK_SRC})
compile_benchmark(benchmark_front_end)

# add_executable(test_traj_opt
#   src/test_traj_opt.cpp
# )
# target_link_libraries(test_traj_opt 
#   ${catkin_LIBRARIES}
#   benchmark::benchmark
# )

