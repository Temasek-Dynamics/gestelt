# A CMakeLists.txt to show how to use Ruckig with CMake when library and headers are in a known directory.

cmake_minimum_required(VERSION 3.10)
project(ruckig_planner)

set(ROS_LIBRARIES
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  gestelt_msgs
  controller_msgs
  rospy
  nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ruckig_planner
#  CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
# link_directories(lib)

# Build the position example
# add_executable(example_position src/example_1.cpp)
# target_compile_features(example_position PUBLIC cxx_std_17)



# target_link_libraries(example_position 
#   ${catkin_LIBRARIES}
#   ${EIGEN3_LIBRARIES}
# )