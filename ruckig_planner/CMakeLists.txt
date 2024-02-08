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
  trajectory_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

catkin_package(
 CATKIN_DEPENDS ${ROS_DEPENDS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(ruckig_trajectory_generator 
  src/ruckig_trajectory_generator.cpp
  src/ruckig_trajectory_generator_node.cpp
)
add_dependencies(ruckig_trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(ruckig_trajectory_generator PUBLIC cxx_std_17)
target_link_libraries(ruckig_trajectory_generator PUBLIC ${catkin_LIBRARIES} ruckig::ruckig)

install(TARGETS ruckig_trajectory_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)