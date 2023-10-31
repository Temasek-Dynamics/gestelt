cmake_minimum_required(VERSION 3.0.2)
project(fake_drone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17")

set (ROS_LIBRARIES
  geometry_msgs
  roscpp
  tf2_ros
)

# set (ROS_DEPENDS
# )

find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  CATKIN_DEPENDS ${ROS_DEPENDS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (fake_drone 
  src/fake_drone_node.cpp
  src/fake_drone.cpp
)
target_link_libraries(fake_drone
  ${catkin_LIBRARIES}
)