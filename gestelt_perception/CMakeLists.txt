cmake_minimum_required(VERSION 3.0.2)
project(gestelt_perception)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  tf2_geometry_msgs 
  tf2
  gestelt_msgs
)

find_package( OpenCV REQUIRED )



catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES gestelt_perception
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


include_directories( 
      include
      ${OpenCV_INCLUDE_DIRS} 
      ${catkin_INCLUDE_DIRS}
)

add_executable( drone_vision 
                  src/drone_vision.cpp
                  src/drone_vision_node.cpp
)


target_link_libraries( drone_vision 
                                ${catkin_LIBRARIES}
                                ${OpenCV_LIBS} 
)
