cmake_minimum_required(VERSION 3.0.2)
project(trajectory_server_msgs)

add_compile_options(-std=c++14)

set(MSG_DEP
  std_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP}
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  State.msg
  TrackingError.msg
  Waypoints.msg
  TimeBenchmark.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ${MSG_DEP}
)
