cmake_minimum_required(VERSION 3.0.2)
project(gestelt_msgs)

add_compile_options(-std=c++17)

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
  Sphere.msg
  SphericalSFCTrajectory.msg

  Goals.msg
  Command.msg
  BoolMap.msg
  BoolMapArray.msg
  CommanderState.msg
  PlanRequestDebug.msg
  ExecTrajectory.msg
  FrontEndPlan.msg
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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gestelt_msgs
 CATKIN_DEPENDS ${MSG_DEP} message_runtime
#  DEPENDS system_lib
)
