cmake_minimum_required(VERSION 3.0.2)
project(radxa_utils)

add_compile_options(-std=c++14)

set (ROS_LIBRARIES
  roscpp
  std_msgs
)

# set (ROS_DEPENDS

# )

find_package(catkin REQUIRED COMPONENTS
  ${ROS_LIBRARIES}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gestelt_mapping
  # CATKIN_DEPENDS ${ROS_DEPENDS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(listener src/listener.cpp)
# add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(listener
#   ${catkin_LIBRARIES}
# )

# add_executable(talker src/talker.cpp)
# add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(talker
#   ${catkin_LIBRARIES}
# )
