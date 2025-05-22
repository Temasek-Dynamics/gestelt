cmake_minimum_required(VERSION 3.0.2)
project(model_inference)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17 -g") #  -O0")
# set(CMAKE_PREFIX_PATH "/opt/libtorch" ${CMAKE_PREFIX_PATH})
set(TORCH_DIR /opt/libtorch/share/cmake/Torch)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
COMPONENTS
  roscpp
  std_msgs
  gestelt_msgs
  tf2
  tf2_ros
  geometry_msgs
)
 
find_package(Torch PATHS ${TORCH_DIR} REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    tf2 
    tf2_ros
    gestelt_msgs
)

message(STATUS "catkin_LIBRARIES: ${catkin_LIBRARIES}")
message(STATUS "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
)

## Declare a C++ library
file(GLOB SRC_FILES 
    src/torch_engine.cpp) 
add_library(${PROJECT_NAME}_shared_lib ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}_shared_lib
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/inference_node.cpp
)
add_dependencies(${PROJECT_NAME}_node 
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Threads::Threads
  ${TORCH_LIBRARIES}
  ${PROJECT_NAME}_shared_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_model_inference.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
