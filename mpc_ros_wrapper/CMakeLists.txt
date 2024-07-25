cmake_minimum_required(VERSION 3.0.2)
project(mpc_ros_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -DACADOS_WITH_QPOASES -DACADOS_WITH_HPIPM )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gestelt_msgs
  mavros_msgs
  roscpp
  std_msgs
  gestelt_bringup
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
set(ACADOS_SOURCE_DIR $ENV{HOME}/acados)
set(acados_include ${ACADOS_SOURCE_DIR}/include)
set(solver_code ../gestelt_bringup/src/Learning_Agile/c_generated_code/)


catkin_package(
    INCLUDE_DIRS include
    ${acados_include}/blasfeo/include/
    ${acados_include}/hpipm/include/
    ${acados_include}/
    # ${acados_include}/daqp/
    ${acados_include}/qpOASES_e/
    ${acados_include}/osqp/
    ${acados_include}/../
    ${solver_code}
    ${catkin_INCLUDE_DIRS}
    LIBRARIES mpc_ros_wrapper
#  CATKIN_DEPENDS gestelt_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/
  # ${acados_include}/daqp/
  ${acados_include}/qpOASES_e/
  ${acados_include}/osqp/
  ${acados_include}/../
  ${solver_code}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  $ENV{ACADOS_SOURCE_DIR}/lib
)


## Declare a C++ library

file(GLOB ocp_solver
        ../gestelt_bringup/src/Learning_Agile/c_generated_code/acados_solver_ACADOS_model.c
        ../gestelt_bringup/src/Learning_Agile/c_generated_code/ACADOS_model_cost/*.c
        )

file(GLOB casadi_fun
        ../gestelt_bringup/src/Learning_Agile/c_generated_code/ACADOS_model_model/ACADOS_model_expl_ode_fun.c
        ../gestelt_bringup/src/Learning_Agile/c_generated_code/ACADOS_model_model/ACADOS_model_expl_vde_forw.c)


add_library(ocp_shared_lib SHARED ${ocp_solver} ${casadi_fun} )
target_link_libraries(ocp_shared_lib acados qpOASES_e hpipm blasfeo)

add_library(${PROJECT_NAME} SHARED
            src/mpc_ros_wrapper_node.cpp
            src/mpc_ros_wrapper.cpp
            )
target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES}
                      ocp_shared_lib 
                      )
                      
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} PUBLIC include)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mpc_ros_wrapper_node src/mpc_ros_wrapper_node.cpp
                                    src/mpc_ros_wrapper.cpp
                                    )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(mpc_ros_wrapper_node
  ocp_shared_lib 
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mpc_ros_wrapper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
