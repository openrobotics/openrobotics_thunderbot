cmake_minimum_required( VERSION 2.8.3 )
project( talos_base_controller )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME} src
#  LIBRARIES base_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set( Boost_USE_STATIC_LIBS on )
find_package( Boost REQUIRED COMPONENTS system thread)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/libs" include
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)


add_subdirectory( libs )

message ("nothing to see here")

## Declare a cpp library
add_library( odometry_manager
   src/odometry_manager.cc
 )
add_library(parser src/parser.cpp)

## Declare a cpp executable
add_executable( base_controller_node src/base_controller_node.cpp )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(base_controller_node base_controller_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(base_controller_node
  ${Boost_LIBRARIES} 
  BufferedAsyncSerial
  odometry_manager
  parser
  ${catkin_LIBRARIES}
)

target_link_libraries( odometry_manager
  ${catkin_LIBRARIES}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS base_controller base_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
message ("building tests for ${PROJECT_NAME}")

## Add gtest based cpp test target and link libraries
 catkin_add_gtest(odometry-manager-test test/odometry_manager_test.cc)
 if(TARGET odometry-manager-test)
   target_link_libraries(odometry-manager-test odometry_manager)
 endif()

catkin_add_gtest(parser_test test/parser_test.cpp)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
