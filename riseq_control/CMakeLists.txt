cmake_minimum_required(VERSION 2.8.3)
project(riseq_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  riseq_msgs
  riseq_common  
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES riseq_control
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp rospy std_msgs riseq_common riseq_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/position_controller.cpp)
add_library(${PROJECT_NAME}_fc src/fast_controller_node.cpp)
add_library(feedback_linearization_controller libs/feedback_linearization_controller/feedback_linearization_controller.cpp)
add_library(fast_controller libs/fast_controller/fast_controller.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/riseq_control_node.cpp)

#add_executable(position_controller_node src/position_controller_node.cpp)
#target_link_libraries(position_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(position_controller src/position_controller.cpp)
#target_link_libraries(position_controller ${catkin_LIBRARIE  S})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})

#add_executable(position_controller src/position_controller.cpp)
#target_link_libraries(position_controller ${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(position_controller riseq_control_gencpp)

add_executable(position_controller_node src/position_controller_node.cpp)
target_link_libraries(position_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(position_controller_node ${PROJECT_NAME}_fc)
target_link_libraries(position_controller_node feedback_linearization_controller)
target_link_libraries(position_controller_node fast_controller)


#add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})

#add_dependencies(position_controller_node riseq_control_gencpp)

