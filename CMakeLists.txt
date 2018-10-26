cmake_minimum_required(VERSION 2.8.3)
project(interactive_marker_tf2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  rospy
  tf2
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS geometry_msgs interactive_markers rospy tf2 visualization_msgs
#  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/int_marker_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/basic_controls.launch
  launch/pong.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
