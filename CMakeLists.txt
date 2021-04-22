#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(coordination_robot_pkg)

###########################################################################
## Find catkin packages and libraries for catkin and system dependencies ##
###########################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  message_generation
)

##########################################
## Setup for python modules and scripts ##
##########################################
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Waypoint_init.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ServiceRegistration.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  # nav_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs rospy message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/coordination_robot
  nodes/tf_broadcastor
  nodes/waypoint_send
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
