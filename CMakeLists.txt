cmake_minimum_required(VERSION 3.0.2)
project(mission_manager)


## Find catkin packages first
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_msgs
  std_srvs
  smach
  smach_ros
  smach_msgs
  rqt_gui
  rqt_gui_py
  python_qt_binding
  message_generation
  cavity_detection_msgs
)

## Declare message files
add_message_files(
  FILES
  Context.msg
)

add_service_files(
  FILES
  Trigger.srv
)

add_action_files(
  DIRECTORY action
  FILES 
  ArmTrajectoryProcessing.action
  BaseTrajectoryProcessing.action
)

catkin_python_setup()

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  actionlib_msgs
  cavity_detection_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
