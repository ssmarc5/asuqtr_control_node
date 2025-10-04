cmake_minimum_required(VERSION 2.8.3)
project(asuqtr_control_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  genmsg
  actionlib_msgs
  actionlib
  message_generation
)
catkin_python_setup()

add_action_files(DIRECTORY action FILES Control.action)

add_service_files(
   FILES
   TuneMatrix.srv
   ToggleLqr.srv
   UpdateAngleThreshold.srv
   UpdateAsRate.srv
   UpdateAttenuation.srv
   UpdateMaxLqrRate.srv
   UpdatePosThreshold.srv
 )

generate_messages(DEPENDENCIES actionlib_msgs)



# add_message_files(DIRECTORY msg FILES
# generate_messages(DEPENDENCIES std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  message_runtime
)

#catkin_install_python(
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
