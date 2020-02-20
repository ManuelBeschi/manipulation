cmake_minimum_required(VERSION 2.8.3)
project(manipulation_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  control_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
)

add_message_files(
  FILES
  Grasp.msg
  Object.msg
  PickBox.msg
  )
add_service_files(
  FILES
  AddBox.srv
  AddObjects.srv
  AddRemoveObjects.srv
  ListOfObjects.srv
  )
add_action_files(
  FILES
  PickPlan.action
  PickExecute.action
  TeachPickingPose.action
  PickObjects.action
  PlaceObjects.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   std_msgs  actionlib_msgs geometry_msgs control_msgs sensor_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
