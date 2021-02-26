cmake_minimum_required(VERSION 3.0.1)
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
  Box.msg
  Grasp.msg
  Location.msg
  Object.msg
  Slot.msg	  
  )

add_service_files(
  FILES
  AddBoxes.srv
  AddObjects.srv
  AddLocations.srv
  AddSlots.srv
  ListOfObjects.srv
  RemoveBoxes.srv
  RemoveLocations.srv
  RemoveObjects.srv
  RemoveObjectsFromSlot.srv
  RemoveSlots.srv
  ResetBoxes.srv
  ResetSlots.srv
  )

add_action_files(
  FILES
  PickObjects.action
  PlaceObjects.action
  GoTo.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs actionlib_msgs control_msgs sensor_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
