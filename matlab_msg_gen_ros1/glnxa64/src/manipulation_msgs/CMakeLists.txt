cmake_minimum_required(VERSION 3.5)
project(manipulation_msgs)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)
## System dependencies are found with CMake''s conventions
################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
    FILES
    Grasp.msg
    Location.msg
    Object.msg
    PickBox.msg
)
add_service_files(
    FILES
    AddBox.srv
    AddLocations.srv
    AddObjects.srv
    AddRemoveObjects.srv
    ListOfObjects.srv
    RemoveLocations.srv
)
find_package( geometry_msgs REQUIRED)
find_package( roscpp REQUIRED)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  "message_runtime"
  "geometry_msgs"
  "roscpp"
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
 "/usr/local/MATLAB/R2020b/extern/include"
 "/usr/local/MATLAB/R2020b/extern/include/MatlabDataArray"
 "/usr/local/MATLAB/R2020b/toolbox/ros/include/ros1"
 "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/include"
 "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/console_bridge/include"
 "/usr/local/MATLAB/R2020b/toolbox/ros/mlroscpp/custom_messages/include"
 "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/boost/include"
  "include/manipulation_msgs"
)
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
## Get list of .c files in project source directory
file(GLOB ${PROJECT_NAME}_C_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.c)
## Get list of .cpp files in project source directory
file(GLOB ${PROJECT_NAME}_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/*.cpp)
link_directories("/usr/local/MATLAB/R2020b/extern/bin/glnxa64")
link_directories("/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/console_bridge/lib")
link_directories("/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/lib")
link_directories("/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/boost/lib")
# Declare library
add_library(
    manipulation_msgs_matlab 
    SHARED
    src/manipulation_msgs_Grasp_common.cpp
    src/manipulation_msgs_msg_Grasp_publisher.cpp
    src/manipulation_msgs_msg_Grasp_subscriber.cpp
    src/manipulation_msgs_Location_common.cpp
    src/manipulation_msgs_msg_Location_publisher.cpp
    src/manipulation_msgs_msg_Location_subscriber.cpp
    src/manipulation_msgs_Object_common.cpp
    src/manipulation_msgs_msg_Object_publisher.cpp
    src/manipulation_msgs_msg_Object_subscriber.cpp
    src/manipulation_msgs_PickBox_common.cpp
    src/manipulation_msgs_msg_PickBox_publisher.cpp
    src/manipulation_msgs_msg_PickBox_subscriber.cpp
    src/manipulation_msgs_AddBox_request_common.cpp
    src/manipulation_msgs_AddBox_response_common.cpp
    src/manipulation_msgs_srv_AddBox_client.cpp
    src/manipulation_msgs_srv_AddBox_server.cpp
    src/manipulation_msgs_msg_AddBoxRequest_publisher.cpp
    src/manipulation_msgs_msg_AddBoxRequest_subscriber.cpp
    src/manipulation_msgs_msg_AddBoxResponse_publisher.cpp
    src/manipulation_msgs_msg_AddBoxResponse_subscriber.cpp
    src/manipulation_msgs_AddBoxRequest_common.cpp
    src/manipulation_msgs_AddBoxResponse_common.cpp
    src/manipulation_msgs_AddLocations_request_common.cpp
    src/manipulation_msgs_AddLocations_response_common.cpp
    src/manipulation_msgs_srv_AddLocations_client.cpp
    src/manipulation_msgs_srv_AddLocations_server.cpp
    src/manipulation_msgs_msg_AddLocationsRequest_publisher.cpp
    src/manipulation_msgs_msg_AddLocationsRequest_subscriber.cpp
    src/manipulation_msgs_msg_AddLocationsResponse_publisher.cpp
    src/manipulation_msgs_msg_AddLocationsResponse_subscriber.cpp
    src/manipulation_msgs_AddLocationsRequest_common.cpp
    src/manipulation_msgs_AddLocationsResponse_common.cpp
    src/manipulation_msgs_AddObjects_request_common.cpp
    src/manipulation_msgs_AddObjects_response_common.cpp
    src/manipulation_msgs_srv_AddObjects_client.cpp
    src/manipulation_msgs_srv_AddObjects_server.cpp
    src/manipulation_msgs_msg_AddObjectsRequest_publisher.cpp
    src/manipulation_msgs_msg_AddObjectsRequest_subscriber.cpp
    src/manipulation_msgs_msg_AddObjectsResponse_publisher.cpp
    src/manipulation_msgs_msg_AddObjectsResponse_subscriber.cpp
    src/manipulation_msgs_AddObjectsRequest_common.cpp
    src/manipulation_msgs_AddObjectsResponse_common.cpp
    src/manipulation_msgs_AddRemoveObjects_request_common.cpp
    src/manipulation_msgs_AddRemoveObjects_response_common.cpp
    src/manipulation_msgs_srv_AddRemoveObjects_client.cpp
    src/manipulation_msgs_srv_AddRemoveObjects_server.cpp
    src/manipulation_msgs_msg_AddRemoveObjectsRequest_publisher.cpp
    src/manipulation_msgs_msg_AddRemoveObjectsRequest_subscriber.cpp
    src/manipulation_msgs_msg_AddRemoveObjectsResponse_publisher.cpp
    src/manipulation_msgs_msg_AddRemoveObjectsResponse_subscriber.cpp
    src/manipulation_msgs_AddRemoveObjectsRequest_common.cpp
    src/manipulation_msgs_AddRemoveObjectsResponse_common.cpp
    src/manipulation_msgs_ListOfObjects_request_common.cpp
    src/manipulation_msgs_ListOfObjects_response_common.cpp
    src/manipulation_msgs_srv_ListOfObjects_client.cpp
    src/manipulation_msgs_srv_ListOfObjects_server.cpp
    src/manipulation_msgs_msg_ListOfObjectsRequest_publisher.cpp
    src/manipulation_msgs_msg_ListOfObjectsRequest_subscriber.cpp
    src/manipulation_msgs_msg_ListOfObjectsResponse_publisher.cpp
    src/manipulation_msgs_msg_ListOfObjectsResponse_subscriber.cpp
    src/manipulation_msgs_ListOfObjectsRequest_common.cpp
    src/manipulation_msgs_ListOfObjectsResponse_common.cpp
    src/manipulation_msgs_RemoveLocations_request_common.cpp
    src/manipulation_msgs_RemoveLocations_response_common.cpp
    src/manipulation_msgs_srv_RemoveLocations_client.cpp
    src/manipulation_msgs_srv_RemoveLocations_server.cpp
    src/manipulation_msgs_msg_RemoveLocationsRequest_publisher.cpp
    src/manipulation_msgs_msg_RemoveLocationsRequest_subscriber.cpp
    src/manipulation_msgs_msg_RemoveLocationsResponse_publisher.cpp
    src/manipulation_msgs_msg_RemoveLocationsResponse_subscriber.cpp
    src/manipulation_msgs_RemoveLocationsRequest_common.cpp
    src/manipulation_msgs_RemoveLocationsResponse_common.cpp
)
target_include_directories(manipulation_msgs_matlab PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries( manipulation_msgs_matlab
	"/usr/local/MATLAB/R2020b/extern/bin/glnxa64/libMatlabDataArray.so"
	"/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/console_bridge/lib/libconsole_bridge.so"
	"/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/lib/libclass_loader.so"
	"/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/lib/librosconsole.so"
	"/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/lib/libroscpp.so"
	"/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/lib/libroscpp_serialization.so"
	"/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/lib/librostime.so"
	${Boost_LIBRARIES}
	${CMAKE_DL_LIBS}
	)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
  manipulation_msgs_matlab
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(manipulation_msgs_matlab PRIVATE "MANIPULATION_MSGS_BUILDING_LIBRARY")
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS manipulation_msgs_matlab
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install (DIRECTORY m/ DESTINATION m/ FILES_MATCHING PATTERN "*.m")
# Generated 14-Oct-2020 13:58:50
# Copyright 2019-2020 The MathWorks, Inc.
