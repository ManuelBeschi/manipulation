cmake_minimum_required(VERSION 3.10.0)
project(go_to_location)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  moveit_planning_helper
  manipulation_msgs
  manipulation_utils
  roscpp
  rosdyn_core
  rosparam_utilities
  tf_conversions
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go_to_location
  CATKIN_DEPENDS moveit_planning_helper manipulation_msgs manipulation_utils roscpp rosdyn_core rosparam_utilities tf_conversions
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server src/go_to_location_server.cpp)
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
)

add_executable(       ${PROJECT_NAME}_client src/go_to_location_client.cpp)
add_dependencies(     ${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_client
  ${catkin_LIBRARIES}
)

add_executable(       ${PROJECT_NAME}_loader src/go_to_location_loader.cpp)
add_dependencies(     ${PROJECT_NAME}_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_loader
  ${catkin_LIBRARIES}
)
