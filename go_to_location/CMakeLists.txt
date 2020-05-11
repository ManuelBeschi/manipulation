cmake_minimum_required(VERSION 2.8.3)
project(go_to_location)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  moveit_planning_helper
  roscpp
  roscpp
  rosdyn_core
  rosparam_utilities
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go_to_location
  CATKIN_DEPENDS manipulation_msgs moveit_planning_helper roscpp roscpp rosdyn_core rosparam_utilities
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(node src/goto_node.cpp)
add_dependencies(node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

