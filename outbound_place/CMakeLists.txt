cmake_minimum_required(VERSION 2.8.3)
project(outbound_place)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  moveit_planning_helper
  roscpp
  rosdyn_core
  rosparam_utilities
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES outbound_place
  CATKIN_DEPENDS manipulation_msgs moveit_planning_helper roscpp rosdyn_core rosparam_utilities std_srvs
  DEPENDS
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

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/outbound_place.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

