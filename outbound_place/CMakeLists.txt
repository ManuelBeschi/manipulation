cmake_minimum_required(VERSION 2.8.3)
project(outbound_place)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  moveit_planning_helper
  roscpp
  rosdyn_core
  rosparam_utilities
  std_srvs
  object_loader_msgs
  tf_conversions
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES outbound_place
  CATKIN_DEPENDS manipulation_msgs moveit_planning_helper roscpp rosdyn_core rosparam_utilities std_srvs object_loader_msgs tf_conversions eigen_conversions
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
  src/${PROJECT_NAME}/outbound_pallet.cpp
  src/${PROJECT_NAME}/outbound_mosaic.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(pallet src/pallet.cpp)
add_dependencies(pallet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pallet
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(mosaic src/mosaic.cpp)
add_dependencies(mosaic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mosaic
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
