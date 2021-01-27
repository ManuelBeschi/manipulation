cmake_minimum_required(VERSION 3.0.2)
project(manipulation_utils)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  moveit_planning_helper
  object_loader_msgs
  roscpp
  roscpp
  rosdyn_core
  rosparam_utilities
  std_srvs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES manipulation_utils
  CATKIN_DEPENDS manipulation_msgs moveit_planning_helper object_loader_msgs roscpp roscpp rosdyn_core rosparam_utilities std_srvs
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	    src/${PROJECT_NAME}/location.cpp
      src/${PROJECT_NAME}/skill_base.cpp
      src/${PROJECT_NAME}/manipulation_utils.cpp 
      src/${PROJECT_NAME}/pick_objects.cpp 
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(        test_recipe src/test_recipe.cpp)
#add_dependencies(      test_recipe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries( test_recipe
#   ${catkin_LIBRARIES}
#)
