cmake_minimum_required(VERSION 3.10.0)
project(outbound_place)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  moveit_planning_helper
  manipulation_msgs
  manipulation_utils
  roscpp
  rosdyn_core
  std_srvs
  rosparam_utilities
  object_loader_msgs
  tf_conversions
  #eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES outbound_place
  CATKIN_DEPENDS  moveit_planning_helper
                  manipulation_msgs 
                  manipulation_utils 
                  roscpp 
                  rosdyn_core 
                  rosparam_utilities 
                  std_srvs 
                  object_loader_msgs 
                  tf_conversions 
                  #eigen_conversions
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server src/outbound_place_server.cpp)
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_loader src/outbound_place_loader.cpp)
add_dependencies(${PROJECT_NAME}_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_loader
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_client src/outbound_place_client.cpp)
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_client
  ${catkin_LIBRARIES}
)
