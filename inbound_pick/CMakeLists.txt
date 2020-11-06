cmake_minimum_required(VERSION 2.8.3)
project(inbound_pick)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  moveit_planning_helper
  manipulation_msgs
  roscpp
  rosdyn_core
  std_srvs
  rosparam_utilities
  object_loader_msgs
  tf_conversions
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inbound_pick
  CATKIN_DEPENDS moveit_planning_helper manipulation_msgs roscpp rosdyn_core std_srvs rosparam_utilities object_loader_msgs tf_conversions
  DEPENDS
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/inbound_pick.cpp
   src/${PROJECT_NAME}/pick_objects.cpp
   src/${PROJECT_NAME}/inbound_client.cpp
)
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_server src/inbound_pick_server.cpp)
add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       ${PROJECT_NAME}_client src/inbound_pick_client.cpp)
add_dependencies(     ${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_client
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(       ${PROJECT_NAME}_loader src/inbound_pick_loader.cpp)
add_dependencies(     ${PROJECT_NAME}_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_loader
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

