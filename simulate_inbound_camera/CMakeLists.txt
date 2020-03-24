cmake_minimum_required(VERSION 2.8.3)
project(simulate_inbound_camera)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  object_loader_msgs
  roscpp
)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS manipulation_msgs object_loader_msgs roscpp
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(simulate_cam src/simulate_inbound_camera_node.cpp)
 add_dependencies(simulate_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(simulate_cam
   ${catkin_LIBRARIES}
 )

 install(TARGETS simulate_cam
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
