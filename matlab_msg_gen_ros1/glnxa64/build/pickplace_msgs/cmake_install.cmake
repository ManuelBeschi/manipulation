# Install script for directory: /home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pickplace_msgs/msg" TYPE FILE FILES
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg"
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pickplace_msgs/srv" TYPE FILE FILES
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv"
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pickplace_msgs/cmake" TYPE FILE FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/build/pickplace_msgs/catkin_generated/installspace/pickplace_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/devel/include/pickplace_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/jacobi/.matlab/R2020b/ros1/glnxa64/venv/bin/python2" -m compileall "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/devel/lib/python2.7/dist-packages/pickplace_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/devel/lib/python2.7/dist-packages/pickplace_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/build/pickplace_msgs/catkin_generated/installspace/pickplace_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pickplace_msgs/cmake" TYPE FILE FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/build/pickplace_msgs/catkin_generated/installspace/pickplace_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pickplace_msgs/cmake" TYPE FILE FILES
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/build/pickplace_msgs/catkin_generated/installspace/pickplace_msgsConfig.cmake"
    "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/build/pickplace_msgs/catkin_generated/installspace/pickplace_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pickplace_msgs" TYPE FILE FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/devel/lib/libpickplace_msgs_matlab.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpickplace_msgs_matlab.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpickplace_msgs_matlab.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpickplace_msgs_matlab.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()

