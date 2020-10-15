# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pickplace_msgs: 3 messages, 2 services")

set(MSG_I_FLAGS "-Ipickplace_msgs:/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg;-Istd_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg;-Igeometry_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pickplace_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg" NAME_WE)
add_custom_target(_pickplace_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pickplace_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg" "pickplace_msgs/Object:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose:pickplace_msgs/Grasp"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg" NAME_WE)
add_custom_target(_pickplace_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pickplace_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg" NAME_WE)
add_custom_target(_pickplace_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pickplace_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg" "pickplace_msgs/Grasp:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv" NAME_WE)
add_custom_target(_pickplace_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pickplace_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv" "geometry_msgs/Point:geometry_msgs/Pose:pickplace_msgs/PickBox:pickplace_msgs/Grasp:pickplace_msgs/Object:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv" NAME_WE)
add_custom_target(_pickplace_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pickplace_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv" "pickplace_msgs/Object:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose:pickplace_msgs/Grasp"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg"
  "${MSG_I_FLAGS}"
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
)
_generate_msg_cpp(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
)
_generate_msg_cpp(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
)

### Generating Services
_generate_srv_cpp(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
)
_generate_srv_cpp(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv"
  "${MSG_I_FLAGS}"
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
)

### Generating Module File
_generate_module_cpp(pickplace_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pickplace_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pickplace_msgs_generate_messages pickplace_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_cpp _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_cpp _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_cpp _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_cpp _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_cpp _pickplace_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pickplace_msgs_gencpp)
add_dependencies(pickplace_msgs_gencpp pickplace_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pickplace_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg"
  "${MSG_I_FLAGS}"
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
)
_generate_msg_py(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
)
_generate_msg_py(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
)

### Generating Services
_generate_srv_py(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
)
_generate_srv_py(pickplace_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv"
  "${MSG_I_FLAGS}"
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
)

### Generating Module File
_generate_module_py(pickplace_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pickplace_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pickplace_msgs_generate_messages pickplace_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/PickBox.msg" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_py _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Grasp.msg" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_py _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg/Object.msg" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_py _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/ListOfObjects.srv" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_py _pickplace_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/srv/AddRemoveObjects.srv" NAME_WE)
add_dependencies(pickplace_msgs_generate_messages_py _pickplace_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pickplace_msgs_genpy)
add_dependencies(pickplace_msgs_genpy pickplace_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pickplace_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pickplace_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pickplace_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(pickplace_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs)
  install(CODE "execute_process(COMMAND \"/home/jacobi/.matlab/R2020b/ros1/glnxa64/venv/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pickplace_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pickplace_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(pickplace_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
