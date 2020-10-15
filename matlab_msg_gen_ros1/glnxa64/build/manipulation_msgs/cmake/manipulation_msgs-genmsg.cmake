# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "manipulation_msgs: 4 messages, 6 services")

set(MSG_I_FLAGS "-Imanipulation_msgs:/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg;-Istd_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg;-Igeometry_msgs:/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(manipulation_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/ListOfObjects.srv" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/ListOfObjects.srv" ""
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddBox.srv" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddBox.srv" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose:manipulation_msgs/Grasp"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/PickBox.msg" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/PickBox.msg" "geometry_msgs/Point:manipulation_msgs/Grasp:geometry_msgs/Quaternion:geometry_msgs/Pose:manipulation_msgs/Object"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/RemoveLocations.srv" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/RemoveLocations.srv" ""
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddRemoveObjects.srv" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddRemoveObjects.srv" "geometry_msgs/Point:manipulation_msgs/Grasp:geometry_msgs/Quaternion:geometry_msgs/Pose:manipulation_msgs/Object"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddObjects.srv" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddObjects.srv" "geometry_msgs/Point:manipulation_msgs/Grasp:geometry_msgs/Quaternion:geometry_msgs/Pose:manipulation_msgs/Object"
)

get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddLocations.srv" NAME_WE)
add_custom_target(_manipulation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "manipulation_msgs" "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddLocations.srv" "geometry_msgs/Point:manipulation_msgs/Location:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/PickBox.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_msg_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_msg_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_msg_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)

### Generating Services
_generate_srv_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/ListOfObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddBox.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddLocations.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddRemoveObjects.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddObjects.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_cpp(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/RemoveLocations.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
)

### Generating Module File
_generate_module_cpp(manipulation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(manipulation_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(manipulation_msgs_generate_messages manipulation_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/ListOfObjects.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddBox.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/PickBox.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/RemoveLocations.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddRemoveObjects.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddObjects.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddLocations.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_cpp _manipulation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manipulation_msgs_gencpp)
add_dependencies(manipulation_msgs_gencpp manipulation_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manipulation_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/PickBox.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_msg_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_msg_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_msg_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)

### Generating Services
_generate_srv_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/ListOfObjects.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddBox.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddLocations.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddRemoveObjects.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddObjects.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Quaternion.msg;/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)
_generate_srv_py(manipulation_msgs
  "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/RemoveLocations.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
)

### Generating Module File
_generate_module_py(manipulation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(manipulation_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(manipulation_msgs_generate_messages manipulation_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/ListOfObjects.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddBox.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Location.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Grasp.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/Object.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg/PickBox.msg" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/RemoveLocations.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddRemoveObjects.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddObjects.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/srv/AddLocations.srv" NAME_WE)
add_dependencies(manipulation_msgs_generate_messages_py _manipulation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(manipulation_msgs_genpy)
add_dependencies(manipulation_msgs_genpy manipulation_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS manipulation_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/manipulation_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(manipulation_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(manipulation_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs)
  install(CODE "execute_process(COMMAND \"/home/jacobi/.matlab/R2020b/ros1/glnxa64/venv/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/manipulation_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(manipulation_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(manipulation_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
