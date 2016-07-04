# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vesc_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ivesc_msgs:/home/racecar/team-ws/src/vesc/vesc_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vesc_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg" NAME_WE)
add_custom_target(_vesc_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vesc_msgs" "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg" "vesc_msgs/VescState:std_msgs/Header"
)

get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg" NAME_WE)
add_custom_target(_vesc_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vesc_msgs" "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vesc_msgs
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vesc_msgs
)
_generate_msg_cpp(vesc_msgs
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vesc_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(vesc_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vesc_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vesc_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vesc_msgs_generate_messages vesc_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg" NAME_WE)
add_dependencies(vesc_msgs_generate_messages_cpp _vesc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg" NAME_WE)
add_dependencies(vesc_msgs_generate_messages_cpp _vesc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vesc_msgs_gencpp)
add_dependencies(vesc_msgs_gencpp vesc_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vesc_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vesc_msgs
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vesc_msgs
)
_generate_msg_lisp(vesc_msgs
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vesc_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(vesc_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vesc_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vesc_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vesc_msgs_generate_messages vesc_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg" NAME_WE)
add_dependencies(vesc_msgs_generate_messages_lisp _vesc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg" NAME_WE)
add_dependencies(vesc_msgs_generate_messages_lisp _vesc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vesc_msgs_genlisp)
add_dependencies(vesc_msgs_genlisp vesc_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vesc_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vesc_msgs
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vesc_msgs
)
_generate_msg_py(vesc_msgs
  "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vesc_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(vesc_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vesc_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vesc_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vesc_msgs_generate_messages vesc_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg" NAME_WE)
add_dependencies(vesc_msgs_generate_messages_py _vesc_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/racecar/team-ws/src/vesc/vesc_msgs/msg/VescState.msg" NAME_WE)
add_dependencies(vesc_msgs_generate_messages_py _vesc_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vesc_msgs_genpy)
add_dependencies(vesc_msgs_genpy vesc_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vesc_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vesc_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vesc_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(vesc_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vesc_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vesc_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(vesc_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vesc_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vesc_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vesc_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(vesc_msgs_generate_messages_py std_msgs_generate_messages_py)
