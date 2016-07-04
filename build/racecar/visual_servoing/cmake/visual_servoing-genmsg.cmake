# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "visual_servoing: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ivisual_servoing:/home/racecar/team-ws/src/racecar/visual_servoing/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(visual_servoing_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg" NAME_WE)
add_custom_target(_visual_servoing_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "visual_servoing" "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg" "geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg" NAME_WE)
add_custom_target(_visual_servoing_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "visual_servoing" "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg" "geometry_msgs/Point:geometry_msgs/Vector3:std_msgs/Header:visual_servoing/CameraObject"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(visual_servoing
  "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/visual_servoing
)
_generate_msg_cpp(visual_servoing
  "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/visual_servoing
)

### Generating Services

### Generating Module File
_generate_module_cpp(visual_servoing
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/visual_servoing
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(visual_servoing_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(visual_servoing_generate_messages visual_servoing_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg" NAME_WE)
add_dependencies(visual_servoing_generate_messages_cpp _visual_servoing_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg" NAME_WE)
add_dependencies(visual_servoing_generate_messages_cpp _visual_servoing_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(visual_servoing_gencpp)
add_dependencies(visual_servoing_gencpp visual_servoing_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS visual_servoing_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(visual_servoing
  "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/visual_servoing
)
_generate_msg_lisp(visual_servoing
  "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/visual_servoing
)

### Generating Services

### Generating Module File
_generate_module_lisp(visual_servoing
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/visual_servoing
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(visual_servoing_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(visual_servoing_generate_messages visual_servoing_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg" NAME_WE)
add_dependencies(visual_servoing_generate_messages_lisp _visual_servoing_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg" NAME_WE)
add_dependencies(visual_servoing_generate_messages_lisp _visual_servoing_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(visual_servoing_genlisp)
add_dependencies(visual_servoing_genlisp visual_servoing_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS visual_servoing_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(visual_servoing
  "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing
)
_generate_msg_py(visual_servoing
  "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing
)

### Generating Services

### Generating Module File
_generate_module_py(visual_servoing
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(visual_servoing_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(visual_servoing_generate_messages visual_servoing_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObject.msg" NAME_WE)
add_dependencies(visual_servoing_generate_messages_py _visual_servoing_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/racecar/team-ws/src/racecar/visual_servoing/msg/CameraObjectsStamped.msg" NAME_WE)
add_dependencies(visual_servoing_generate_messages_py _visual_servoing_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(visual_servoing_genpy)
add_dependencies(visual_servoing_genpy visual_servoing_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS visual_servoing_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/visual_servoing)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/visual_servoing
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(visual_servoing_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(visual_servoing_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/visual_servoing)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/visual_servoing
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(visual_servoing_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(visual_servoing_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/visual_servoing/.+/__init__.pyc?$"
  )
endif()
add_dependencies(visual_servoing_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(visual_servoing_generate_messages_py geometry_msgs_generate_messages_py)
