# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gazebo_sfm_plugin: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gazebo_sfm_plugin_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" NAME_WE)
add_custom_target(_gazebo_sfm_plugin_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gazebo_sfm_plugin" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(gazebo_sfm_plugin
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_sfm_plugin
)

### Generating Module File
_generate_module_cpp(gazebo_sfm_plugin
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_sfm_plugin
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gazebo_sfm_plugin_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gazebo_sfm_plugin_generate_messages gazebo_sfm_plugin_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" NAME_WE)
add_dependencies(gazebo_sfm_plugin_generate_messages_cpp _gazebo_sfm_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_sfm_plugin_gencpp)
add_dependencies(gazebo_sfm_plugin_gencpp gazebo_sfm_plugin_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_sfm_plugin_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(gazebo_sfm_plugin
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_sfm_plugin
)

### Generating Module File
_generate_module_eus(gazebo_sfm_plugin
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_sfm_plugin
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gazebo_sfm_plugin_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gazebo_sfm_plugin_generate_messages gazebo_sfm_plugin_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" NAME_WE)
add_dependencies(gazebo_sfm_plugin_generate_messages_eus _gazebo_sfm_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_sfm_plugin_geneus)
add_dependencies(gazebo_sfm_plugin_geneus gazebo_sfm_plugin_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_sfm_plugin_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(gazebo_sfm_plugin
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_sfm_plugin
)

### Generating Module File
_generate_module_lisp(gazebo_sfm_plugin
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_sfm_plugin
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gazebo_sfm_plugin_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gazebo_sfm_plugin_generate_messages gazebo_sfm_plugin_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" NAME_WE)
add_dependencies(gazebo_sfm_plugin_generate_messages_lisp _gazebo_sfm_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_sfm_plugin_genlisp)
add_dependencies(gazebo_sfm_plugin_genlisp gazebo_sfm_plugin_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_sfm_plugin_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(gazebo_sfm_plugin
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_sfm_plugin
)

### Generating Module File
_generate_module_nodejs(gazebo_sfm_plugin
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_sfm_plugin
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gazebo_sfm_plugin_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gazebo_sfm_plugin_generate_messages gazebo_sfm_plugin_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" NAME_WE)
add_dependencies(gazebo_sfm_plugin_generate_messages_nodejs _gazebo_sfm_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_sfm_plugin_gennodejs)
add_dependencies(gazebo_sfm_plugin_gennodejs gazebo_sfm_plugin_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_sfm_plugin_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(gazebo_sfm_plugin
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_sfm_plugin
)

### Generating Module File
_generate_module_py(gazebo_sfm_plugin
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_sfm_plugin
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gazebo_sfm_plugin_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gazebo_sfm_plugin_generate_messages gazebo_sfm_plugin_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_sfm_plugin/srv/ped_state.srv" NAME_WE)
add_dependencies(gazebo_sfm_plugin_generate_messages_py _gazebo_sfm_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gazebo_sfm_plugin_genpy)
add_dependencies(gazebo_sfm_plugin_genpy gazebo_sfm_plugin_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gazebo_sfm_plugin_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_sfm_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gazebo_sfm_plugin
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_sfm_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gazebo_sfm_plugin
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_sfm_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gazebo_sfm_plugin
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_sfm_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gazebo_sfm_plugin
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_sfm_plugin)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_sfm_plugin\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gazebo_sfm_plugin
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
