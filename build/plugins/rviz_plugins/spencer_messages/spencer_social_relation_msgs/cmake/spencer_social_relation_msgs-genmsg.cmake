# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "spencer_social_relation_msgs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ispencer_social_relation_msgs:/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(spencer_social_relation_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" NAME_WE)
add_custom_target(_spencer_social_relation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_social_relation_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" ""
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" NAME_WE)
add_custom_target(_spencer_social_relation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_social_relation_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" "std_msgs/Header:spencer_social_relation_msgs/SocialRelation"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" NAME_WE)
add_custom_target(_spencer_social_relation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_social_relation_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" ""
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" NAME_WE)
add_custom_target(_spencer_social_relation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_social_relation_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" "std_msgs/Header:spencer_social_relation_msgs/SocialActivity"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_cpp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_cpp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_cpp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(spencer_social_relation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(spencer_social_relation_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(spencer_social_relation_msgs_generate_messages spencer_social_relation_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_cpp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_cpp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_cpp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_cpp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_social_relation_msgs_gencpp)
add_dependencies(spencer_social_relation_msgs_gencpp spencer_social_relation_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_social_relation_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_eus(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_eus(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_eus(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(spencer_social_relation_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(spencer_social_relation_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(spencer_social_relation_msgs_generate_messages spencer_social_relation_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_eus _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_eus _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_eus _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_eus _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_social_relation_msgs_geneus)
add_dependencies(spencer_social_relation_msgs_geneus spencer_social_relation_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_social_relation_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_lisp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_lisp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_lisp(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(spencer_social_relation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(spencer_social_relation_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(spencer_social_relation_msgs_generate_messages spencer_social_relation_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_lisp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_lisp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_lisp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_lisp _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_social_relation_msgs_genlisp)
add_dependencies(spencer_social_relation_msgs_genlisp spencer_social_relation_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_social_relation_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_nodejs(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_nodejs(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_nodejs(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(spencer_social_relation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(spencer_social_relation_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(spencer_social_relation_msgs_generate_messages spencer_social_relation_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_social_relation_msgs_gennodejs)
add_dependencies(spencer_social_relation_msgs_gennodejs spencer_social_relation_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_social_relation_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_py(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_py(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs
)
_generate_msg_py(spencer_social_relation_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(spencer_social_relation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(spencer_social_relation_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(spencer_social_relation_msgs_generate_messages spencer_social_relation_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_py _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_py _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_py _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_social_relation_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(spencer_social_relation_msgs_generate_messages_py _spencer_social_relation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_social_relation_msgs_genpy)
add_dependencies(spencer_social_relation_msgs_genpy spencer_social_relation_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_social_relation_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_social_relation_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_social_relation_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(spencer_social_relation_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(spencer_social_relation_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(spencer_social_relation_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(spencer_social_relation_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_social_relation_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(spencer_social_relation_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_social_relation_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(spencer_social_relation_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_social_relation_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(spencer_social_relation_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(spencer_social_relation_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(spencer_social_relation_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(spencer_social_relation_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
