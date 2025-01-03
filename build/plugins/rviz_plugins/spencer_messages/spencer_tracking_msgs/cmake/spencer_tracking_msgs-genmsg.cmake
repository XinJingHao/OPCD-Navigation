# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "spencer_tracking_msgs: 13 messages, 1 services")

set(MSG_I_FLAGS "-Ispencer_tracking_msgs:/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(spencer_tracking_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" "geometry_msgs/PoseWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" "geometry_msgs/Pose:spencer_tracking_msgs/DetectedPerson:geometry_msgs/PoseWithCovariance:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" "geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:spencer_tracking_msgs/DetectedPerson:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" "spencer_tracking_msgs/CompositeDetectedPerson:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:spencer_tracking_msgs/DetectedPerson:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" "geometry_msgs/Twist:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" "spencer_tracking_msgs/TrackedPerson:geometry_msgs/Twist:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" ""
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" "spencer_tracking_msgs/TrackedPerson2d:std_msgs/Header"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" "geometry_msgs/PoseWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" "geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:spencer_tracking_msgs/TrackedGroup:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" ""
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" "spencer_tracking_msgs/ImmDebugInfo:std_msgs/Header"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" NAME_WE)
add_custom_target(_spencer_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "spencer_tracking_msgs" "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" "geometry_msgs/Twist:geometry_msgs/Pose:spencer_tracking_msgs/PersonTrajectory:geometry_msgs/PoseWithCovariance:spencer_tracking_msgs/PersonTrajectoryEntry:geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Services
_generate_srv_cpp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectoryEntry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Module File
_generate_module_cpp(spencer_tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(spencer_tracking_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(spencer_tracking_msgs_generate_messages spencer_tracking_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_cpp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_tracking_msgs_gencpp)
add_dependencies(spencer_tracking_msgs_gencpp spencer_tracking_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_tracking_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Services
_generate_srv_eus(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectoryEntry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Module File
_generate_module_eus(spencer_tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(spencer_tracking_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(spencer_tracking_msgs_generate_messages spencer_tracking_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_eus _spencer_tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_tracking_msgs_geneus)
add_dependencies(spencer_tracking_msgs_geneus spencer_tracking_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_tracking_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Services
_generate_srv_lisp(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectoryEntry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Module File
_generate_module_lisp(spencer_tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(spencer_tracking_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(spencer_tracking_msgs_generate_messages spencer_tracking_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_lisp _spencer_tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_tracking_msgs_genlisp)
add_dependencies(spencer_tracking_msgs_genlisp spencer_tracking_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_tracking_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Services
_generate_srv_nodejs(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectoryEntry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Module File
_generate_module_nodejs(spencer_tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(spencer_tracking_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(spencer_tracking_msgs_generate_messages spencer_tracking_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_nodejs _spencer_tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_tracking_msgs_gennodejs)
add_dependencies(spencer_tracking_msgs_gennodejs spencer_tracking_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_tracking_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg"
  "${MSG_I_FLAGS}"
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)
_generate_msg_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Services
_generate_srv_py(spencer_tracking_msgs
  "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectory.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/PersonTrajectoryEntry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
)

### Generating Module File
_generate_module_py(spencer_tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(spencer_tracking_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(spencer_tracking_msgs_generate_messages spencer_tracking_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/DetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/CompositeDetectedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPerson2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedPersons2d.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfo.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/ImmDebugInfos.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/msg/TrackingTimingMetrics.msg" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_tracking_msgs/srv/GetPersonTrajectories.srv" NAME_WE)
add_dependencies(spencer_tracking_msgs_generate_messages_py _spencer_tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(spencer_tracking_msgs_genpy)
add_dependencies(spencer_tracking_msgs_genpy spencer_tracking_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS spencer_tracking_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/spencer_tracking_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(spencer_tracking_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(spencer_tracking_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/spencer_tracking_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(spencer_tracking_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(spencer_tracking_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/spencer_tracking_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(spencer_tracking_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(spencer_tracking_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/spencer_tracking_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(spencer_tracking_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(spencer_tracking_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/spencer_tracking_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(spencer_tracking_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(spencer_tracking_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
