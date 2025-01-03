# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pedsim_msgs: 17 messages, 0 services")

set(MSG_I_FLAGS "-Ipedsim_msgs:/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pedsim_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" "geometry_msgs/Point:std_msgs/Header:pedsim_msgs/AgentForce:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/Twist:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" "geometry_msgs/Point:std_msgs/Header:pedsim_msgs/AgentForce:pedsim_msgs/AgentState:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/Twist:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" "std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:pedsim_msgs/AgentGroup:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" "std_msgs/Header:pedsim_msgs/LineObstacle:geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" "geometry_msgs/Point:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:geometry_msgs/Twist:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" "geometry_msgs/Point:std_msgs/Header:pedsim_msgs/TrackedPerson:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/TwistWithCovariance:geometry_msgs/Twist:geometry_msgs/PoseWithCovariance"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/PoseWithCovariance"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:pedsim_msgs/TrackedGroup:geometry_msgs/PoseWithCovariance"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" ""
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" "std_msgs/Header:pedsim_msgs/SocialRelation"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" ""
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" "std_msgs/Header:pedsim_msgs/SocialActivity"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" NAME_WE)
add_custom_target(_pedsim_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pedsim_msgs" "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" "std_msgs/Header:pedsim_msgs/Waypoint:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_cpp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(pedsim_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pedsim_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pedsim_msgs_generate_messages pedsim_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_cpp _pedsim_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pedsim_msgs_gencpp)
add_dependencies(pedsim_msgs_gencpp pedsim_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pedsim_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_eus(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(pedsim_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pedsim_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pedsim_msgs_generate_messages pedsim_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_eus _pedsim_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pedsim_msgs_geneus)
add_dependencies(pedsim_msgs_geneus pedsim_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pedsim_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_lisp(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(pedsim_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pedsim_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pedsim_msgs_generate_messages pedsim_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_lisp _pedsim_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pedsim_msgs_genlisp)
add_dependencies(pedsim_msgs_genlisp pedsim_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pedsim_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_nodejs(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pedsim_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pedsim_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pedsim_msgs_generate_messages pedsim_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_nodejs _pedsim_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pedsim_msgs_gennodejs)
add_dependencies(pedsim_msgs_gennodejs pedsim_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pedsim_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)
_generate_msg_py(pedsim_msgs
  "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(pedsim_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pedsim_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pedsim_msgs_generate_messages pedsim_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentState.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentStates.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/AgentForce.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacle.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/LineObstacles.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPerson.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedPersons.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroup.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/TrackedGroups.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelation.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialRelations.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivity.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/SocialActivities.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/xjh/ros_motion_planning/src/plugins/gazebo_plugins/pedestrian_visualizer_plugin/pedsim_msgs/msg/Waypoints.msg" NAME_WE)
add_dependencies(pedsim_msgs_generate_messages_py _pedsim_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pedsim_msgs_genpy)
add_dependencies(pedsim_msgs_genpy pedsim_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pedsim_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pedsim_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pedsim_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(pedsim_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(pedsim_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(pedsim_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pedsim_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pedsim_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(pedsim_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(pedsim_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(pedsim_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pedsim_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pedsim_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(pedsim_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(pedsim_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(pedsim_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pedsim_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pedsim_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(pedsim_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(pedsim_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(pedsim_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pedsim_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pedsim_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(pedsim_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(pedsim_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(pedsim_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
