# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "base_local_planner;dynamic_reconfigure;nav_msgs;pluginlib;sensor_msgs;roscpp;tf2;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldwa_controller".split(';') if "-ldwa_controller" != "" else []
PROJECT_NAME = "dwa_controller"
PROJECT_SPACE_DIR = "/home/xjh/ros_motion_planning/install"
PROJECT_VERSION = "1.17.1"
