# Install script for directory: /home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/xjh/ros_motion_planning/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spencer_human_attribute_msgs/msg" TYPE FILE FILES
    "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/msg/CategoricalAttribute.msg"
    "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/msg/ScalarAttribute.msg"
    "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/msg/HumanAttributes.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spencer_human_attribute_msgs/cmake" TYPE FILE FILES "/home/xjh/ros_motion_planning/build/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/catkin_generated/installspace/spencer_human_attribute_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/xjh/ros_motion_planning/devel/include/spencer_human_attribute_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/xjh/ros_motion_planning/devel/share/roseus/ros/spencer_human_attribute_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/xjh/ros_motion_planning/devel/share/common-lisp/ros/spencer_human_attribute_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/xjh/ros_motion_planning/devel/share/gennodejs/ros/spencer_human_attribute_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/xjh/ros_motion_planning/devel/lib/python3/dist-packages/spencer_human_attribute_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/xjh/ros_motion_planning/devel/lib/python3/dist-packages/spencer_human_attribute_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/xjh/ros_motion_planning/build/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/catkin_generated/installspace/spencer_human_attribute_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spencer_human_attribute_msgs/cmake" TYPE FILE FILES "/home/xjh/ros_motion_planning/build/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/catkin_generated/installspace/spencer_human_attribute_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spencer_human_attribute_msgs/cmake" TYPE FILE FILES
    "/home/xjh/ros_motion_planning/build/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/catkin_generated/installspace/spencer_human_attribute_msgsConfig.cmake"
    "/home/xjh/ros_motion_planning/build/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/catkin_generated/installspace/spencer_human_attribute_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/spencer_human_attribute_msgs" TYPE FILE FILES "/home/xjh/ros_motion_planning/src/plugins/rviz_plugins/spencer_messages/spencer_human_attribute_msgs/package.xml")
endif()

