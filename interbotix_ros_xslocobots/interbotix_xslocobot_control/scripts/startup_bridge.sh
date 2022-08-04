#!/usr/bin/env bash

# Use this script to start a bridge between ROS 1 and ROS 2 topics and services if using the iRobot
# Create3 base.

. /opt/ros/$ROS_DISTRO/setup.bash &&                      \
  . /opt/ros/galactic/setup.bash &&                       \
  . $BRIDGE_MSGS_ROS1_WS/install_isolated/setup.bash &&   \
  . $BRIDGE_MSGS_ROS2_WS/install/setup.bash &&            \
  . $BRIDGE_WS/install/local_setup.bash &&                \
  ros2 run ros1_bridge parameter_bridge
