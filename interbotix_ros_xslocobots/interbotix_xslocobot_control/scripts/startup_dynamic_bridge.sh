#!/usr/bin/env bash

# Use this script to start a bridge between ROS 1 and ROS 2 topics and services if using the iRobot
# Create3 base.

. $BRIDGE_WS/install/local_setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
