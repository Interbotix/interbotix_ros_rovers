#!/usr/bin/env bash

ROBOT_NAME=$1

. /opt/ros/galactic/setup.bash

sleep 10

ros2 param set /$ROBOT_NAME/motion_control reflexes_enabled False
