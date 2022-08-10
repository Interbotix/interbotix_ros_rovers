#!/usr/bin/env bash

RED="\033[0;31m"
OFF="\033[0m"

ROBOT_NAME=$1

. /opt/ros/galactic/setup.bash

if ros2 node list | grep -q motion_control; then
  :
else
  >&2 echo -e "${RED}[ERROR] The Create3 base could not be discovered. Is it turned on? Quitting...${OFF}"
  exit 1
fi

>&2 echo "[INFO] Disabling Create3 reflexes. This might take a few seconds..."
ros2 param set /$ROBOT_NAME/motion_control reflexes_enabled False
