#!/usr/bin/env bash

RED="\033[0;31m"
OFF="\033[0m"

ROBOT_NAME=$1

. /opt/ros/galactic/setup.bash

sleep 10

if ros2 node list | grep -q motion_control; then
  :
else
  >&2 echo -e "${RED}[ERROR] The Create3 base could not be discovered. Is it turned on? Quitting...${OFF}"
  exit 1
fi

if ros2 param set /$ROBOT_NAME/motion_control reflexes_enabled False; then
  >&2 echo -e "[INFO] Succeeded to set default parameters on the Create3 base."
else
  >&2 echo -e "${RED}[ERROR] Failed to set default parameters on the Create3 base.${OFF}"
  exit 1
fi
