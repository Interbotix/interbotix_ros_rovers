#!/usr/bin/env bash

# USAGE: ./xslocobot_amd64_update.sh
#
# Updates the Interbotix X-Series LoCoBot packages and their dependencies.

set -e

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

PROMPT="> "

ALL_VALID_DISTROS=('melodic' 'noetic' 'galactic' 'humble' 'rolling')
ROS1_DISTROS=('melodic' 'noetic')
ROS2_DISTROS=('galactic' 'humble' 'rolling')

# https://stackoverflow.com/a/8574392/16179107
function contains_element () {
  # check if an element is in an array
  local e match="$1"
  shift
  for e; do [[ "$e" == "$match" ]] && return 0; done
  return 1
}

function failed() {
  # Log error and quit with a failed exit code
  echo -e "${ERR}[ERROR] $@${RRE}"
  echo -e "${ERR}[ERROR] Interbotix Installation Failed!${RRE}"
  exit 1
}

function validate_distro() {
    # check if chosen distro is valid and set ROS major version
    if contains_element $ROS_DISTRO "${ALL_VALID_DISTROS[@]}"; then
        ROS_DISTRO=$ROS_DISTRO
        return 0
    else
        failed "'$ROS_DISTRO' is not a valid ROS Distribution. Choose one of: "${ALL_VALID_DISTROS[@]}""
    fi
}

function validate_install_path() {
  # check if the interbotix installation path is valid
    if [ -d "$INTERBOTIX_WS" ]; then
        if [ -d ""$INTERBOTIX_WS"/src/interbotix_ros_core" ] && [ -d ""$INTERBOTIX_WS"/src/interbotix_ros_core" ] && [ -d ""$INTERBOTIX_WS"/src/interbotix_ros_core" ]; then
            return 0
        else
            failed "'$INTERBOTIX_WS/src' does not contain the necessary repositories."
        fi
    else
        failed "'$INTERBOTIX_WS' is not a valid path."
    fi
}

if [[ -z "$INTERBOTIX_WS" ]]; then
    echo -e "Enter the path to your interbotix installation (i.e. /home/interbotix/interbotix_ws):\n${PROMPT}${NORM}${OFF}\c"
    read -r INTERBOTIX_WS
    echo -e "export INTERBOTIX_WS=$INTERBOTIX_WS" >> ~/.bashrc
fi

validate_install_path

if [[ -z "$ROS_DISTRO" ]]; then
    echo -e "Enter the name of your ROS distribution:\n${PROMPT}${NORM}${OFF}\c"
    read -r ROS_DISTRO
fi

validate_distro

if contains_element $ROS_DISTRO "${ROS1_DISTROS[@]}"; then
    BUILD_CMD='catkin_make'
elif contains_element $ROS_DISTRO "${ROS2_DISTROS[@]}"; then
    BUILD_CMD='colcon build'
fi

echo -e "${BLU}${BOLD}UPDATE SUMMARY:"
echo -e "\tInstallation path:          ${INTERBOTIX_WS}"
echo -e "\tROS Distribution:           ${ROS_DISTRO}"
echo -e "\tWorkspace build cmd:        ${BUILD_CMD}"
echo -e "\tAccepting will run the following commands:"
echo -e "${NORM}
\t\tcd $INTERBOTIX_WS
\t\tcd src/interbotix_ros_core && git pull && cd -
\t\tcd src/interbotix_ros_rovers && git pull && cd -
\t\tcd src/interbotix_ros_toolboxes && git pull && cd -
\t\tsudo apt-get update
\t\tcd $INTERBOTIX_WS && rosdep install --from-paths src --ignore-src -r -y --rosdistro=$ROS_DISTRO
\t\tcd $INTERBOTIX_WS && $BUILD_CMD"
echo -e "\n${BLU}${BOLD}Is this correct?\n${PROMPT}${NORM}${OFF}\c"
read -r resp

if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
    :
else
    exit 0
fi

cd $INTERBOTIX_WS
cd src/interbotix_ros_core && git pull && cd -
cd src/interbotix_ros_rovers && git pull && cd -
cd src/interbotix_ros_toolboxes && git pull && cd -

sudo apt-get update
cd $INTERBOTIX_WS && rosdep install --from-paths src --ignore-src -r -y --rosdistro=$ROS_DISTRO

cd $INTERBOTIX_WS && $BUILD_CMD
