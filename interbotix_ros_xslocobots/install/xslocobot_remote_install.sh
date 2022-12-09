#!/usr/bin/env bash

# USAGE: ./xslocobot_remote_install.sh [-h][-d DISTRO][-p PATH][-b BASE_TYPE][-r HOSTNAME]
#
# Install the Interbotix X-Series LoCoBot packages and their dependencies.

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'
ORANGE='\033[;33m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

PROMPT="> "

ALL_VALID_DISTROS=('melodic' 'noetic' 'galactic')
ROS1_VALID_DISTROS=('melodic' 'noetic')
ROS2_VALID_DISTROS=('galactic')

BIONIC_VALID_DISTROS=('melodic')
FOCAL_VALID_DISTROS=('noetic' 'galactic')

VALID_BASE_TYPES=('kobuki' 'create3')

DISTRO_SET_FROM_CL=false
INSTALL_PATH=~/interbotix_ws
HOSTNAME=""

_usage="${BOLD}USAGE: ./xslocobot_remote_install.sh [-h][-d DISTRO][-p PATH][-b BASE_TYPE][-r HOSTNAME]${NORM}

Install the Interbotix X-Series LoCoBot packages and their dependencies.

Options:

  -h              Display this help message and quit

  -d DISTRO       Install the DISTRO ROS distro compatible with your Ubuntu version. See
                  'https://github.com/Interbotix/.github/blob/main/SECURITY.md' for the list of
                  supported distributions. If not given, installs the ROS 1 distro compatible with
                  your Ubuntu version, or the stable ROS 2 distro if using Ubuntu 22.04 or later.

  -b BASE_TYPE    Sets the base type for the robot, either 'kobuki' or 'create3'. If not specified,
                  the default of 'create3' will be used. Note that the Create 3 is not compatible with
                  Ubuntu 18.04 or ROS 1 Melodic.

  -p PATH         Sets the absolute install location for the Interbotix workspace. If not specified,
                  the Interbotix workspace directory will default to '~/interbotix_ws'.

  -r HOSTNAME     Sets the LoCoBot computer's hostname. If not specified, the script will
                  prompt for this later on.

Examples:

  ./xslocobot_remote_install.sh ${BOLD}-h${NORM}
    This will display this help message and quit.

  ./xslocobot_remote_install.sh
    This will install just the ROS 1 distro compatible with your Ubuntu version, or the stable ROS 2
    distro if using Ubuntu 22.04 or later. It will prompt you to ask if you want to install certain
    packages and dependencies.

  ./xslocobot_remote_install.sh ${BOLD}-b kobuki${NORM}
    This will install packages for the kobuki base depending on your ROS distro and configure your
    environment accordingly.

  ./xslocobot_remote_install.sh ${BOLD}-d noetic${NORM}
    This will install ROS 1 Noetic assuming that your Ubuntu version is compatible.

  ./xslocobot_remote_install.sh ${BOLD}-d galactic${NORM}
    Install ROS2 Galactic assuming that your Ubuntu version is compatible.

  ./xslocobot_remote_install.sh ${BOLD}-p ~/custom_ws${NORM}
    Installs the Interbotix packages under the '~/custom_ws' path.

  ./xslocobot_remote_install.sh ${BOLD}-r locobot${NORM}
    Sets the LoCoBot computer's hostname to 'locobot'."

function help() {
  # print usage
  cat << EOF
$_usage
EOF
}
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
  if contains_element $ROS_DISTRO_TO_INSTALL "${ALL_VALID_DISTROS[@]}"; then
    if contains_element $ROS_DISTRO_TO_INSTALL "${ROS1_VALID_DISTROS[@]}"; then
      # Supported ROS 1 distros
      ROS_VERSION_TO_INSTALL=1
    elif contains_element $ROS_DISTRO_TO_INSTALL "${ROS2_VALID_DISTROS[@]}"; then
      # Supported ROS2 distros
      ROS_VERSION_TO_INSTALL=2
    else
      # For cases where it passes the first check but somehow fails the second check
      failed "Something went wrong."
    fi
    ROS_DISTRO_TO_INSTALL=$ROS_DISTRO_TO_INSTALL
    echo -e "${GRN}${BOLD}Chosen Version: ROS${ROS_VERSION_TO_INSTALL} $ROS_DISTRO_TO_INSTALL${NORM}${OFF}"
    return 0
  else
    failed "'$ROS_DISTRO_TO_INSTALL' is not a valid ROS Distribution. Choose one of: "${ALL_VALID_DISTROS[@]}""
  fi
}

function validate_base_type() {
  if [ -z "$BASE_TYPE" ]; then
    failed "You must specify a base type using the '-b' flag followed by one of: "${VALID_BASE_TYPES[@]}""
  fi
  if contains_element $BASE_TYPE "${VALID_BASE_TYPES[@]}"; then
    :
  else
    failed "Base type '$BASE_TYPE' is not valid. Choose one of: "${VALID_BASE_TYPES[@]}""
  fi
}

function check_ubuntu_version() {
 # check if the chosen distribution is compatible with the Ubuntu version
  case $UBUNTU_VERSION in

    18.04 )
      if contains_element $ROS_DISTRO_TO_INSTALL "${BIONIC_VALID_DISTROS[@]}"; then
        PY_VERSION=2
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    20.04 )
      if contains_element $ROS_DISTRO_TO_INSTALL "${FOCAL_VALID_DISTROS[@]}"; then
        PY_VERSION=3
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    *)
      failed "Something went wrong."
      ;;

  esac
}

# parse command line arguments
while getopts 'hd:b:p:r:' OPTION;
do
  case "$OPTION" in
    h) help && exit 0;;
    d) ROS_DISTRO_TO_INSTALL="$OPTARG" && DISTRO_SET_FROM_CL=true && validate_distro;;
    b) BASE_TYPE="$OPTARG";;
    p) INSTALL_PATH="$OPTARG";;
    r) HOSTNAME="$OPTARG";;
    *) echo "Unknown argument $OPTION" && help && exit 0;;
  esac
done
shift "$(($OPTIND -1))"

validate_base_type

if ! command -v lsb_release &> /dev/null; then
  sudo apt update
  sudo apt-get install -yq lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

# set default ROS distro before reading clargs
if [ "$DISTRO_SET_FROM_CL" = false ]; then
  if [ $UBUNTU_VERSION == "18.04" ]; then
    ROS_DISTRO_TO_INSTALL="melodic"
  elif [ $UBUNTU_VERSION == "20.04" ]; then
    ROS_DISTRO_TO_INSTALL="noetic"
  else
    echo -e "${BOLD}${RED}Unsupported Ubuntu version: $UBUNTU_VERSION.${NORM}${OFF}"
    failed "Interbotix LoCoBot only works with Ubuntu 18.04 Bionic or 20.04 Focal on your hardware."
  fi
fi

check_ubuntu_version

if [ -z "$HOSTNAME" ]; then
  # prompt for hostname
  echo -e "${BLU}${BOLD}What is the hostname of the LoCoBot's computer (type 'hostname' in a terminal to find it)?\n$PROMPT${NORM}${OFF}\c"
  read -r HOSTNAME
fi

echo -e "${BLU}${BOLD}INSTALLATION SUMMARY:"
echo -e "\tROS Distribution:           ROS ${ROS_VERSION_TO_INSTALL} ${ROS_DISTRO_TO_INSTALL}"
echo -e "\tLoCoBot hostname:           ${HOSTNAME}"
echo -e "\tBase Type:                  ${BASE_TYPE}"
echo -e "\tInstallation path:          ${INSTALL_PATH}"
echo -e "\nIs this correct?\n${PROMPT}${NORM}${OFF}\c"
read -r resp

if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
  :
else
  help && exit 0
fi

echo -e "\n# Interbotix Configurations" >> ~/.bashrc

export INTERBOTIX_XSLOCOBOT_BASE_TYPE=${BASE_TYPE}

sudo apt-get install -yq                  \
  ros-$ROS_DISTRO_TO_INSTALL-rtabmap-ros  \
  ros-$ROS_DISTRO_TO_INSTALL-xacro

# Install Locobot packages
if [ ! -d "$INSTALL_PATH/src" ]; then
  echo -e "${GRN}Installing Simulation/Visualization ROS packages for the Interbotix Locobot...${OFF}"
  mkdir -p $INSTALL_PATH/src
  cd $INSTALL_PATH
  catkin_make
  cd src
  git clone https://github.com/Interbotix/interbotix_ros_rovers.git
  git clone https://github.com/Interbotix/create3_sim_ros1.git -b ros1
  cd interbotix_ros_rovers && git checkout $ROS_DISTRO_TO_INSTALL && cd ..
  echo "source $INSTALL_PATH/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix Locobot ROS packages already installed!"
fi
source $INSTALL_PATH/devel/setup.bash

if [ -z "$ROS_IP" ]; then
  echo -e "${GRN}Setting up Environment Variables...${OFF}"
  echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
  echo -e "export INTERBOTIX_XSLOCOBOT_BASE_TYPE=${BASE_TYPE}" >> ~/.bashrc
  echo "export ROS_MASTER_URI=http://$HOSTNAME.local:11311" >> ~/.bashrc
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi

echo "Remote Installation Complete! Close this terminal and open a new one to finish."
echo -e "NOTE: Remember to comment out the ${ORANGE}source $INSTALL_PATH/devel/setup.bash${OFF} and ${ORANGE}export ROS_MASTER_URI=http://$HOSTNAME.local:11311${OFF} lines from the ~/.bashrc file when done using the Locobot! Then close out of your terminal and open a new one."
