#!/usr/bin/env bash

# USAGE: ./xslocobot_remote_install.sh [-h][-d DISTRO][-p PATH][-b BASE_TYPE][-r HOSTNAME][-i LOCOBOT_IP]
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

NONINTERACTIVE=false
DISTRO_SET_FROM_CL=false
INSTALL_PATH=~/interbotix_ws
FASTRTPS_DEFAULT_PROFILES_FILE="$INSTALL_PATH"/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml
IP_ROUTING_SCRIPT="$INSTALL_PATH"/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/service/ip_routing.sh
IP_ROUTING_SERVICE_FILE="$INSTALL_PATH"/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/service/ip_routing.service
HOSTNAME=""

_usage="${BOLD}USAGE: ./xslocobot_remote_install.sh [-h][-d DISTRO][-p PATH][-b BASE_TYPE][-r HOSTNAME][-i LOCOBOT_IP][-n]${NORM}

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

  -i LOCOBOT_IP   Sets the LoCoBot computer's IP address. If not specified, the script will
                  prompt for this later on.

  -n              Install all packages and dependencies without prompting. This is useful if
                  you're running this script in a non-interactive terminal like when building a
                  Docker image.

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
  echo -e "${ERR}[ERROR] $*${RRE}"
  echo -e "${ERR}[ERROR] Interbotix Installation Failed!${RRE}"
  exit 1
}

function validate_distro() {
  # check if chosen distro is valid and set ROS major version
  if contains_element "$ROS_DISTRO_TO_INSTALL" "${ALL_VALID_DISTROS[@]}"; then
    if contains_element "$ROS_DISTRO_TO_INSTALL" "${ROS1_VALID_DISTROS[@]}"; then
      # Supported ROS 1 distros
      ROS_VERSION_TO_INSTALL=1
    elif contains_element "$ROS_DISTRO_TO_INSTALL" "${ROS2_VALID_DISTROS[@]}"; then
      # Supported ROS2 distros
      ROS_VERSION_TO_INSTALL=2
    else
      # For cases where it passes the first check but somehow fails the second check
      failed "Something went wrong. ROS_DISTRO_TO_INSTALL=$ROS_DISTRO_TO_INSTALL."
    fi
    echo -e "${GRN}${BOLD}Chosen Version: ROS ${ROS_VERSION_TO_INSTALL} $ROS_DISTRO_TO_INSTALL${NORM}${OFF}"
    return 0
  else
    failed "'$ROS_DISTRO_TO_INSTALL' is not a valid ROS Distribution. Choose one of: '${ALL_VALID_DISTROS[*]}'"
  fi
}

function validate_base_type() {
  if [ -z "$BASE_TYPE" ]; then
    failed "You must specify a base type using the '-b' flag followed by one of: '${VALID_BASE_TYPES[*]}'"
  fi
  if contains_element "$BASE_TYPE" "${VALID_BASE_TYPES[@]}"; then
    :
  else
    failed "Base type '$BASE_TYPE' is not valid. Choose one of: '${VALID_BASE_TYPES[*]}'"
  fi
}

function check_ubuntu_version() {
 # check if the chosen distribution is compatible with the Ubuntu version
  case $UBUNTU_VERSION in

    18.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${BIONIC_VALID_DISTROS[@]}"; then
        :
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    20.04 )
      if contains_element "$ROS_DISTRO_TO_INSTALL" "${FOCAL_VALID_DISTROS[@]}"; then
        :
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    *)
      failed "Something went wrong. UBUNTU_VERSION='$UBUNTU_VERSION', should be 18.04 or 20.04."
      ;;

  esac
}


function install_locobot_ros1() {
  if source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash &> /dev/null && \
     source "$INSTALL_PATH"/devel/setup.bash &> /dev/null && \
     rospack list | grep -q interbotix_ &> /dev/null;
  then
    echo "Interbotix LoCoBot ROS 1 packages already installed!"
  else
    # install dependencies
    sudo apt-get install -yq                    \
      ros-"$ROS_DISTRO_TO_INSTALL"-rtabmap-ros  \
      ros-"$ROS_DISTRO_TO_INSTALL"-xacro
    # Install LoCoBot packages
    if [ ! -d "$INSTALL_PATH/src" ]; then
      echo -e "${GRN}Installing Simulation/Visualization ROS packages for the Interbotix LoCoBot...${OFF}"
      cd "$INSTALL_PATH" || exit
      # we run catkin_make before cloning repos to set up devel space
      # don't care about building, just having launch files and resources i.e. descriptions, sim, etc.
      catkin_make
      git clone https://github.com/Interbotix/interbotix_ros_rovers.git src
      git clone https://github.com/Interbotix/create3_sim_ros1.git -b ros1 src
      cd interbotix_ros_rovers && git checkout "$ROS_DISTRO_TO_INSTALL" && cd ..
      echo "source $INSTALL_PATH/devel/setup.bash" >> ~/.bashrc
    else
      echo "Interbotix LoCoBot ROS packages already installed!"
    fi
    source "$INSTALL_PATH"/devel/setup.bash
  fi
}

function install_locobot_ros2() {
  if source /opt/ros/"$ROS_DISTRO_TO_INSTALL"/setup.bash &> /dev/null && \
     source "$INSTALL_PATH"/install/setup.bash &> /dev/null           && \
     ros2 pkg list | grep -q interbotix_ &> /dev/null
  then
    echo "Interbotix LoCoBot ROS packages already installed!"
  else
    echo -e "${GRN}Installing ROS 2 Remote packages for the Interbotix LoCoBot...${OFF}"
    sudo rosdep init
    rosdep update --include-eol-distros
    # install dependencies
    sudo apt-get install -yq ros-"$ROS_DISTRO_TO_INSTALL"-rtabmap-ros
    cd "$INSTALL_PATH"/src || exit
    git clone https://github.com/Interbotix/interbotix_ros_core.git -b "$ROS_DISTRO_TO_INSTALL"
    git clone https://github.com/Interbotix/interbotix_ros_rovers.git -b "$ROS_DISTRO_TO_INSTALL"
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git -b "$ROS_DISTRO_TO_INSTALL"
    # TODO(lsinterbotix) remove below when moveit_visual_tools is available in apt repo
    git clone https://github.com/ros-planning/moveit_visual_tools.git -b ros2
    # remove COLCON_IGNORE files for necessary packages
    rm                                                                                                  \
      interbotix_ros_toolboxes/interbotix_perception_toolbox/COLCON_IGNORE                              \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE      \
      interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE
    # add COLCON_IGNORE files for unnecessary packages
    # this keeps packages:
    #   * interbotix_xs_msgs
    #   * interbotix_xslocobot_descriptions
    #   * interbotix_xslocobot_sim
    touch                                                                                               \
      interbotix_ros_core/interbotix_ros_xseries/interbotix_ros_xseries/COLCON_IGNORE                   \
      interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/COLCON_IGNORE                        \
      interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_ros_xslocobots/COLCON_IGNORE           \
      interbotix_ros_rovers/interbotix_ros_xslocobots/examples/COLCON_IGNORE                            \
      interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_control/COLCON_IGNORE        \
      interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_moveit/COLCON_IGNORE         \
      interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_nav/COLCON_IGNORE            \
      interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_perception/COLCON_IGNORE     \
      interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobot_ros_control/COLCON_IGNORE    \
      interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_toolbox/COLCON_IGNORE                \
      interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_rviz/COLCON_IGNORE
    cd "$INSTALL_PATH" || exit
    rosdep install --from-paths src --ignore-src -r -y --rosdistro="$ROS_DISTRO_TO_INSTALL"
    if colcon build; then
      echo -e "${GRN}${BOLD}Interbotix LoCoBot ROS 2 Remote packages built successfully!${NORM}${OFF}"
      echo "source $INSTALL_PATH/install/setup.bash" >> ~/.bashrc
      source "$INSTALL_PATH"/install/setup.bash
    else
      failed "Failed to build remote packages for the Interbotix LoCoBot."
    fi
  fi
}

function config_rmw() {
  # configures remote computer's RMW
  # shellcheck disable=SC2016,SC2129
  if [ -z "$ROS_DISCOVERY_SERVER" ]; then
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}" >> ~/.bashrc
    echo "export ROS_DISCOVERY_SERVER=${LOCOBOT_IP}:11811" >> ~/.bashrc
    sudo cp "$IP_ROUTING_SERVICE_FILE" /lib/systemd/system/
    sed -i "s/127.0.0.1/${LOCOBOT_IP}/g" "$FASTRTPS_DEFAULT_PROFILES_FILE"
    sed -i "s/10.42.0.15/${LOCOBOT_IP}/g" "$IP_ROUTING_SCRIPT"
    sudo systemctl daemon-reload
    sudo systemctl enable ip_routing.service
  fi
}

function setup_env_vars_ros1() {
  # shellcheck disable=SC2016,SC2129
  # Setup Environment Variables
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
}

function setup_env_vars_ros2() {
  # Setup Environment Variables
  if [ -z "$INTERBOTIX_WS" ]; then
    echo "Setting up Environment Variables..."
    echo -e "export INTERBOTIX_XSLOCOBOT_BASE_TYPE=${BASE_TYPE}"  >> ~/.bashrc
    echo -e "export INTERBOTIX_WS=${INSTALL_PATH}"                >> ~/.bashrc
  else
    echo "Environment variables already set!"
  fi
}

# parse command line arguments
while getopts 'hnd:b:p:r:i:' OPTION;
do
  case "$OPTION" in
    h) help && exit 0;;
    n) NONINTERACTIVE=true;;
    d) ROS_DISTRO_TO_INSTALL="$OPTARG" && DISTRO_SET_FROM_CL=true;;
    b) BASE_TYPE="$OPTARG";;
    p) INSTALL_PATH="$OPTARG";;
    r) HOSTNAME="$OPTARG";;
    i) LOCOBOT_IP="$OPTARG";;
    *) echo "Unknown argument $OPTION" && help && exit 0;;
  esac
done
shift "$((OPTIND-1))"

validate_base_type

if ! command -v lsb_release &> /dev/null; then
  sudo apt update
  sudo apt-get install -yq lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

# set default ROS distro before reading clargs
if [ "$DISTRO_SET_FROM_CL" = false ]; then
  if [ "$UBUNTU_VERSION" == "18.04" ]; then
    if [ "$BASE_TYPE" == "create3" ]; then
      failed "The iRobot Create 3 base is incompatible with Ubuntu 18.04 and ROS 1 Melodic."
    fi
    ROS_DISTRO_TO_INSTALL="melodic"
  elif [ "$UBUNTU_VERSION" == "20.04" ]; then
    ROS_DISTRO_TO_INSTALL="noetic"
  else
    echo -e "${BOLD}${RED}Unsupported Ubuntu version: $UBUNTU_VERSION.${NORM}${OFF}"
    failed "Interbotix LoCoBot only works with Ubuntu 18.04 bionic or 20.04 focal on your hardware."
  fi
fi

validate_distro
check_ubuntu_version

if [ -z "$HOSTNAME" ]; then
  # prompt for hostname
  echo -e "${BLU}${BOLD}What is the hostname of the LoCoBot's computer (type 'hostname' in a terminal to find it)?\n$PROMPT${NORM}${OFF}\c"
  read -r HOSTNAME
fi

if [[ $ROS_VERSION_TO_INSTALL == 2 ]]; then
  if [ -z "$LOCOBOT_IP" ]; then
    # prompt for LoCoBot IP. this only applies to ROS 2
    echo -e "${BLU}${BOLD}What is the wireless IP address of the LoCoBot's computer? This will be the address of the fastdds discovery server."
    echo -e "(type 'ifconfig' in a terminal and search for the wireless interface's 'inet' value to find it)\n$PROMPT${NORM}${OFF}\c"
    read -r LOCOBOT_IP
  fi
fi

if [ "$NONINTERACTIVE" = false ]; then
  echo -e "${BLU}${BOLD}INSTALLATION SUMMARY:"
  echo -e "\tROS Distribution:           ROS ${ROS_VERSION_TO_INSTALL} ${ROS_DISTRO_TO_INSTALL}"
  echo -e "\tLoCoBot hostname:           ${HOSTNAME}"
  echo -e "\tLoCoBot IP address:         ${LOCOBOT_IP}"
  echo -e "\tBase Type:                  ${BASE_TYPE}"
  echo -e "\tInstallation path:          ${INSTALL_PATH}"
  echo -e "\nIs this correct?\n${PROMPT}${NORM}${OFF}\c"
  read -r resp

  if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
    :
  else
    help && exit 0
  fi
fi

echo -e "\n# Interbotix Configurations" >> ~/.bashrc

export INTERBOTIX_XSLOCOBOT_BASE_TYPE=${BASE_TYPE}

# Update the system
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get -y autoremove

mkdir -p "$INSTALL_PATH"/src

shopt -s extglob

if [[ "$ROS_VERSION_TO_INSTALL" == 1 ]]; then
  install_locobot_ros1
  setup_env_vars_ros1
elif [[ "$ROS_VERSION_TO_INSTALL" == 2 ]]; then
  install_locobot_ros2
  setup_env_vars_ros2
  config_rmw
else
  failed "Something went wrong. ROS_VERSION_TO_INSTALL='$ROS_VERSION_TO_INSTALL', should be 1 or 2."
fi

shopt -u extglob

if [[ "$ROS_VERSION_TO_INSTALL" == 1 ]]; then
  echo "Remote Installation Complete! Close this terminal and open a new one to finish."
  echo -en "NOTE: Remember to comment out the ${ORANGE}source $INSTALL_PATH/devel/setup.bash${OFF} "
  echo -en "and ${ORANGE}export ROS_MASTER_URI=http://$HOSTNAME.local:11311${OFF} lines from the "
  echo -e  ".bashrc file when done using the LoCoBot! Then close out of your terminal and open a new one."
elif [[ "$ROS_VERSION_TO_INSTALL" == 2 ]]; then
  echo "Remote Installation Complete! reboot your computer to finish."
  echo -en "NOTE: Remember to comment out the ${ORANGE}source $INSTALL_PATH/install/setup.bash${OFF} and "
  echo -en "${ORANGE}export ROS_DISCOVERY_SERVER=${LOCOBOT_IP}:11811${OFF} lines from the .bashrc file "
  echo -en "and disable the ip_routing service when done using the LoCoBot! Then reboot your computer."
fi
