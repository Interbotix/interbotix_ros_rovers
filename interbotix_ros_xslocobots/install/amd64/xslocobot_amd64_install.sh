#!/usr/bin/env bash

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "16.04" ]; then
	ROS_NAME="kinetic"
elif [ $ubuntu_version == "18.04" ]; then
	ROS_NAME="melodic"
else
	echo -e "Unsupported Ubuntu verison: $ubuntu_version"
	echo -e "Interbotix Locobot only works with 16.04 or 18.04"
	exit 1
fi

echo "Ubuntu $ubuntu_version detected. ROS-$ROS_NAME chosen for installation.";
if [ $ROS_NAME == "melodic" ]; then
	echo "There are no patches available for Kernel 5.4 at this time - needed for the RealSense Camera to work properly."
	echo "The camera should still work with ROS but warnings will appear in your terminal window..."
fi

echo -e "\e[1;33m ******************************************** \e[0m"
echo -e "\e[1;33m The installation may take around 15 Minutes! \e[0m"
echo -e "\e[1;33m ******************************************** \e[0m"
sleep 4
start_time="$(date -u +%s)"

# Update the system
sudo apt update && sudo apt -y upgrade
sudo apt autoremove

# Install some necessary core packages
sudo apt -y install openssh-server
sudo apt -y install python-pip
sudo -H pip install modern_robotics

# Step 1: Install ROS
if [ $(dpkg-query -W -f='${Status}' ros-$ROS_NAME-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing ROS..."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt update
  sudo apt -y install ros-$ROS_NAME-desktop-full
	if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
		sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
	fi
  echo "source /opt/ros/$ROS_NAME/setup.bash" >> ~/.bashrc
  sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo rosdep init
  rosdep update
else
  echo "ros-$ROS_NAME-desktop-full is already installed!"
fi
source /opt/ros/$ROS_NAME/setup.bash

# Step 2: Install Realsense packages

# Step 2A: Install librealsense2
if [ $(dpkg-query -W -f='${Status}' librealsense2 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing librealsense2..."
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
  sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -sc) main" -u
	if [ $ubuntu_version == "16.04" ]; then
		version="2.36.0-0~realsense0.3168"
	elif [ $ubuntu_version == "18.04" ]; then
		version="2.36.0-0~realsense0.3169"
	fi

  sudo apt -y install librealsense2-udev-rules=${version}
	# Patches for Kernel 5.4 is NOT supported yet, so don't even try to install the package below in Melodic
	if [ $ROS_NAME == "kinetic" ]; then
  	sudo apt -y install librealsense2-dkms=1.3.12-0ubuntu1
	fi
  sudo apt -y install librealsense2=${version}
  sudo apt -y install librealsense2-gl=${version}
  sudo apt -y install librealsense2-net=${version}
  sudo apt -y install librealsense2-utils=${version}
  sudo apt -y install librealsense2-dev=${version}
  sudo apt -y install librealsense2-dbg=${version}
  sudo apt-mark hold librealsense2*
  sudo apt -y install ros-$ROS_NAME-ddynamic-reconfigure
else
  echo "librealsense2 already installed!"
fi

# Step 2B: Install realsense2 ROS Wrapper
REALSENSE_WS=~/realsense_ws
if [ ! -d "$REALSENSE_WS/src" ]; then
  echo "Installing RealSense ROS Wrapper..."
  mkdir -p $REALSENSE_WS/src
  cd $REALSENSE_WS/src
  git clone https://github.com/IntelRealSense/realsense-ros.git
  cd realsense-ros/
  git checkout 2.2.15
  cd $REALSENSE_WS
  catkin_make clean
  catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
  catkin_make install
  echo "source $REALSENSE_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "RealSense ROS Wrapper already installed!"
fi
source $REALSENSE_WS/devel/setup.bash

# Step 3: Install apriltag ROS Wrapper
APRILTAG_WS=~/apriltag_ws
if [ ! -d "$APRILTAG_WS/src" ]; then
  echo "Installing Apriltag ROS Wrapper..."
  mkdir -p $APRILTAG_WS/src
  cd $APRILTAG_WS/src
  git clone https://github.com/AprilRobotics/apriltag.git
  git clone https://github.com/AprilRobotics/apriltag_ros.git
  cd $APRILTAG_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make_isolated
  echo "source $APRILTAG_WS/devel_isolated/setup.bash" >> ~/.bashrc
else
  echo "Apriltag ROS Wrapper already installed!"
fi
source $APRILTAG_WS/devel_isolated/setup.bash

# Step 4: Install Locobot packages
INTERBOTIX_WS=~/interbotix_ws
if [ ! -d "$INTERBOTIX_WS/src" ]; then
  echo "Installing ROS packages for the Interbotix Locobot..."
  mkdir -p $INTERBOTIX_WS/src
  cd $INTERBOTIX_WS/src
	if [ $ROS_NAME != "kinetic" ]; then
		echo "Building Kobuki ROS packages from source..."
		git clone https://github.com/yujinrobot/kobuki
		cd kobuki
		git checkout melodic
		sudo rm -r kobuki_capabilities kobuki
		cd $INTERBOTIX_WS/src
	fi
  git clone https://github.com/Interbotix/interbotix_ros_core.git
  git clone https://github.com/Interbotix/interbotix_ros_rovers.git
	cd interbotix_ros_rovers
	git checkout $ROS_NAME
	cd ..
  git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
  cd $INTERBOTIX_WS/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
  sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  cd $INTERBOTIX_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  echo "source $INTERBOTIX_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix Locobot ROS packages already installed!"
fi
source $INTERBOTIX_WS/devel/setup.bash

# Step 5: Setup Environment Variables
if [ -z "$ROS_IP" ]; then
	echo "Setting up Environment Variables..."
	echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
	echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
	echo "Environment variables already set!"
fi

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to reboot the computer before using the robot!"
