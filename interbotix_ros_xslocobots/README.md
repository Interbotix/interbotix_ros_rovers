# InterbotiX X-Series Locobot ROS Packages
![xslocobot_banner](images/xslocobot_banner.png)

## Overview
Welcome to the *interbotix_ros_xslocobots* sub-repo! This sub-repository contains ROS packages meant to be used with the various Locobot Rovers sold by Trossen Robotics. Packages were tested on Ubuntu Linux 16.04, 18.04, and 20.04 using ROS Kinetic, Melodic, and Noetic respectively. Additionally, all ROS nodes were written using Python or C++. However, any programming language capable of sending ROS messages can be used to control the robots. To that effect, the core packages inside this repo are as follows:
- **interbotix_xslocobot_nav** - contains the config and launch files necessary to run the Nav Stack on the locobot
- **interbotix_xslocobot_perception** - contains the config and launch files necessary to run the Perception Pipeline on the locobot
- **interbotix_xslocobot_moveit** - contains the config files necessary to launch an arm (on the locobot) using MoveIt either in Gazebo, on the physical robot, or just in Rviz
- **interbotix_xslocobot_gazebo** - contains the config files necessary to launch a locobot in Gazebo, including tuned PID gains for ros_control
- **interbotix_xslocobot_control** - contains the motor configuration files and the 'root' launch file that is responsible for launching the locobot
- **interbotix_xslocobot_ros_control** - contains the config files necessary to setup ROS controllers between MoveIt and the physical robot arm that's on the locobot
- **interbotix_xslocobot_descriptions** - contains the meshes and URDFs (including accurate inertial models for the links) for all locobot platforms

Finally, there is also an **examples** directory containing various demos of how the above mentioned core packages can be used. So what are you waiting for? Let's get started!

<p align="center">
  <a href=”https://www.youtube.com/watch?v=xIril2gF0-Y”>
    <img width="410" height="auto" src="https://www.trossenrobotics.com/shared/github/github_open_source.png">
  </a>
</p>

## IRROS Structure
Refer [here](https://github.com/Interbotix/interbotix_ros_core#code-structure) to get a general understanding of IRROS.
![xslocobot_irros_structure](images/xslocobot_irros_structure.png)

##### Hardware Layer
All locobots contain arms and pan/tilt mechanisms made up of [X-Series Dynamixel servos](https://www.trossenrobotics.com/dynamixel-x-series-robot-servos). Each servo has two 3-pin JST ports that allows it to be daisy chained with other servos using 3-pin cables. The 'root' Dynamixels (i.e. the 'waist' and 'pan' motors) then connect to the [XM/XL motor power hub](https://www.trossenrobotics.com/3-pin-x-series-power-hub.aspx). Besides for providing 12V to the motors from the barrel jack, the hub also connects to the 3-pin JST port on the [U2D2](https://www.trossenrobotics.com/dynamixel-u2d2.aspx). This device acts as a communication interface between a computer (connected via microUSB cable) and the motors - converting USB/TTL signals back and forth.

Besides for the Dynamixel servo, there is another type of actuator present on the locobot that drives the Kobuki base. This is shown on the bottom left of the diagram above. Two of these actuators exist to drive the two wheels on the base.

On the right side of this layer, there are three sensors. All locobots come with the [RealSense D435 camera](https://www.intelrealsense.com/depth-camera-d435/) used to perform depth and color sensing. Additionally, there is the [A2M8 RPLidar](https://www.slamtec.com/en/Lidar/A2) navigation scanner which can be added on if desired. Finally, a joystick controller can be used to control the robot.

##### Driver Layer
The ROS packages in this sub-repo build up from the *interbotix_xs_sdk* ROS wrapper found in the *interbotix_ros_core* repository. Reference the package there for implementation details. The *realsense_ros* and *joy* packages are ROS wrappers around the RealSense camera and PS3/PS4 controller devices respectively. Similarly, the *rplidar* package is a a ROS Wrapper around the 360 degree laser scanner.

##### Control Layer
The *interbotix_xslocobot_control* ROS package found in this layer holds the config files for every one of our X-Series locobots. These config files define the names of the joints that make up each arm and pant/tilt mechansim as well as initial values for the motor registers. The launch file inside the package then passes the appropriate parameters to the *interbotix_xs_sdk* driver node depending on the type of locobot being used.

##### Application Support Layer
The three main items shown in this layer can be found in the *interbotix_ros_toolboxes* repository [here](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/main/interbotix_xs_toolbox). Specifically, the locobot module can be found within the *interbotix_xs_modules* ROS package in a file called 'locobot.py'. It essentially provides a small API to allow users to control any of the locobot's actuators in Python - no ROS experience necessary. Additionally, the *interbotix_xs_ros_control* and *interbotix_moveit_interface* packages make it possible for the *interbotix_xslocobot_ros_control* and *interbotix_xslocobot_moveit_interface* packages respectively to function properly.

##### Research Layer
All the ROS packages and Python scripts found within the [examples](examples/) directory fall in this category.

## Compatible Products
The ROS packages located here can be used with any of the Interbotix Locobot kits linked below. Next to each name is the name used to describe it in software (specifically for the `robot_model` argument in launch files) and a description. There are up to four parts in a name. The first word 'locobot' specifies that the robot is a type of rover. The next two letters represent model type (ex. 'wx' for 'WidowX'). The number afterwards corresponds to the length of both the arm's forearm and upper-arm links in millimeters. Finally, the 's' after some numbers signifies if that arm has six degrees of freedom. If the robot has 'base' in it, that means that it has no arm.

- [Locobot Base Robot Rover]() (**locobot_base**): equipped with a [Kobuki](http://kobuki.yujinrobot.com/about2/) mobile base, a [RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) Depth camera on a [Dynamixel pan-tilt servo](http://emanual.robotis.com/docs/en/dxl/x/2xl430-w250/), and a [NUC7i3BNH](https://www.intel.com/content/www/us/en/products/boards-kits/nuc/kits/nuc7i3bnh.html) Intel Computer, this rover is ready to perform some serious navigation and mapping tasks. For even more flexibility, the [RPLidar A2M8](https://www.slamtec.com/en/Lidar/A2) laser scanner can be added as well.
- [Locobot PincherX 100 Robot Rover]() (**locobot_px100**): with all the features of the **locobot_base** platform, this robot steps it up a notch by including the 4dof [PincherX 100](https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx) Interbotix Arm. Now officially a mobile-manipulator, this rover can perform manipulation tasks in addition to navigating/mapping an environment.
- [Locobot WidowX 200 Robot Rover]() (**locobot_wx200**): similar in structure to the **locobot_px100** robot, this platform substitutes the PincherX 100 arm with the 5dof [WidowX 200](https://www.trossenrobotics.com/widowx-200-robot-arm-mobile-base.aspx) Interbotix Arm. With longer range, a higher payload capacity, and an extra degree of freedom, this rover makes your manipulation tasks easier to perform.
- [Locobot WidowX 250 6DOF Robot Rover]() (**locobot_wx250s**): similar in structure to the **locobot_wx200** rover, this platform substitutes the WidowX 200 arm with the 6dof [WidowX 250 6DOF](https://www.trossenrobotics.com/widowx-250-mobile-robot-arm-6dof.aspx) Interbotix Arm. With even longer range, a higher payload capacity, and yet another degree of freedom, this platform raises the bar on research mobile-manipulators.

## Requirements
Below is a list of the hardware you will need to get started:
- Keyboard, mouse, HDMI monitor, and HDMI cable
- One of the X-Series Locobot Kits mentioned above
- A Linux Computer with ROS (for remote networking)

## Hardware Setup
Follow the [assembly guide]() on our website to fully build the locobot. Note that the USB connections to the Kobuki base and RPLidar (if present) can be left unplugged for now. Instead, plug in a mouse, keyboard, and monitor to the computer, and follow the power-on sequence below.

- Press the circular gray button on the rectangular battery (located right above the Kobuki base). One to four white LED lights should illuminate, indicating the power level of the battery. If only one LED is white, that means the battery should be charged. Also, the LEDs on the Dynamixel servos should briefly flash red.
- Turn on the computer. A red light should appear in the [U2D2](https://www.trossenrobotics.com/dynamixel-u2d2.aspx).
- For normal operation, you could now turn on the Kobuki base by pressing the On/Off switch on the side of the platform. The base should make a chirping noise and a green 'Status' LED should appear. If the LED is yellow, this means the base must be charged. However, the base could be turned on at any time before, after, or during this power-on sequence. Also, the base does not have to be on to perform the installation below.

<p align="center">
    <a href=”https://www.youtube.com/watch?v=PQxgWxqFeZg”>
        <img width="410" height="auto" src="https://www.trossenrobotics.com/shared/github/github_locobot_hardware.png">
    </a>
</p>

## Software Setup
To get all the code setup, refer to the computer platform types below (currently only one option, but this may change in the future) and run the appropriate installation script. Afterwards, continue with the [Installation Checks](#installation-checks) sub-section.

<p align="center">
    <a href=”https://www.youtube.com/watch?v=0lnbw6n6vs4”>
        <img width="410" height="auto" src="https://www.trossenrobotics.com/shared/github/github_locobot_software.png">
    </a>
</p>

###### AMD64 Architecture
If you purchased a NUC-based ROS Locobot from our website, note that it comes pre-installed with [Ubuntu Desktop image (20.04)](https://releases.ubuntu.com/focal/) already on it. After powering it on via the steps in the previous section, a login screen should appear with **locobot** as the user name. Conveniently, the password is the same as the user name so type *locobot* and hit **Enter**. Next, update the computer by performing the following steps.

1. Connect to the Internet. This is as simple as clicking the Wifi icon on the top right of the Desktop and selecting your desired network.
2. Press **Ctrl-Alt-T** to open a terminal screen, and type `sudo apt update`.
3. After the process completes, type `sudo apt -y upgrade`. It might take a few minutes for the computer to upgrade.
4. Finally, type `sudo apt -y autoremove` to get rid of unnecessary software packages. Then close out of the terminal and reboot the computer.
5. Once rebooted, login and open up a terminal as before. Instead of manually installing all the software needed for the robot, you will download and run an installation script. Follow the commands below to get started! Note that nothing (camera, RPLidar, Kobuki, U2D2) needs to be connected to the computer for the installation to work.

        $ sudo apt install curl
        $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/amd64/xslocobot_amd64_install.sh' > xslocobot_amd64_install.sh
        $ chmod +x xslocobot_amd64_install.sh
        $ ./xslocobot_amd64_install.sh

6. Once the script is done, shutdown the computer, and remove the HDMI cable, keyboard, and mouse. Replug any sensors into the computer that were unplugged initially. Then turn the computer on again by pressing the power button.

#### Remote Install
For some robotic projects, you may want to run your robot in a 'headless' state on some computer (like a NUC or Raspberry Pi), and monitor the robot's state (in Rviz for example) on your personal (a.k.a remote) computer over a local network. For this to work, run the installation script below on your personal Linux computer. Note that ROS and Rviz must already be installed! As an FYI, the script will prompt you to insert the hostname of the robot (NOT the remote) computer. As an example, if you wanted to monitor the state of a NUC-based locobot, you would set the hostname to `locobot`. To find out the hostname of the robot computer, just open a terminal and type `hostname`.

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_rovers/main/interbotix_ros_xslocobots/install/xslocobot_remote_install.sh' > xslocobot_remote_install.sh
    $ chmod +x xslocobot_remote_install.sh
    $ ./xslocobot_remote_install.sh

Be aware that the installation script will export the ROS_MASTER_URI environment variable in your personal computer's ~/.bashrc file to `http://<hostname>.local:11311`. Make sure to comment out this line when done monitoring or your personal computer will complain about not being able to find its ROS Master.

To SSH from your remote to the robot computer, first connect your personal Linux computer to the same network to which the locobot is connected. Then open a terminal and SSH into the locobot by typing (assuming a NUC-based locobot)...

    $ ssh -X locobot@locobot.local

You will be prompted for a password - just type *locobot* and you should be in!

The *-X* flag in the command above allows window forwarding. This means that it's possible to open small graphical applications on the locobot computer which will be forwarded to your personal computer. Let's open the terminal application by...

    $ gnome-terminal &

Now, we can open up new terminals (via **Ctrl-Shift-T**) on the locobot computer without having to SSH each time. Note that unless otherwise stated, all the following commands should be executed in the new terminal window that pops up.

#### Installation Checks

After running the installation script on the robot computer, verify that it was successful in finding the U2D2, Kobuki, and Lidar (if applicable) by checking that the port names show up as `ttyDXL`, `kobuki`, and `rplidar` (if applicable) respectively. Note that these sensors should be plugged back in at this point if they're not already.

    $ cd /dev
    $ ls
    $ cd

Verify that the RealSense camera can be found by typing `rs-enumerate-devices -S` in the terminal. The output should give info about the type of RealSense camera that is plugged in. Note that the camera should be plugged back in at this point if it's not already.

For ROS Melodic users, open the following Gazebo config file to fix an issue described [here](https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/).

    $ nano ~/.ignition/fuel/config.yaml

Now change the url inside from `https://api.ignitionfuel.org` to `https://api.ignitionrobotics.org`.

## Quickstart

1. Get familiar with the physical robot rover (let's say... a Locobot WidowX 250 6DOF with lidar!) by executing the following command in the terminal:

        $ roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true

    Now, in a terminal on *your remote computer* (not via SSH), type...

        $ roslaunch interbotix_xslocobot_descriptions remote_view.launch

    Rviz should appear on your remote computer and display a virtual real-time representation of the robot!

2. By default, all the Dynamixel motors in the robot are torqued on so it will be very difficult to manually manipulate them. To torque off all the motors, execute the command below in another terminal (either via SSH or on your remote computer). Be aware though that this will cause the robot arm (if present) to collapse (if it's not already resting) so manually hold or secure it before executing.

        $ rosservice call /locobot/torque_enable "{cmd_type: 'group', name: 'all', enable: false}"

    The command above torques off every motor in the 'all' group. This is a special group that includes every Dynamixel motor. To only torque off the arm motors, change the name from 'all' to 'arm'. Likewise, to only torque off the motors controlling the camera, change the name from 'all' to 'camera'.

    Now you should be able to freely manipulate the arm, gripper, and pan/tilt mechanism. Take note of how the Rviz model accurately mimics the real robot. To make the robot hold a certain pose, manually hold the arm in the desired pose and execute the following command:

        $ rosservice call /locobot/torque_enable "{cmd_type: 'group', name: 'all', enable: true}"

    You can now let go and observe how the arm and pan/tilt mechanism stay in place.

3. Now let's get the Kobuki base moving and visualize the sensor output! In the Rviz window, check the *Camera* and *LaserScan* displays and adjust the topic names as necessary. You should see image data from the camera streaming in the lower left corner of the window, and small red pixels being displayed in the Rviz grid from the lidar. To actually move the Kobuki base with a translational velocity of 0.5 m/s and angular velocity of 0.3 rad/s, type the following in another terminal...

        $ rostopic pub -r 10 /locobot/mobile_base/commands/velocity geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

    To stop the base from moving, just `Cntrl-C` the publisher.

4. Shutdown all nodes by pressing **Ctrl-C** in the terminal where you started the launch file. To shutdown the locobot computer, type `sudo poweroff` in the terminal. Then press and hold the circular gray button on the battery until the white LEDs next to it turn off. Next, flick the switch on the side of the Kobuki base off.

Note that the remote installation script sets the ROS_MASTER_URI variable in your remote computer's `~/.bashrc` file to `http://locobot.local:11311`. When working with the locobot, this line (towards the bottom of the file) should be uncommented. Otherwise, you should comment it out so that you can run ROS on your remote computer normally.

That ends the quickstart tutorial. To get familiar with the architecture and launch file arguments, refer to the READMEs of the core packages. Start with the [interbotix_xslocobot_descriptions](interbotix_xslocobot_descriptions/) package, then the [interbotix_xslocobot_control](interbotix_xslocobot_control/) package. Next, look at the [interbotix_xslocobot_gazebo](interbotix_xslocobot_gazebo/) package followed by the [interbotix_xslocobot_ros_control](interbotix_xslocobot_ros_control/) and [interbotix_xslocobot_moveit](interbotix_xslocobot_moveit/) packages. This is the most logical approach to take to gain a better understanding of how they relate to each other. Afterwards, feel free to check out the demo projects in the [examples](examples/) directory.

## Troubleshooting
Refer to the guide [here](https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_sdk/TROUBLESHOOTING.md#troubleshooting-a-dynamixel-based-robot) to try to solve your problem. If you still need help, feel free to contact us as trsupport@trossenrobotics.com or submit an Issue. We strongly recommend the latter option though so that other people who may be facing the same difficulty can benefit. This repository is actively maintained and any open Issues will be addressed as soon as possible.

## Future Work
- Create a more flushed out Gazebo simulation experience by simulating sensors correctly.

## Contributing
To contribute your own custom X-Series locobot in this repo, you will need to do the following steps:
- Create a motor config file similar to the YAML files found [here](interbotix_xslocobot_control/config/) (excluding the 'modes.yaml' file). To get familiar with the parameter names, checkout the [Motor Config Template](https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_sdk/config/motor_configs_template.yaml). Note that the name of this file is what defines your *robot_model* name, and should be used when naming other files like the URDF.
- Create a URDF similar in structure to the ones found [here](interbotix_xslocobot_descriptions/urdf/). Don't forget to put all necessary meshes in the [meshes](interbotix_xslocobot_descriptions/meshes/) directory! As an FYI, you should follow the naming convention for the links, joints, and frame poses as found in the other arm files for consistency.
- Create a set of Gazeo/ROS position controllers similar to the ones found [here](interbotix_xslocobot_gazebo/config/position_controllers/).
- Create a set of Gazeo/ROS trajectory controllers similar to the ones found [here](interbotix_xslocobot_gazebo/config/trajectory_controllers/).
- Create an SRDF file for Moveit similar to the ones found [here](interbotix_xslocobot_moveit/config/srdf/). You should first use the MoveIt Setup Assistant Wizard for this step and then edit the generated SRDF file based on the structure of those files.
- If you are integrating your own Dynamixel-based custom designed arm (not one of the Interbotix models), add the appropriate Screw axes and M matrices to the [mr_descriptions](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/mr_descriptions.py) module (syntax should be something like *mobile_XXXXX*). For help doing this, refer to Chapter 4 in [Modern Robotics](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) and [this video](https://www.youtube.com/watch?v=cKHsil0V6Qk&ab_channel=NorthwesternRobotics).
- Make sure to follow the same naming convention, structure, and documentation procedures as found in the repo before making a PR.

## Contributors
- [Solomon Wiznitzer](https://github.com/swiz23) - **ROS Engineer**
- [Levi Todes](https://github.com/LeTo37) - **CAD Engineer**
