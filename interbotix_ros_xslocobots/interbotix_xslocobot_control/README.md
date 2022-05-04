# interbotix_xslocobot_control

[![View Documentation](https://trossenrobotics.com/docs/docs_button.svg)](https://www.trossenrobotics.com/docs/interbotix_xslocobots/ros_packages/locobot_control.html)

## Overview

This package contains the configuration and launch files necessary to easily start the various components of the X-Series Locobot platform. This includes launching the **xs_sdk** node responsible for driving the Dynamixel motors on the robot, loading the URDF to the `robot_description` parameter, starting the Kobuki base, and activating the RealSense D435 camera and RPLidar 2D laser scanner. Essentially, this package is what all 'downstream' ROS packages should reference to get the robot up and running.
