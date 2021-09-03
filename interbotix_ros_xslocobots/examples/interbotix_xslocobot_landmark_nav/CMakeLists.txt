cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xslocobot_landmark_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    interbotix_xslocobot_control
    interbotix_xslocobot_nav
    interbotix_xslocobot_perception
    rospy
    tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
# add_message_files(
#     FILES
  
# )

## Generate services in the 'srv' folder
# add_service_files(
    # FILES
  
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
    CATKIN_DEPENDS 
        interbotix_xslocobot_control 
        interbotix_xslocobot_nav 
        interbotix_xslocobot_perception 
        rospy 
        tf2_ros 
        geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
    scripts/nav_to_landmark
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
