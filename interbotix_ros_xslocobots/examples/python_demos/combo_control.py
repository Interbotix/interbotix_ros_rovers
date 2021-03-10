import math
from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script manipulates the arm, pan/tilts the camera, and moves the base
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200 use_nav:=true use_lidar:=true rtabmap_args:=-d'
# Then change to this directory and type 'python combo_control.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx200", arm_model="mobile_wx200", use_move_base_action=True)
    locobot.arm.set_ee_pose_components(x=0.3, z=0.2)
    locobot.arm.set_single_joint_position("waist", math.pi/4.0)
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.close()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.set_single_joint_position("waist", -math.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    locobot.arm.set_single_joint_position("waist", math.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

    locobot.base.move_to_pose(1, 1, 3.14, True)
    locobot.base.move_to_pose(0, 0, 0, True)

    locobot.arm.go_to_home_pose()
    locobot.arm.set_ee_cartesian_trajectory(z=-0.2)
    locobot.arm.set_ee_cartesian_trajectory(x=-0.2)
    locobot.arm.set_ee_cartesian_trajectory(z=0.2)
    locobot.arm.set_ee_cartesian_trajectory(x=0.2)
    locobot.arm.go_to_sleep_pose()

    locobot.camera.pan(1)
    locobot.camera.tilt(1)
    locobot.camera.pan_tilt_move(-1, -1)
    locobot.camera.pan_tilt_go_home()

if __name__=='__main__':
    main()
