from interbotix_xs_modules.locobot import InterbotixLocobotXS
import numpy as np

# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s show_lidar:=true'
# Then change to this directory and type 'python bartender.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")
    locobot.arm.set_ee_pose_components(x=0.3, z=0.2)
    locobot.arm.set_single_joint_position("waist", np.pi/4.0)
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.close()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.set_single_joint_position("waist", -np.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    locobot.arm.set_single_joint_position("waist", np.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
