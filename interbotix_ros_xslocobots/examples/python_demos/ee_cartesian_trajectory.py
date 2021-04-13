from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script makes the end-effector draw a square in 3D space
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200'
# Then change to this directory and type 'python ee_cartesian_trajectory.py'

def main():
    locobot = InterbotixLocobotXS("locobot_wx200", "mobile_wx200")
    locobot.arm.go_to_home_pose()
    locobot.arm.set_ee_cartesian_trajectory(z=-0.2)
    locobot.arm.set_ee_cartesian_trajectory(x=-0.2)
    locobot.arm.set_ee_cartesian_trajectory(z=0.2)
    locobot.arm.set_ee_cartesian_trajectory(x=0.2)
    locobot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
