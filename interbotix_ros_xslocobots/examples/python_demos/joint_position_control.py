from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s show_lidar:=true'
# Then change to this directory and type 'python joint_position_control.py'

def main():
    joint_positions = [-1.0, 0.5 , 0.5, 0, -0.5, 1.57]
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")
    locobot.arm.go_to_home_pose()
    locobot.arm.set_joint_positions(joint_positions)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
