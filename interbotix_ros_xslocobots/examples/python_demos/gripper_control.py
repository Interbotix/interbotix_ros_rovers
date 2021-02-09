from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script closes and opens the gripper twice, changing the gripper pressure half way through
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_px100'
# Then change to this directory and type 'python gripper_control.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_px100", arm_model="mobile_px100")
    locobot.arm.close_gripper(2.0)
    locobot.arm.open_gripper(2.0)
    locobot.arm.set_gripper_pressure(1.0)
    locobot.arm.close_gripper(2.0)
    locobot.arm.open_gripper(2.0)

if __name__=='__main__':
    main()
