from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script commands PWMs to the arm joints
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s show_lidar:=true'
# Then change to this directory and type 'python joint_pwm_control.py'

def main():
    joint_pwms = [0, 200 , 200, 50, 0]
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")
    locobot.dxl.robot_set_operating_modes("group", "arm", "pwm")
    locobot.dxl.robot_write_commands("arm", joint_pwms)

if __name__=='__main__':
    main()
