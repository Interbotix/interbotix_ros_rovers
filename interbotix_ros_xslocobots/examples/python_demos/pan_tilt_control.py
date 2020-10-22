from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script commands arbitrary positions to the pan-tilt servos when using Time-Based-Profile for its Drive Mode
# When operating motors in 'position' control mode, Time-Based-Profile allows you to easily set the duration of a particular movement
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_base'
# Then change to this directory and type 'python pan_tilt_control.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_base")
    locobot.camera.pan(1)
    locobot.camera.tilt(1)
    locobot.camera.pan_tilt_move(-1, -1)
    locoobot.camera.pan_tilt_go_home()

if __name__=='__main__':
    main()
