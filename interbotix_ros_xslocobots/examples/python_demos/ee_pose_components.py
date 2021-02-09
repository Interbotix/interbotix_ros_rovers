from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script makes the end-effector go to a specific pose by defining the pose components
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200'
# Then change to this directory and type 'python ee_pose_components.py'

def main():
    locobot = InterbotixLocobotXS("locobot_wx200", "mobile_wx200")
    locobot.arm.go_to_home_pose()
    locobot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
