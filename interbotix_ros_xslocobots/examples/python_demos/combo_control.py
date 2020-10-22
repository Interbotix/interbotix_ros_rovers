from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script makes the end-effector go to a specific pose by defining the pose components. It also shows how to move the pan-tilt-mechanism and base
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx200'
# Then change to this directory and type 'python combo_control.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx200", arm_model="mobile_wx200")
    locobot.arm.go_to_home_pose()
    locobot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
    locobot.camera.pan(position=1.0, profile_velocity=1.0, profile_acceleration=0.3)
    locobot.camera.tilt(position=1.0, profile_velocity=0.5, profile_acceleraton=0.3)
    locobot.camera.pan_tilt_move(-1, -1)
    locobot.camera.pan_tilt_go_home()
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()
    locobot.base.move(x=0.4, yaw=0.5, duration=5.0)
    locobot.base.move(x=-0.4, yaw=-0.5, duration=5.0)

if __name__=='__main__':
    main()
