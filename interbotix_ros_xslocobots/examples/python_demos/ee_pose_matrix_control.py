from interbotix_xs_modules.locobot import InterbotixLocobotXS
import numpy as np

# This script makes the end-effector go to a specific pose only possible with a 6dof arm using a transformation matrix
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s show_lidar:=true'
# Then change to this directory and type 'python ee_pose_matrix_control.py'

def main():
    T_sd = np.identity(4)
    T_sd[0,3] = 0.3
    T_sd[1,3] = 0.1
    T_sd[2,3] = 0.2

    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")
    locobot.arm.go_to_home_pose()
    locobot.arm.set_ee_pose_matrix(T_sd)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
