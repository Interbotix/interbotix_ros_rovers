from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script commands the base to move to an arbitrary point on a map
# Note that this script assumes you already have a map built
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx250s localization:=true'
# Then change to this directory and type 'python locobot_move.py'

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx200", arm_model="mobile_wx200", use_move_base_action=True)
    locobot.base.move_to_pose(1, 1, 0, True)

if __name__=='__main__':
    main()
