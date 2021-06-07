import math
from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script sends the Locobot to its docking station to charge
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx250s show_lidar:=true'
# Then change to this directory and type 'python auto_docking.py'

def main(): 
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s")
    locobot.base.auto_dock()

if __name__ == "__main__":
    main()