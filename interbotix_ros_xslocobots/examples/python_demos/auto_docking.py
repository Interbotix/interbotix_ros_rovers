import math
from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script sends the Locobot to its docking station to charge
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_MODEL use_base:=true dock:=true'
# Then change to this directory and type 'python auto_docking.py'

# Change this value to your locobot model
MODEL = "locobot_px100"

def main(): 
    locobot = InterbotixLocobotXS(robot_model=MODEL)
    if locobot.base.auto_dock():
        print("Docking Successful.")
    else:
        print("Docking Unsuccessful.")


if __name__ == "__main__":
    main()