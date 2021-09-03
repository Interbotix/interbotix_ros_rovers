from interbotix_xs_modules.locobot import InterbotixLocobotXS as Locobot

# This script sends the Locobot to its docking station to charge
#   For best performance, robot should be under 1m in front of the dock
#   Note that this script should only be run on robots with smaller or no arms
#       - locobot_base
#       - locobot_px100

# Update this value with your model
MODEL = "px100"

# To get started, open a terminal on the robot and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_MODEL use_base:=true use_dock:=true'
#                                                                                ^ update MODEL
# Then open a new terminal in this directory and type 'python auto_docking.py'


def main(): 
    if MODEL is not ("px100" or "base"):
        print("This script should only be run on robots with smaller or no arms")
        exit()

    locobot = Locobot(robot_model="locobot_"+MODEL)
    locobot.base.auto_dock()


if __name__ == "__main__":
    main()
