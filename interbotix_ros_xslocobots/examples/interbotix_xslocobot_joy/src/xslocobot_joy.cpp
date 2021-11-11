#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "interbotix_xs_msgs/LocobotJoy.h"

static const double MAX_BASE_X = 0.7;         // Max translational motion that the Kobuki base can do is 0.7 m/s
static const double MAX_BASE_THETA = 3.14;    // Max rotational motion that the Kobuki base can do is 3.14 rad/s

// PS3 Controller button mappings
static const std::map<std::string, int> ps3 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_PWM_INC", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_INC", 4},
                                               {"EE_Y_DEC", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"RESET_ODOM", 8},
                                               {"HOME_POSE", 9},
                                               {"PAN_TILT_HOME", 9},
                                               {"SWITCH", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_EE_ROLL", 12},
                                               {"SPEED_INC", 13},
                                               {"SPEED_DEC", 14},
                                               {"SPEED_COURSE", 15},
                                               {"SPEED_FINE", 16},
                                               {"EE_X", 0},            // axes start here
                                               {"EE_Z", 1},
                                               {"BASE_X", 1},
                                               {"BASE_CCW", 2},
                                               {"EE_ROLL", 3},
                                               {"PAN", 3},
                                               {"EE_PITCH", 4},
                                               {"TILT", 4},
                                               {"BASE_CW", 5}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_PWM_INC", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_INC", 4},
                                               {"EE_Y_DEC", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"RESET_ODOM", 8},
                                               {"HOME_POSE", 9},
                                               {"PAN_TILT_HOME", 9},
                                               {"SWITCH", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_EE_ROLL", 12},
                                               {"EE_X", 0},            // axes start here
                                               {"EE_Z", 1},
                                               {"BASE_X", 1},
                                               {"BASE_CCW", 2},
                                               {"EE_ROLL", 3},
                                               {"PAN", 3},
                                               {"EE_PITCH", 4},
                                               {"TILT", 4},
                                               {"BASE_CW", 5},
                                               {"SPEED_TYPE", 6},
                                               {"SPEED", 7}};

ros::Publisher pub_joy_cmd;                                 // ROS Publisher to publish LocobotJoy messages
ros::Subscriber sub_joy_raw;                                // ROS Subscriber to get Joy messages from the 'joy_node'
interbotix_xs_msgs::LocobotJoy prev_joy_cmd;          // Keep track of the previously commanded LocobotJoy message so that only unique messages are published
std::map<std::string, int> cntlr;                           // Holds either the PS3 or PS4 button mappings
std::string controller_type;                                // Holds the name of the controller received from the ROS Parameter server
double threshold;                                           // Joystick sensitivity threshold

/// @brief Joystick callback to create custom LocobotJoy messages to control the Locobot
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static bool R_button_pressed = false;
  static bool L_button_pressed = false;
  static bool switch_cmd = false;
  static bool switch_cmd_last_state = false;
  static bool flip_ee_roll_cmd = false;
  static bool flip_ee_roll_cmd_last_state = false;
  static bool flip_ee_x_cmd = false;
  static bool flip_ee_x_cmd_last_state = false;
  interbotix_xs_msgs::LocobotJoy joy_cmd;

  // Check if the switch_cmd button was pressed
  if (msg.buttons.at(cntlr["SWITCH"]) == 1 && switch_cmd_last_state == false)
    switch_cmd = true;
  else if (msg.buttons.at(cntlr["SWITCH"]) == 1 && switch_cmd_last_state == true)
    switch_cmd = false;
  else if (msg.buttons.at(cntlr["SWITCH"]) == 0)
    switch_cmd_last_state = switch_cmd;

  if (controller_type == "ps3")
  {
    // Check the speed_cmd
    if (msg.buttons.at(cntlr["SPEED_INC"]) == 1)
      joy_cmd.speed_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_INC;
    else if (msg.buttons.at(cntlr["SPEED_DEC"]) == 1)
      joy_cmd.speed_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_DEC;

    // Check the speed_toggle_cmd
    if (msg.buttons.at(cntlr["SPEED_COURSE"]) == 1)
      joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_COURSE;
    else if (msg.buttons.at(cntlr["SPEED_FINE"]) == 1)
      joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_FINE;
  }
  else if (controller_type == "ps4")
  {
    // Check the speed_cmd
    if (msg.axes.at(cntlr["SPEED"]) == 1)
      joy_cmd.speed_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_INC;
    else if (msg.axes.at(cntlr["SPEED"]) == -1)
      joy_cmd.speed_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_DEC;

    // Check the speed_toggle_cmd
    if (msg.axes.at(cntlr["SPEED_TYPE"]) == 1)
      joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_COURSE;
    else if (msg.axes.at(cntlr["SPEED_TYPE"]) == -1)
      joy_cmd.speed_toggle_cmd = interbotix_xs_msgs::LocobotJoy::SPEED_FINE;
  }

  if (switch_cmd == false)
  {
    // Check the base_x_cmd
    joy_cmd.base_x_cmd = msg.axes.at(cntlr["BASE_X"]) * MAX_BASE_X;

    // Check for the first time the R or L buttons are pressed
    // This is necessary to fix a potential bug in the 'joy' package - refer to https://github.com/ros-drivers/joystick_drivers/issues/155
    if (msg.axes.at(cntlr["BASE_CW"]) != 0 && R_button_pressed == false)
      R_button_pressed = true;
    if (msg.axes.at(cntlr["BASE_CCW"]) != 0 && L_button_pressed == false)
      L_button_pressed = true;

    // Check the base_theta_cmd
    if (L_button_pressed && R_button_pressed)
      joy_cmd.base_theta_cmd = (msg.axes.at(cntlr["BASE_CW"]) - msg.axes.at(cntlr["BASE_CCW"]))/2.0 * MAX_BASE_THETA;
    else if (L_button_pressed && !R_button_pressed)
      joy_cmd.base_theta_cmd = (1.0 - msg.axes.at(cntlr["BASE_CCW"]))/2.0 * MAX_BASE_THETA;
    else if (!L_button_pressed && R_button_pressed)
      joy_cmd.base_theta_cmd = (msg.axes.at(cntlr["BASE_CW"]) - 1.0)/2.0 * MAX_BASE_THETA;

    // Check the base_reset_odom_cmd
    if (msg.buttons.at(cntlr["RESET_ODOM"]) == 1)
      joy_cmd.base_reset_odom_cmd = interbotix_xs_msgs::LocobotJoy::RESET_ODOM;

    // Check the pan_cmd
    if (msg.axes.at(cntlr["PAN"]) >= threshold)
      joy_cmd.pan_cmd = interbotix_xs_msgs::LocobotJoy::PAN_CCW;
    else if (msg.axes.at(cntlr["PAN"]) <= -threshold)
      joy_cmd.pan_cmd = interbotix_xs_msgs::LocobotJoy::PAN_CW;

    // Check the tilt_cmd
    if (msg.axes.at(cntlr["TILT"]) >= threshold)
      joy_cmd.tilt_cmd = interbotix_xs_msgs::LocobotJoy::TILT_DOWN;
    else if (msg.axes.at(cntlr["TILT"]) <= -threshold)
      joy_cmd.tilt_cmd = interbotix_xs_msgs::LocobotJoy::TILT_UP;

    // Check if the camera pan-and-tilt mechanism should be reset
    if (msg.buttons.at(cntlr["PAN_TILT_HOME"]) == 1)
    {
      joy_cmd.pan_cmd = interbotix_xs_msgs::LocobotJoy::PAN_TILT_HOME;
      joy_cmd.tilt_cmd = interbotix_xs_msgs::LocobotJoy::PAN_TILT_HOME;
    }
  }
  else
  {
    // Check if the ee_x_cmd should be flipped
    if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state == false)
      flip_ee_x_cmd = true;
    else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 1 && flip_ee_x_cmd_last_state == true)
      flip_ee_x_cmd = false;
    else if (msg.buttons.at(cntlr["FLIP_EE_X"]) == 0)
      flip_ee_x_cmd_last_state = flip_ee_x_cmd;

    // Check the ee_x_cmd
    if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd == false)
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::LocobotJoy::EE_X_INC;
    else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd == false)
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::LocobotJoy::EE_X_DEC;
    else if (msg.axes.at(cntlr["EE_X"]) >= threshold && flip_ee_x_cmd == true)
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::LocobotJoy::EE_X_DEC;
    else if (msg.axes.at(cntlr["EE_X"]) <= -threshold && flip_ee_x_cmd == true)
      joy_cmd.ee_x_cmd = interbotix_xs_msgs::LocobotJoy::EE_X_INC;

    // Check the ee_y_cmd
    if (msg.buttons.at(cntlr["EE_Y_INC"]) == 1)
      joy_cmd.ee_y_cmd = interbotix_xs_msgs::LocobotJoy::EE_Y_INC;
    else if (msg.buttons.at(cntlr["EE_Y_DEC"]) == 1)
      joy_cmd.ee_y_cmd = interbotix_xs_msgs::LocobotJoy::EE_Y_DEC;

    // Check the ee_z_cmd
    if (msg.axes.at(cntlr["EE_Z"]) >= threshold)
      joy_cmd.ee_z_cmd = interbotix_xs_msgs::LocobotJoy::EE_Z_INC;
    else if (msg.axes.at(cntlr["EE_Z"]) <= -threshold)
      joy_cmd.ee_z_cmd = interbotix_xs_msgs::LocobotJoy::EE_Z_DEC;

    // Check if the ee_roll_cmd should be flipped
    if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 1 && flip_ee_roll_cmd_last_state == false)
      flip_ee_roll_cmd = true;
    else if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 1 && flip_ee_roll_cmd_last_state == true)
      flip_ee_roll_cmd = false;
    else if (msg.buttons.at(cntlr["FLIP_EE_ROLL"]) == 0)
      flip_ee_roll_cmd_last_state = flip_ee_roll_cmd;

    // Check the ee_roll_cmd
    if (msg.axes.at(cntlr["EE_ROLL"]) >= threshold && flip_ee_roll_cmd == false)
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::LocobotJoy::EE_ROLL_CW;
    else if (msg.axes.at(cntlr["EE_ROLL"]) <= -threshold && flip_ee_roll_cmd == false)
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::LocobotJoy::EE_ROLL_CCW;
    else if (msg.axes.at(cntlr["EE_ROLL"]) >= threshold && flip_ee_roll_cmd == true)
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::LocobotJoy::EE_ROLL_CCW;
    else if (msg.axes.at(cntlr["EE_ROLL"]) <= -threshold && flip_ee_roll_cmd == true)
      joy_cmd.ee_roll_cmd = interbotix_xs_msgs::LocobotJoy::EE_ROLL_CW;

    // Check the ee_pitch_cmd
    if (msg.axes.at(cntlr["EE_PITCH"]) >= threshold)
      joy_cmd.ee_pitch_cmd = interbotix_xs_msgs::LocobotJoy::EE_PITCH_UP;
    else if (msg.axes.at(cntlr["EE_PITCH"]) <= -threshold)
      joy_cmd.ee_pitch_cmd = interbotix_xs_msgs::LocobotJoy::EE_PITCH_DOWN;

    // Check the waist_cmd
    if (msg.buttons.at(cntlr["WAIST_CCW"]) == 1)
      joy_cmd.waist_cmd = interbotix_xs_msgs::LocobotJoy::WAIST_CCW;
    else if (msg.buttons.at(cntlr["WAIST_CW"]) == 1)
      joy_cmd.waist_cmd = interbotix_xs_msgs::LocobotJoy::WAIST_CW;

    // Check the gripper_cmd
    if (msg.buttons.at(cntlr["GRIPPER_CLOSE"]) == 1)
      joy_cmd.gripper_cmd = interbotix_xs_msgs::LocobotJoy::GRIPPER_CLOSE;
    else if (msg.buttons.at(cntlr["GRIPPER_OPEN"]) == 1)
      joy_cmd.gripper_cmd = interbotix_xs_msgs::LocobotJoy::GRIPPER_OPEN;

    // Check the pose_cmd
    if (msg.buttons.at(cntlr["HOME_POSE"]) == 1)
      joy_cmd.pose_cmd = interbotix_xs_msgs::LocobotJoy::HOME_POSE;
    else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1)
      joy_cmd.pose_cmd = interbotix_xs_msgs::LocobotJoy::SLEEP_POSE;

    // Check the gripper_pwm_cmd
    if (msg.buttons.at(cntlr["GRIPPER_PWM_INC"]) == 1)
      joy_cmd.gripper_pwm_cmd = interbotix_xs_msgs::LocobotJoy::GRIPPER_PWM_INC;
    else if (msg.buttons.at(cntlr["GRIPPER_PWM_DEC"]) == 1)
      joy_cmd.gripper_pwm_cmd = interbotix_xs_msgs::LocobotJoy::GRIPPER_PWM_DEC;
  }

  // Only publish a LocobotJoy message if any of the following fields have changed.
  if (!(prev_joy_cmd.base_x_cmd == joy_cmd.base_x_cmd &&
      prev_joy_cmd.base_theta_cmd == joy_cmd.base_theta_cmd &&
      prev_joy_cmd.base_reset_odom_cmd == joy_cmd.base_reset_odom_cmd &&
      prev_joy_cmd.pan_cmd == joy_cmd.pan_cmd &&
      prev_joy_cmd.tilt_cmd == joy_cmd.tilt_cmd &&
      prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd &&
      prev_joy_cmd.ee_y_cmd == joy_cmd.ee_y_cmd &&
      prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd &&
      prev_joy_cmd.ee_roll_cmd == joy_cmd.ee_roll_cmd &&
      prev_joy_cmd.ee_pitch_cmd == joy_cmd.ee_pitch_cmd &&
      prev_joy_cmd.waist_cmd == joy_cmd.waist_cmd &&
      prev_joy_cmd.gripper_cmd == joy_cmd.gripper_cmd &&
      prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd &&
      prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd &&
      prev_joy_cmd.speed_toggle_cmd == joy_cmd.speed_toggle_cmd &&
      prev_joy_cmd.gripper_pwm_cmd == joy_cmd.gripper_pwm_cmd))
      pub_joy_cmd.publish(joy_cmd);
  prev_joy_cmd = joy_cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locobot_joy");
  ros::NodeHandle n;
  ros::param::get("~threshold", threshold);
  ros::param::get("~controller", controller_type);
  if (controller_type == "ps3")
    cntlr = ps3;
  else
    cntlr = ps4;
  sub_joy_raw = n.subscribe("commands/joy_raw", 10, joy_state_cb);
  pub_joy_cmd = n.advertise<interbotix_xs_msgs::LocobotJoy>("commands/joy_processed", 10);
  ros::spin();
  return 0;
}
