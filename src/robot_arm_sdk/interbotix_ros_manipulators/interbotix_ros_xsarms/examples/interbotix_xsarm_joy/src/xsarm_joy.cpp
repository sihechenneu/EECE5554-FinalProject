#include <map>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <interbotix_xs_msgs/msg/arm_joy.hpp>

static const std::map<std::string, int> ps3 = {
  {"GRIPPER_PWM_DEC", 0}, {"GRIPPER_OPEN", 1}, {"GRIPPER_PWM_INC", 2}, {"GRIPPER_CLOSE", 3},
  {"EE_Y_INC", 4}, {"EE_Y_DEC", 5}, {"WAIST_CCW", 6}, {"WAIST_CW", 7}, {"SLEEP_POSE", 8}, {"HOME_POSE", 9},
  {"TORQUE_ENABLE", 10}, {"FLIP_EE_X", 11}, {"FLIP_EE_ROLL", 12}, {"SPEED_INC", 13}, {"SPEED_DEC", 14},
  {"SPEED_COURSE", 15}, {"SPEED_FINE", 16}, {"EE_X", 0}, {"EE_Z", 1}, {"EE_ROLL", 3}, {"EE_PITCH", 4}};
static const std::map<std::string, int> ps4 = {
  {"GRIPPER_PWM_DEC", 0}, {"GRIPPER_OPEN", 1}, {"GRIPPER_PWM_INC", 2}, {"GRIPPER_CLOSE", 3},
  {"EE_Y_INC", 4}, {"EE_Y_DEC", 5}, {"WAIST_CCW", 6}, {"WAIST_CW", 7}, {"SLEEP_POSE", 8}, {"HOME_POSE", 9},
  {"TORQUE_ENABLE", 10}, {"FLIP_EE_X", 11}, {"FLIP_EE_ROLL", 12},
  {"EE_X", 0}, {"EE_Z", 1}, {"EE_ROLL", 3}, {"EE_PITCH", 4}, {"SPEED_TYPE", 6}, {"SPEED", 7}};
static const std::map<std::string, int> xbox360 = {
  {"GRIPPER_PWM_DEC", 0}, {"GRIPPER_OPEN", 1}, {"GRIPPER_CLOSE", 2}, {"GRIPPER_PWM_INC", 3},
  {"WAIST_CCW", 4}, {"WAIST_CW", 5}, {"SLEEP_POSE", 6}, {"HOME_POSE", 7}, {"TORQUE_ENABLE", 8},
  {"FLIP_EE_X", 9}, {"FLIP_EE_ROLL", 10},
  {"EE_X", 0}, {"EE_Z", 1}, {"EE_Y_INC", 2}, {"EE_ROLL", 3}, {"EE_PITCH", 4}, {"EE_Y_DEC", 5},
  {"SPEED_TYPE", 6}, {"SPEED", 7}};

class XsarmJoy : public rclcpp::Node
{
public:
  XsarmJoy() : Node("xsarm_joy"), controller_type_("ps4"), threshold_(0.75)
  {
    declare_parameter<double>("threshold", 0.75);
    declare_parameter<std::string>("controller", "ps4");
    threshold_ = get_parameter("threshold").as_double();
    controller_type_ = get_parameter("controller").as_string();
    if (controller_type_ == "xbox360") cntlr_ = xbox360;
    else if (controller_type_ == "ps3") cntlr_ = ps3;
    else cntlr_ = ps4;
    sub_ = create_subscription<sensor_msgs::msg::Joy>("commands/joy_raw", 10, std::bind(&XsarmJoy::joy_cb, this, std::placeholders::_1));
    pub_ = create_publisher<interbotix_xs_msgs::msg::ArmJoy>("commands/joy_processed", 10);
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    using interbotix_xs_msgs::msg::ArmJoy;
    static bool flip_ee_roll = false, flip_ee_roll_last = false, flip_ee_x = false, flip_ee_x_last = false;
    static bool flip_torque = true, flip_torque_last = true;
    static double time_start = 0;
    static bool timer_started = false;
    ArmJoy joy_cmd;

    if (msg->buttons.size() <= static_cast<size_t>(cntlr_["TORQUE_ENABLE"])) return;
    if (msg->axes.size() <= static_cast<size_t>(std::max({cntlr_["EE_X"], cntlr_["EE_Z"], cntlr_["EE_ROLL"], cntlr_["EE_PITCH"]}))) return;

    if (msg->buttons[cntlr_["TORQUE_ENABLE"]] == 1 && !flip_torque_last) {
      flip_torque = true;
      joy_cmd.torque_cmd = ArmJoy::TORQUE_ON;
    } else if (msg->buttons[cntlr_["TORQUE_ENABLE"]] == 1 && flip_torque_last) {
      time_start = now().seconds();
      timer_started = true;
    } else if (msg->buttons[cntlr_["TORQUE_ENABLE"]] == 0) {
      if (timer_started && now().seconds() - time_start > 3.0) {
        joy_cmd.torque_cmd = ArmJoy::TORQUE_OFF;
        flip_torque = false;
      }
      flip_torque_last = flip_torque;
      timer_started = false;
    }

    if (msg->buttons[cntlr_["FLIP_EE_X"]] == 1 && !flip_ee_x_last) flip_ee_x = true;
    else if (msg->buttons[cntlr_["FLIP_EE_X"]] == 1 && flip_ee_x_last) flip_ee_x = false;
    else if (msg->buttons[cntlr_["FLIP_EE_X"]] == 0) flip_ee_x_last = flip_ee_x;

    if (msg->axes[cntlr_["EE_X"]] >= threshold_ && !flip_ee_x) joy_cmd.ee_x_cmd = ArmJoy::EE_X_INC;
    else if (msg->axes[cntlr_["EE_X"]] <= -threshold_ && !flip_ee_x) joy_cmd.ee_x_cmd = ArmJoy::EE_X_DEC;
    else if (msg->axes[cntlr_["EE_X"]] >= threshold_ && flip_ee_x) joy_cmd.ee_x_cmd = ArmJoy::EE_X_DEC;
    else if (msg->axes[cntlr_["EE_X"]] <= -threshold_ && flip_ee_x) joy_cmd.ee_x_cmd = ArmJoy::EE_X_INC;

    if (controller_type_ == "ps3" || controller_type_ == "ps4") {
      if (msg->buttons[cntlr_["EE_Y_INC"]] == 1) joy_cmd.ee_y_cmd = ArmJoy::EE_Y_INC;
      else if (msg->buttons[cntlr_["EE_Y_DEC"]] == 1) joy_cmd.ee_y_cmd = ArmJoy::EE_Y_DEC;
    } else if (controller_type_ == "xbox360") {
      if (msg->axes[cntlr_["EE_Y_INC"]] <= 1.0 - 2.0 * threshold_) joy_cmd.ee_y_cmd = ArmJoy::EE_Y_INC;
      else if (msg->axes[cntlr_["EE_Y_DEC"]] <= 1.0 - 2.0 * threshold_) joy_cmd.ee_y_cmd = ArmJoy::EE_Y_DEC;
    }

    if (msg->axes[cntlr_["EE_Z"]] >= threshold_) joy_cmd.ee_z_cmd = ArmJoy::EE_Z_INC;
    else if (msg->axes[cntlr_["EE_Z"]] <= -threshold_) joy_cmd.ee_z_cmd = ArmJoy::EE_Z_DEC;

    if (msg->buttons[cntlr_["FLIP_EE_ROLL"]] == 1 && !flip_ee_roll_last) flip_ee_roll = true;
    else if (msg->buttons[cntlr_["FLIP_EE_ROLL"]] == 1 && flip_ee_roll_last) flip_ee_roll = false;
    else if (msg->buttons[cntlr_["FLIP_EE_ROLL"]] == 0) flip_ee_roll_last = flip_ee_roll;

    if (msg->axes[cntlr_["EE_ROLL"]] >= threshold_ && !flip_ee_roll) joy_cmd.ee_roll_cmd = ArmJoy::EE_ROLL_CW;
    else if (msg->axes[cntlr_["EE_ROLL"]] <= -threshold_ && !flip_ee_roll) joy_cmd.ee_roll_cmd = ArmJoy::EE_ROLL_CCW;
    else if (msg->axes[cntlr_["EE_ROLL"]] >= threshold_ && flip_ee_roll) joy_cmd.ee_roll_cmd = ArmJoy::EE_ROLL_CCW;
    else if (msg->axes[cntlr_["EE_ROLL"]] <= -threshold_ && flip_ee_roll) joy_cmd.ee_roll_cmd = ArmJoy::EE_ROLL_CW;

    if (msg->axes[cntlr_["EE_PITCH"]] >= threshold_) joy_cmd.ee_pitch_cmd = ArmJoy::EE_PITCH_UP;
    else if (msg->axes[cntlr_["EE_PITCH"]] <= -threshold_) joy_cmd.ee_pitch_cmd = ArmJoy::EE_PITCH_DOWN;

    if (msg->buttons[cntlr_["WAIST_CCW"]] == 1) joy_cmd.waist_cmd = ArmJoy::WAIST_CCW;
    else if (msg->buttons[cntlr_["WAIST_CW"]] == 1) joy_cmd.waist_cmd = ArmJoy::WAIST_CW;
    if (msg->buttons[cntlr_["GRIPPER_CLOSE"]] == 1) joy_cmd.gripper_cmd = ArmJoy::GRIPPER_CLOSE;
    else if (msg->buttons[cntlr_["GRIPPER_OPEN"]] == 1) joy_cmd.gripper_cmd = ArmJoy::GRIPPER_OPEN;
    if (msg->buttons[cntlr_["HOME_POSE"]] == 1) joy_cmd.pose_cmd = ArmJoy::HOME_POSE;
    else if (msg->buttons[cntlr_["SLEEP_POSE"]] == 1) joy_cmd.pose_cmd = ArmJoy::SLEEP_POSE;

    if (controller_type_ == "ps3") {
      if (msg->buttons[cntlr_["SPEED_INC"]] == 1) joy_cmd.speed_cmd = ArmJoy::SPEED_INC;
      else if (msg->buttons[cntlr_["SPEED_DEC"]] == 1) joy_cmd.speed_cmd = ArmJoy::SPEED_DEC;
      if (msg->buttons[cntlr_["SPEED_COURSE"]] == 1) joy_cmd.speed_toggle_cmd = ArmJoy::SPEED_COURSE;
      else if (msg->buttons[cntlr_["SPEED_FINE"]] == 1) joy_cmd.speed_toggle_cmd = ArmJoy::SPEED_FINE;
    } else if (controller_type_ == "ps4" || controller_type_ == "xbox360") {
      if (msg->axes[cntlr_["SPEED"]] >= 0.9) joy_cmd.speed_cmd = ArmJoy::SPEED_INC;
      else if (msg->axes[cntlr_["SPEED"]] <= -0.9) joy_cmd.speed_cmd = ArmJoy::SPEED_DEC;
      if (msg->axes[cntlr_["SPEED_TYPE"]] >= 0.9) joy_cmd.speed_toggle_cmd = ArmJoy::SPEED_COURSE;
      else if (msg->axes[cntlr_["SPEED_TYPE"]] <= -0.9) joy_cmd.speed_toggle_cmd = ArmJoy::SPEED_FINE;
    }
    if (msg->buttons[cntlr_["GRIPPER_PWM_INC"]] == 1) joy_cmd.gripper_pwm_cmd = ArmJoy::GRIPPER_PWM_INC;
    else if (msg->buttons[cntlr_["GRIPPER_PWM_DEC"]] == 1) joy_cmd.gripper_pwm_cmd = ArmJoy::GRIPPER_PWM_DEC;

    if (!(prev_.ee_x_cmd == joy_cmd.ee_x_cmd && prev_.ee_y_cmd == joy_cmd.ee_y_cmd && prev_.ee_z_cmd == joy_cmd.ee_z_cmd &&
          prev_.ee_roll_cmd == joy_cmd.ee_roll_cmd && prev_.ee_pitch_cmd == joy_cmd.ee_pitch_cmd &&
          prev_.waist_cmd == joy_cmd.waist_cmd && prev_.gripper_cmd == joy_cmd.gripper_cmd &&
          prev_.pose_cmd == joy_cmd.pose_cmd && prev_.speed_cmd == joy_cmd.speed_cmd &&
          prev_.speed_toggle_cmd == joy_cmd.speed_toggle_cmd && prev_.gripper_pwm_cmd == joy_cmd.gripper_pwm_cmd &&
          prev_.torque_cmd == joy_cmd.torque_cmd))
      pub_->publish(joy_cmd);
    prev_ = joy_cmd;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::ArmJoy>::SharedPtr pub_;
  std::map<std::string, int> cntlr_;
  std::string controller_type_;
  double threshold_;
  interbotix_xs_msgs::msg::ArmJoy prev_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XsarmJoy>());
  rclcpp::shutdown();
  return 0;
}
