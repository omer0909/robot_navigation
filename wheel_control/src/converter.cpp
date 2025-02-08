#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelConverter : public rclcpp::Node {
 public:
  CmdVelConverter() : Node("unstamped_listener") {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_nav", 10,
        std::bind(&CmdVelConverter::cmd_vel_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10);
  }

 private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    auto stamped_msg = geometry_msgs::msg::TwistStamped();
    stamped_msg.header.stamp = this->get_clock()->now();
    stamped_msg.header.frame_id = "base_link";
    stamped_msg.twist = *msg;

    publisher_->publish(stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
