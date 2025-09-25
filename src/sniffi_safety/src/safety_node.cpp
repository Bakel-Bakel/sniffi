#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node {
public:
  SafetyNode() : Node("sniffi_safety") {
    declare_parameter<double>("watchdog_timeout_s", 3.0);

    sub_estop_ = create_subscription<std_msgs::msg::Bool>("/estop", 1,
      [this](const std_msgs::msg::Bool::SharedPtr m){
        estop_ = m->data;
        if (estop_) RCLCPP_WARN(get_logger(), "ESTOP asserted");
        else RCLCPP_INFO(get_logger(), "ESTOP cleared");
      });

    sub_heartbeat_ = create_subscription<std_msgs::msg::Bool>("/heartbeat", 10,
      [this](const std_msgs::msg::Bool::SharedPtr){
        last_heartbeat_ = now();
      });

    sub_cmd_in_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        pending_cmd_ = *msg;
      });

    pub_cmd_out_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(100ms, std::bind(&SafetyNode::maybePublish, this));
  }

private:
  void maybePublish(){
    double to = get_parameter("watchdog_timeout_s").as_double();
    bool wd_ok = ((now() - last_heartbeat_).seconds() <= to);

    if (estop_ || !wd_ok) {
      geometry_msgs::msg::Twist stop;
      pub_cmd_out_->publish(stop);
      return;
    }
    pub_cmd_out_->publish(pending_cmd_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_out_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_in_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_, sub_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_heartbeat_{0,0,RCL_ROS_TIME};
  geometry_msgs::msg::Twist pending_cmd_;
  bool estop_{true}; // start in ESTOP
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
