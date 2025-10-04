// sniffi_safety/src/safety_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node {
public:
  SafetyNode(): Node("sniffi_safety") {
    declare_parameter<double>("watchdog_timeout_s", 3.0);
    sub_estop_ = create_subscription<std_msgs::msg::Bool>("/estop", 1,
      [this](std_msgs::msg::Bool::SharedPtr m){ estop_ = m->data; });
    sub_heartbeat_ = create_subscription<std_msgs::msg::Bool>("/heartbeat", 10,
      [this](std_msgs::msg::Bool::SharedPtr){ last_ = now(); });
    sub_cmd_in_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg){ pending_ = *msg; });
    pub_cmd_out_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(100ms, [this]{ tick(); });
  }
private:
  void tick(){
    auto to = get_parameter("watchdog_timeout_s").as_double();
    bool wd_ok = (now() - last_).seconds() <= to;
    if (estop_ || !wd_ok) { pub_cmd_out_->publish(geometry_msgs::msg::Twist()); }
    else { pub_cmd_out_->publish(pending_); }
  }
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_, sub_heartbeat_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_in_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_out_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_{0,0,RCL_ROS_TIME}; geometry_msgs::msg::Twist pending_; bool estop_{true};
};
int main(int a,char**b){ rclcpp::init(a,b); rclcpp::spin(std::make_shared<SafetyNode>()); rclcpp::shutdown(); }
