#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sniffi_msgs/msg/battery_status.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PowerNode : public rclcpp::Node {
public:
  PowerNode() : Node("sniffi_power") {
    declare_parameter<double>("low_battery_soc", 20.0);   // percent
    declare_parameter<double>("publish_hz", 1.0);

    pub_batt_ = create_publisher<sniffi_msgs::msg::BatteryStatus>("/battery_status", 10);
    pub_return_to_dock_ = create_publisher<std_msgs::msg::Bool>("/return_to_dock", 1);

    double hz = get_parameter("publish_hz").as_double();
    timer_ = create_wall_timer(
      std::chrono::milliseconds((int)(1000.0 / std::max(0.1, hz))),
      std::bind(&PowerNode::onTimer, this));
  }

private:
  void onTimer(){
    auto msg = sniffi_msgs::msg::BatteryStatus();
    msg.stamp = now();
    // TODO: replace with real sensor/Arduino readings
    static double soc = 100.0;
    soc -= 0.05; if (soc < 0) soc = 100.0;
    msg.state_of_charge = soc;
    msg.voltage = 24.0;
    msg.current = 1.2;
    msg.temperature = 30.0;
    msg.external_power = false;
    msg.bms_ok = true;
    pub_batt_->publish(msg);

    // Low-battery trigger
    double low = get_parameter("low_battery_soc").as_double();
    if (msg.state_of_charge <= low) {
      std_msgs::msg::Bool cmd;
      cmd.data = true;
      pub_return_to_dock_->publish(cmd);
    }
  }

  rclcpp::Publisher<sniffi_msgs::msg::BatteryStatus>::SharedPtr pub_batt_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_return_to_dock_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PowerNode>());
  rclcpp::shutdown();
  return 0;
}
