#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"                                       // CHANGE
#include "tutorial_interfaces/msg/ddstype.hpp"                                       // CHANGE

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Ddstype>(    // CHANGE
      "/MQTT/topic1", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local(), std::bind(&MinimalSubscriber::topic_callback, this, _1));    // CHANGE
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Ddstype & msg) const  // CHANGE
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Get int32_test: '" << msg.int32_test << "'");     // CHANGE
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Ddstype>::SharedPtr subscription_;  // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
