#include "ros2usb.hpp"

using namespace std;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ros2usb_node = make_shared<ROS2USB>();

  try {
    RCLCPP_INFO(ros2usb_node->get_logger(), "Initializing node...");
    ros2usb_node->config();
    RCLCPP_INFO(ros2usb_node->get_logger(), "Node initialized");
    ros2usb_node->sendToMicon(make_shared<std_msgs::msg::ByteMultiArray>());
    rclcpp::spin(ros2usb_node);
  } catch (const char *s) {
    RCLCPP_FATAL_STREAM(ros2usb_node->get_logger(), "" << s);
  } catch (...) {
    RCLCPP_FATAL_STREAM(ros2usb_node->get_logger(), "Unexpected error");
  }

  return 0;
}
