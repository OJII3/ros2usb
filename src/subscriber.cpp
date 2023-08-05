#pragma once
/* #include "ros2usb/msg/USBPacket.hpp" */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/* class MinimalSubscriber : public rclcpp::Node { */
/* public: */
/*   MinimalSubscriber() : Node("minimal_subscriber") { */
/*     subscription_ = */
/*         this->create_subscription<ros2usb::msg::USBPacket>( // CHANGE */
/*             "topic", 10, */
/*             std::bind(&MinimalSubscriber::topic_callback, this, _1)); */
/*   } */

/* private: */
/*   void topic_callback(const ros2usb::msg::Num::SharedPtr msg) const // CHANGE
 */
/*   { */
/*     RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num); // CHANGE */
/*   } */
/*   rclcpp::Subscription<ros2usb::msg::Num>::SharedPtr subscription_; // CHANGE
 */
/* }; */

class Subscriber : public rclcpp::Node {
public:
  Subscriber() : Node("minimal_subscriber") {
    auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
