#pragma once

#include "ros2usb_msgs/msg/usb_packet.hpp"

#include <bits/stdc++.h>
#include <fcntl.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <termios.h>

class ROS2USB : public rclcpp::Node {
public:
  ROS2USB();
  ~ROS2USB();
  void config();
  void parameterSetting();
  int openUSBSerial();
  void sendToMicon(const ros2usb_msgs::msg::USBPacket::SharedPtr &msg);
  void sendToNode();

private:
  void topic_callback(const ros2usb_msgs::msg::USBPacket &msg);
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  int fd_;
  const std::string device_default = "/dev/ttyACM0";
  const int baudrate_default = B115200;
  const int packet_size = 64;
  const std::array<char, 2> header = {'S', 'S'};
  const std::array<char, 2> footer = {'E', 'E'};
};
