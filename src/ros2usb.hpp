#pragma once

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
  void sendToMicon(const std_msgs::msg::ByteMultiArray::SharedPtr &msg);
  void sendToNode();

private:
  void topic_callback(const std_msgs::msg::ByteMultiArray &msg);
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  constexpr static const std::array<char, 2> header = {'S', 'S'};
  constexpr static const std::array<char, 2> footer = {'E', 'E'};
  int fd_;
  std::string device_default = "/dev/ttyACM0";
  int baudrate_default = B115200;
};
