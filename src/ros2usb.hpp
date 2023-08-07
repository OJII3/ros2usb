#pragma once

#include <bits/stdc++.h>
#include <fcntl.h>
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
  constexpr static const std::array<char, 2> header = {'S', 'S'};
  constexpr static const std::array<char, 2> footer = {'E', 'E'};
  int fd_;
  /* ros::NodeHandle nh_; */
  /* ros::NodeHandle nh_local_; */
  std::string device_ = "/dev/ttyUSB0";
  int baudrate_ = B115200;
  /* ros::Publisher pub_; */
  /* ros::Subscriber sub_; */
};
