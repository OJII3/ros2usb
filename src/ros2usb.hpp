#pragma once

#include <fcntl.h>
#include <termios.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <string>
#include <vector>

#include "ros2usb_msgs/msg/usb_packet.hpp"

class ROS2USB : public rclcpp::Node {
 public:
  ROS2USB();
  ROS2USB(const ROS2USB &) = delete;
  ROS2USB(ROS2USB &&) = delete;
  ROS2USB &operator=(const ROS2USB &) = delete;
  ROS2USB &operator=(ROS2USB &&) = delete;
  ~ROS2USB() override;
  void config();
  void parameterSetting();
  int openUSBSerial();
  void sendToMicon(const ros2usb_msgs::msg::USBPacket::SharedPtr &msg);
  void sendToNode();

 private:
  static constexpr int baudrate_default = B115200;
  static constexpr int qos = 1024;
  static constexpr int max_packet_size = 45;
  static constexpr std::array<uint8_t, 2> header = {'S', 'S'};
  static constexpr std::array<uint8_t, 2> footer = {'E', 'E'};

  std::string device_default_ = "/dev/ttyACM0";
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  int fd_;

  void topicCallback(const ros2usb_msgs::msg::USBPacket &msg);
};
