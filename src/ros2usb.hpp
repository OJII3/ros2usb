#pragma once

#include <cmath>
#include <cstddef>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <string>
#include <string_view>
#include <termios.h>
#include <unistd.h>
/* #include <usb_server/USBPacket.h> */
/* #include "ros2usb/msg/USBPacket.hpp" */

class ROS2USB {
public:
  /* ROS2USB(ros::NodeHandle &nh, ros::NodeHandle &nh_local); */
  ~ROS2USB();

  void config();
  void parameterSetting();
  int openUSBSerial();
  /* void sendToMicon(const usb_server::USBPacket::ConstPtr &msg); */
  void sendToNode();

private:
  static const std::array<char, 2> header;
  static const std::array<char, 2> footer;
  int fd_;
  /* ros::NodeHandle nh_; */
  /* ros::NodeHandle nh_local_; */
  std::string device_;
  int baudrate_;
  /* ros::Publisher pub_; */
  /* ros::Subscriber sub_; */
};
