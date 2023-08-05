#include "ros2usb.hpp"
#include "subscriber.cpp"

#include <bits/stdc++.h>

using namespace std;

/* void ROS2USB::sendToNode() { */
/*   std::array<std::byte, 1024> buf; */
/*   auto len = read(fd_, buf.data(), 1024); */
/*   if (len < 6) */
/*     return; */
/*   if (buf[0] != bit_cast<std::byte>(header[0]) || */
/*       buf[1] != bit_cast<std::byte>(header[1])) */
/*     return; */
/*   if (buf[len - 2] != bit_cast<std::byte>(footer[0]) || */
/*       buf[len - 1] != bit_cast<std::byte>(footer[1])) */
/*     return; */
/*   usb_server::USBPacket msg; */
/*   msg.id.data = bit_cast<unsigned char>(buf[2]); */
/*   msg.packet.data.resize(len - 5); */
/*   std::memcpy(msg.packet.data.data(), buf.data() + 3, len - 5); */
/*   pub_.publish(msg); */
/* } */

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
