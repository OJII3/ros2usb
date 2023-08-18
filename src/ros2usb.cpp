#include "ros2usb.hpp"

using namespace std;
using std::placeholders::_1;

template <typename To, typename From> To bit_cast(const From &from) noexcept {
  To result;
  memcpy(&result, &from, sizeof(To));
  return result;
}

ROS2USB::ROS2USB() : Node("ros2usb") {
  constexpr auto sub_topic_name = "ros2micon";
  constexpr auto pub_topic_name = "micon2ros";

  subscription_ = this->create_subscription<ros2usb_msgs::msg::USBPacket>(
      sub_topic_name, 10, bind(&ROS2USB::topic_callback, this, _1));
  publisher_ =
      this->create_publisher<ros2usb_msgs::msg::USBPacket>(pub_topic_name, 10);
}

ROS2USB::~ROS2USB() { close(fd_); }

/**
 * @brief please call once in main function
 */
void ROS2USB::config() {
  parameterSetting();
  fd_ = openUSBSerial();
  while (rclcpp::ok()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open USB serial");
    fd_ = openUSBSerial();
    rclcpp::sleep_for(chrono::milliseconds(1000));
    if (fd_ > 0) {
      break;
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "USB serial opened");
}

/**
 * @brief use rosparam
 */
void ROS2USB::parameterSetting() {
  this->declare_parameter("device", device_default);
  this->declare_parameter("baudrate", baudrate_default);
}

/**
 * @brief open serial (termios)
 * @return int file descriptor
 */
int ROS2USB::openUSBSerial() {
  char *device_name = const_cast<char *>(device_default.c_str());
  RCLCPP_INFO_STREAM(this->get_logger(), "Trying to Open " << device_name);
  int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  fcntl(fd, F_SETFL, 0);

  // load configuration
  struct termios conf_tio;
  tcgetattr(fd, &conf_tio);

  // set baudrate
  speed_t BAUDRATE = static_cast<speed_t>(baudrate_default);
  cfsetispeed(&conf_tio, BAUDRATE);
  cfsetospeed(&conf_tio, BAUDRATE);

  // change raw mode
  cfmakeraw(&conf_tio);

  // non blocking
  conf_tio.c_cc[VMIN] = 0;
  conf_tio.c_cc[VTIME] = 0;

  // store configuration
  tcsetattr(fd, TCSANOW, &conf_tio);

  return fd;
}

/**
 * @brief subscribe packet from node and send packet to micon
 * @param msg
 */
void ROS2USB::sendToMicon(const ros2usb_msgs::msg::USBPacket::SharedPtr &msg) {
  array<byte, 41> raw_packet;
  auto header_size = header.size();
  auto id_size = sizeof(msg->id);
  auto packet_size = msg->packet.data.size();
  auto footer_size = footer.size();
  if (packet_size > 36) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Packet size is too large");
    return;
  }
  memcpy(raw_packet.data(), header.data(), header_size);
  memcpy(raw_packet.data() + header_size, &msg->id, id_size);
  memcpy(raw_packet.data() + header_size + id_size, msg->packet.data.data(),
         packet_size);
  memcpy(raw_packet.data() + header_size + id_size + packet_size, footer.data(),
         footer_size);
  int rec = write(fd_, reinterpret_cast<char *>(raw_packet.data()),
                  raw_packet.size());
  if (rec < 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot write to USB serial");
  }
}

void ROS2USB::topic_callback(const ros2usb_msgs::msg::USBPacket &msg) {
  this->sendToMicon(std::make_shared<ros2usb_msgs::msg::USBPacket>(msg));
}

/**
 * @brief receive packet from micon and publish packet to node
 * @todo multi packet support
 */
void ROS2USB::sendToNode() {
  array<std::byte, 1024> buf;
  auto len = read(fd_, buf.data(), 1024);
  if (len < 6)
    return;
  if (buf[0] != bit_cast<byte>(header[0]) ||
      buf[1] != bit_cast<byte>(header[1]))
    return;
  if (buf[len - 2] != bit_cast<byte>(footer[0]) ||
      buf[len - 1] != bit_cast<byte>(footer[1]))
    return;
  ros2usb_msgs::msg::USBPacket msg;
  // TODO: assign data
  memcpy(msg.packet.data.data(), buf.data() + 3, len - 5);
  publisher_->publish(msg);
}
