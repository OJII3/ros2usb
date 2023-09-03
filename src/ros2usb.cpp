#include "ros2usb.hpp"

ROS2USB::ROS2USB() : Node("ros2usb"), fd_(-1) {
  constexpr auto sub_topic_name = "ros2micon";
  constexpr auto pub_topic_name = "micon2ros";

  subscription_ = this->create_subscription<ros2usb_msgs::msg::USBPacket>(
      sub_topic_name, qos,
      bind(&ROS2USB::topicCallback, this, std::placeholders::_1));
  publisher_ =
      this->create_publisher<ros2usb_msgs::msg::USBPacket>(pub_topic_name, qos);
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
    rclcpp::sleep_for(std::chrono::seconds(1));
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
  this->declare_parameter("device", device_default_);
  this->declare_parameter("baudrate", baudrate_default);
}

/**
 * @brief open serial (termios)
 * @return int file descriptor
 */
int ROS2USB::openUSBSerial() {
  char *device_name = device_default_.data();
  RCLCPP_INFO_STREAM(this->get_logger(), "Trying to Open " << device_name);
  int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  fcntl(fd, F_SETFL, 0);

  // load configuration
  struct termios conf_tio {};
  tcgetattr(fd, &conf_tio);

  // set baudrate
  auto baudrate = static_cast<speed_t>(baudrate_default);
  cfsetispeed(&conf_tio, baudrate);
  cfsetospeed(&conf_tio, baudrate);

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
  std::vector<uint8_t> raw_packet(max_packet_size);
  auto len = msg->packet.data.size();
  if (len >
      max_packet_size - footer.size() - header.size() - sizeof(msg->id.data)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Packet size is too large");
    return;
  }
  raw_packet[0] = header[0];
  raw_packet[1] = header[1];
  raw_packet[2] = msg->id.data;
  copy(msg->packet.data.begin(), msg->packet.data.end(),
       raw_packet.begin() + 3);
  raw_packet[len + 3] = footer[0];
  raw_packet[len + 4] = footer[1];

  int rec = write(fd_, raw_packet.data(), raw_packet.size());
  if (rec < 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot write to USB serial");
  }
}

void ROS2USB::topicCallback(const ros2usb_msgs::msg::USBPacket &msg) {
  this->sendToMicon(std::make_shared<ros2usb_msgs::msg::USBPacket>(msg));
}

/**
 * @brief receive packet from micon and publish packet to node
 * @todo multi packet support
 */
void ROS2USB::sendToNode() {
  std::vector<uint8_t> buf(max_packet_size);
  auto len = read(fd_, buf.data(), max_packet_size);
  if (len > header.size() + footer.size() && buf[0] == header[0] &&
      buf[1] != header[1] && buf[len - 2] == footer[0] &&
      buf[len - 1] == footer[1]) {
    ros2usb_msgs::msg::USBPacket msg;
    msg.id.data = buf[2];
    copy(buf.begin() + 3, buf.end() - 2, msg.packet.data.begin());
    publisher_->publish(msg);
  }
}
