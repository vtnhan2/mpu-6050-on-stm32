#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

using namespace std::chrono_literals;

const uint8_t FRAMESTARTBYTE = 0xAA;
const uint8_t FRAMEENDBYTE = 0xFF;
const size_t FRAMESIZE = 40;

struct SensorData {
  float accelx, accely, accelz;
  float gyrox, gyroy, gyroz;
  float magx, magy, magz;
};

class IMUNode : public rclcpp::Node {
public:
  IMUNode()
  : Node("imu_node") {
    serialport_ = "/dev/ttyUSB0";
    baudrate_ = B115200;

    imupublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    if (!init_serial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication");
      return;
    }

    timer_ = this->create_wall_timer(
      10ms, std::bind(&IMUNode::publish_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "IMU Node started");
  }

  ~IMUNode() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  std::string serialport_;
  speed_t baudrate_;
  int serial_fd_ = -1;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imupublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool init_serial() {
    serial_fd_ = open(serialport_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serialport_.c_str());
      return false;
    }

    termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr error");
      close(serial_fd_);
      return false;
    }

    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN] = 0;                             // read doesn't block
    tty.c_cc[VTIME] = 10;                           // 1 second read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr error");
      close(serial_fd_);
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Serial communication initialized (port: %s, baudrate: 115200)", serialport_.c_str());
    return true;
  }

  std::vector<uint8_t> read_frame() {
    std::vector<uint8_t> buffer(FRAMESIZE);
    size_t offset = 0;
    while (offset < FRAMESIZE) {
      int n = read(serial_fd_, &buffer[offset], FRAMESIZE - offset);
      if (n > 0) {
        offset += n;
      } else {
        break; // timeout or read error
      }
    }
    if (offset == FRAMESIZE && buffer[0] == FRAMESTARTBYTE && buffer[FRAMESIZE-1] == FRAMEENDBYTE) {
      return buffer;
    }
    return std::vector<uint8_t>(); // invalid frame
  }

  uint16_t calculate_checksum(const std::vector<uint8_t>& frame) {
    uint16_t checksum = 0;
    for (size_t i = 1; i < FRAMESIZE - 3; i++) { // exclude start byte & last 3 bytes (checksum + end byte)
      checksum += frame[i];
    }
    return checksum;
  }

  float bytes_to_float(const uint8_t* bytes) {
    uint32_t val = (uint32_t)bytes[0] | ((uint32_t)bytes[1]<<8) | ((uint32_t)bytes[2]<<16) | ((uint32_t)bytes[3]<<24);
    float ret;
    memcpy(&ret, &val, sizeof(ret));
    return ret;
  }

  SensorData parse_frame(const std::vector<uint8_t>& frame) {
    SensorData data{};
    if (frame.size() != FRAMESIZE) {
      RCLCPP_ERROR(this->get_logger(), "Invalid frame size: %zu", frame.size());
      return data;
    }
    auto checksum_calc = calculate_checksum(frame);
    uint16_t checksum_recv = ((uint16_t)frame[FRAMESIZE-3] << 8) | frame[FRAMESIZE-2];
    if (checksum_calc != checksum_recv) {
      RCLCPP_ERROR(this->get_logger(), "Checksum mismatch: calc=0x%04X recv=0x%04X", checksum_calc, checksum_recv);
      return data;
    }

    size_t offset = 1; // skip start byte
    data.accelx = bytes_to_float(&frame[offset]); offset += 4;
    data.accely = bytes_to_float(&frame[offset]); offset += 4;
    data.accelz = bytes_to_float(&frame[offset]); offset += 4;
    data.gyrox = bytes_to_float(&frame[offset]); offset += 4;
    data.gyroy = bytes_to_float(&frame[offset]); offset += 4;
    data.gyroz = bytes_to_float(&frame[offset]); offset += 4;
    data.magx = bytes_to_float(&frame[offset]); offset += 4;
    data.magy = bytes_to_float(&frame[offset]); offset += 4;
    data.magz = bytes_to_float(&frame[offset]); offset += 4;

    return data;
  }

  void publish_imu_data() {
    auto frame = read_frame();
    if (frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No valid frame received");
      return;
    }

    auto data = parse_frame(frame);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = data.accelx;
    imu_msg.linear_acceleration.y = data.accely;
    imu_msg.linear_acceleration.z = data.accelz;

    imu_msg.angular_velocity.x = data.gyrox;
    imu_msg.angular_velocity.y = data.gyroy;
    imu_msg.angular_velocity.z = data.gyroz;

    // Orientation is placeholder - real implementation should use sensor fusion
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    imupublisher_->publish(imu_msg);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
