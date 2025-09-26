#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

inline uint16_t calculate_checksum(const uint8_t* data, size_t length) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

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
    IMUNode() : Node("imu_node") {
        serialport_ = "/dev/ttyUSB0";
        baudrate_ = B115200;
        imupublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        if (!init_serial()) {
            RCLCPP_ERROR(this->get_logger(), "Can't open serial port");
            return;
        }

        timer_ = this->create_wall_timer(10ms, std::bind(&IMUNode::publish_imu_data, this));
        RCLCPP_INFO(this->get_logger(), "IMU Node started");
    }

    ~IMUNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    std::string serialport_;
    speed_t baudrate_;
    int serial_fd_ = -1;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imupublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool init_serial() {
        serial_fd_ = open(serialport_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            close(serial_fd_);
            return false;
        }

        cfsetospeed(&tty, baudrate_);
        cfsetispeed(&tty, baudrate_);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            close(serial_fd_);
            return false;
        }

        return true;
    }

    std::vector<uint8_t> read_frame() {
        std::vector<uint8_t> buffer;
        buffer.reserve(FRAMESIZE);

        uint8_t byte = 0;
        bool frame_started = false;

        while (true) {
            int n = read(serial_fd_, &byte, 1);
            if (n <= 0) {
                // No data or error
                break;
            }

            if (!frame_started) {
                if (byte == FRAMESTARTBYTE) {
                    frame_started = true;
                    buffer.clear();
                    buffer.push_back(byte);
                }
            } else {
                buffer.push_back(byte);
                if (byte == FRAMEENDBYTE) {
                    // Frame complete
                    if (buffer.size() == FRAMESIZE) {
                        std::cout << "";
                        for (auto b : buffer) {
                            std::cout << " 0x" << std::hex << (int)b;
                        }
                        std::cout << std::dec << std::endl;
                        return buffer;
                    } else {
                       // RCLCPP_WARN(this->get_logger(), "Frame size mismatch: expected %zu, got %zu", FRAMESIZE, buffer.size());
                        frame_started = false;
                        buffer.clear();
                    }
                }
            }
        }

        return {};
    }

    float bytes_to_float(const uint8_t* bytes) {
        uint32_t val = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
        float f;
        memcpy(&f, &val, sizeof(f));
        return f;
    }

    SensorData parse_frame(const std::vector<uint8_t>& frame) {
        SensorData data{};
        if (frame.size() != FRAMESIZE) {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame size: %zu", frame.size());
            return data;
        }

        uint16_t checksum_calc = calculate_checksum(&frame[1], FRAMESIZE - 4);
        uint16_t checksum_recv = (frame[FRAMESIZE - 2] << 8) | frame[FRAMESIZE - 3];

        std::cout << "Checksum calc=0x" << std::hex << checksum_calc << " recv=0x" << checksum_recv << std::dec << std::endl;

        if (checksum_calc != checksum_recv) {
            RCLCPP_ERROR(this->get_logger(), "Checksum mismatch: calc=0x%04X recv=0x%04X", checksum_calc, checksum_recv);
            return data;
        }

        size_t offset = 1;
        data.accelx = bytes_to_float(&frame[offset]); offset += 4;
        data.accely = bytes_to_float(&frame[offset]); offset += 4;
        data.accelz = bytes_to_float(&frame[offset]); offset += 4;
        data.gyrox = bytes_to_float(&frame[offset]); offset += 4;
        data.gyroy = bytes_to_float(&frame[offset]); offset += 4;
        data.gyroz = bytes_to_float(&frame[offset]); offset += 4;
        data.magx = bytes_to_float(&frame[offset]); offset += 4;
        data.magy = bytes_to_float(&frame[offset]); offset += 4;
        data.magz = bytes_to_float(&frame[offset]);

        return data;
    }

    void publish_imu_data() {
        auto frame = read_frame();
        if (frame.empty()) {
           // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No valid frame received");
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

        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        imupublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        imupublisher_->publish(imu_msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
