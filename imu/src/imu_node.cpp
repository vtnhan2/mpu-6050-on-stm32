#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>

// STM32 Frame Protocol Constants
const uint8_t FRAME_START_BYTE = 0xAA;
const uint8_t FRAME_END_BYTE = 0xFF;
const size_t FRAME_SIZE = 40;  // 1 + 36 + 2 + 1 = 40 bytes
const size_t DATA_SIZE = 36;   // 9 floats * 4 bytes = 36 bytes

class IMUNode : public rclcpp::Node
{
public:
    IMUNode() : Node("imu_node")
    {
        // Declare parameters
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("publish_rate", 100.0);
        
        // Get parameters
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        
        // Create publisher
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        
        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&IMUNode::publish_imu_data, this));
        
        // Initialize serial communication
        if (!init_serial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "IMU Node started");
        RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate_);
    }

private:
    void publish_imu_data()
    {
        // Read frame from serial
        std::vector<uint8_t> frame = read_frame();
        
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                               "No valid frame received");
            return;
        }
        
        // Parse frame data
        SensorData data = parse_frame(frame);
        
        // Create IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = frame_id_;
        
        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = data.accel_x;
        imu_msg.linear_acceleration.y = data.accel_y;
        imu_msg.linear_acceleration.z = data.accel_z;
        
        // Angular velocity (rad/s)
        imu_msg.angular_velocity.x = data.gyro_x;
        imu_msg.angular_velocity.y = data.gyro_y;
        imu_msg.angular_velocity.z = data.gyro_z;
        
        // Orientation (quaternion) - placeholder for now
        // In a real implementation, you would calculate orientation from
        // accelerometer and magnetometer data using sensor fusion
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;
        
        // Covariance matrices (set to unknown for now)
        imu_msg.linear_acceleration_covariance[0] = -1.0;
        imu_msg.angular_velocity_covariance[0] = -1.0;
        imu_msg.orientation_covariance[0] = -1.0;
        
        // Publish message
        imu_publisher_->publish(imu_msg);
        
        // Log data periodically
        static int log_counter = 0;
        if (++log_counter >= 100) {  // Log every 100 messages
            RCLCPP_INFO(this->get_logger(), 
                       "IMU Data - Accel: [%.4f, %.4f, %.4f] m/s² | "
                       "Gyro: [%.4f, %.4f, %.4f] rad/s | "
                       "Mag: [%.4f, %.4f, %.4f] µT",
                       data.accel_x, data.accel_y, data.accel_z,
                       data.gyro_x, data.gyro_y, data.gyro_z,
                       data.mag_x, data.mag_y, data.mag_z);
            log_counter = 0;
        }
    }
    
    bool init_serial()
    {
        // Note: This is a simplified implementation
        // In a real implementation, you would use a proper serial library
        // like boost::asio or similar for cross-platform serial communication
        
        RCLCPP_INFO(this->get_logger(), "Serial communication initialized (simulated)");
        return true;
    }
    
    std::vector<uint8_t> read_frame()
    {
        // Simulate reading a frame from serial
        // In a real implementation, you would:
        // 1. Read bytes from serial port
        // 2. Look for start byte (0xAA)
        // 3. Read 39 more bytes
        // 4. Verify end byte (0xFF)
        // 5. Calculate and verify checksum
        
        // For now, return empty vector to indicate no data
        // This would be replaced with actual serial reading logic
        return std::vector<uint8_t>();
    }
    
    SensorData parse_frame(const std::vector<uint8_t>& frame)
    {
        SensorData data;
        
        if (frame.size() != FRAME_SIZE) {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame size: %zu", frame.size());
            return data;
        }
        
        // Verify start and end bytes
        if (frame[0] != FRAME_START_BYTE || frame[FRAME_SIZE-1] != FRAME_END_BYTE) {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame format");
            return data;
        }
        
        // Verify checksum
        uint16_t calculated_checksum = calculate_checksum(frame);
        uint16_t received_checksum = (frame[FRAME_SIZE-3] << 8) | frame[FRAME_SIZE-2];
        
        if (calculated_checksum != received_checksum) {
            RCLCPP_ERROR(this->get_logger(), "Checksum mismatch: calculated=0x%04X, received=0x%04X",
                        calculated_checksum, received_checksum);
            return data;
        }
        
        // Parse data payload (36 bytes = 9 floats)
        size_t offset = 1;  // Skip start byte
        
        // Accelerometer data (m/s²)
        data.accel_x = bytes_to_float(&frame[offset]); offset += 4;
        data.accel_y = bytes_to_float(&frame[offset]); offset += 4;
        data.accel_z = bytes_to_float(&frame[offset]); offset += 4;
        
        // Gyroscope data (rad/s)
        data.gyro_x = bytes_to_float(&frame[offset]); offset += 4;
        data.gyro_y = bytes_to_float(&frame[offset]); offset += 4;
        data.gyro_z = bytes_to_float(&frame[offset]); offset += 4;
        
        // Magnetometer data (µT)
        data.mag_x = bytes_to_float(&frame[offset]); offset += 4;
        data.mag_y = bytes_to_float(&frame[offset]); offset += 4;
        data.mag_z = bytes_to_float(&frame[offset]); offset += 4;
        
        return data;
    }
    
    uint16_t calculate_checksum(const std::vector<uint8_t>& frame)
    {
        uint16_t checksum = 0;
        // Calculate checksum over data payload only (exclude start byte)
        for (size_t i = 1; i < FRAME_SIZE - 3; i++) {  // Skip start byte and checksum bytes
            checksum += frame[i];
        }
        return checksum;
    }
    
    float bytes_to_float(const uint8_t* bytes)
    {
        // Convert 4 bytes to float (little-endian)
        uint32_t bits = (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
        return *reinterpret_cast<const float*>(&bits);
    }
    
    struct SensorData {
        float accel_x, accel_y, accel_z;  // m/s²
        float gyro_x, gyro_y, gyro_z;     // rad/s
        float mag_x, mag_y, mag_z;        // µT
    };
    
    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    std::string serial_port_;
    int baud_rate_;
    std::string frame_id_;
    double publish_rate_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
