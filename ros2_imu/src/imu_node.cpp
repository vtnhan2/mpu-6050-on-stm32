#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

class IMUNode : public rclcpp::Node
{
public:
    IMUNode() : Node("imu_node")
    {
        // Initialize publishers
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
        temp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temp", 10);
        
        // Initialize transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Initialize timer for data publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100Hz
            std::bind(&GY87IMUNode::publish_imu_data, this)
        );
        
        // Initialize complementary filter variables
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        alpha_ = 0.98; // Complementary filter coefficient
        
        RCLCPP_INFO(this->get_logger(), "IMU Node started");
        RCLCPP_INFO(this->get_logger(), "Publishing IMU data at 100Hz");
    }

private:
    void publish_imu_data()
    {
        // TODO: Replace with actual sensor data reading
        // For now, using simulated data for demonstration
        
        // Simulate sensor data (replace with actual I2C/UART reading)
        double ax = 0.0, ay = 0.0, az = 9.81;  // Accelerometer (m/s²)
        double gx = 0.0, gy = 0.0, gz = 0.0;   // Gyroscope (rad/s)
        double mx = 0.0, my = 0.0, mz = 0.0;   // Magnetometer (T)
        double temperature = 25.0;              // Temperature (°C)
        
        // Apply complementary filter for orientation estimation
        update_orientation(ax, ay, az, gx, gy, gz, mx, my, mz);
        
        // Create IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        
        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        
        // Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;
        
        // Orientation (quaternion)
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        
        // Covariance matrices - ROS2 standard compliance
        // If no covariance data available, set first element to -1
        // For known variances, set diagonal elements
        
        // Linear acceleration covariance (3x3 matrix, row-major order)
        imu_msg.linear_acceleration_covariance[0] = 0.01;  // xx
        imu_msg.linear_acceleration_covariance[1] = 0.0;   // xy
        imu_msg.linear_acceleration_covariance[2] = 0.0;   // xz
        imu_msg.linear_acceleration_covariance[3] = 0.0;   // yx
        imu_msg.linear_acceleration_covariance[4] = 0.01;  // yy
        imu_msg.linear_acceleration_covariance[5] = 0.0;   // yz
        imu_msg.linear_acceleration_covariance[6] = 0.0;   // zx
        imu_msg.linear_acceleration_covariance[7] = 0.0;   // zy
        imu_msg.linear_acceleration_covariance[8] = 0.01;  // zz
        
        // Angular velocity covariance (3x3 matrix, row-major order)
        imu_msg.angular_velocity_covariance[0] = 0.01;     // xx
        imu_msg.angular_velocity_covariance[1] = 0.0;      // xy
        imu_msg.angular_velocity_covariance[2] = 0.0;      // xz
        imu_msg.angular_velocity_covariance[3] = 0.0;      // yx
        imu_msg.angular_velocity_covariance[4] = 0.01;     // yy
        imu_msg.angular_velocity_covariance[5] = 0.0;      // yz
        imu_msg.angular_velocity_covariance[6] = 0.0;      // zx
        imu_msg.angular_velocity_covariance[7] = 0.0;      // zy
        imu_msg.angular_velocity_covariance[8] = 0.01;     // zz
        
        // Orientation covariance (3x3 matrix, row-major order)
        imu_msg.orientation_covariance[0] = 0.01;          // xx
        imu_msg.orientation_covariance[1] = 0.0;           // xy
        imu_msg.orientation_covariance[2] = 0.0;           // xz
        imu_msg.orientation_covariance[3] = 0.0;           // yx
        imu_msg.orientation_covariance[4] = 0.01;          // yy
        imu_msg.orientation_covariance[5] = 0.0;           // yz
        imu_msg.orientation_covariance[6] = 0.0;           // zx
        imu_msg.orientation_covariance[7] = 0.0;           // zy
        imu_msg.orientation_covariance[8] = 0.01;          // zz
        
        // Create magnetic field message
        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header.stamp = this->now();
        mag_msg.header.frame_id = "imu_link";
        mag_msg.magnetic_field.x = mx;
        mag_msg.magnetic_field.y = my;
        mag_msg.magnetic_field.z = mz;
        
        // Create temperature message
        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.header.stamp = this->now();
        temp_msg.header.frame_id = "imu_link";
        temp_msg.temperature = temperature;
        temp_msg.variance = 0.1;
        
        // Publish messages
        imu_publisher_->publish(imu_msg);
        mag_publisher_->publish(mag_msg);
        temp_publisher_->publish(temp_msg);
        
        // Publish transform
        publish_transform();
        
        // Log data (optional, for debugging)
        static int counter = 0;
        if (++counter % 100 == 0) { // Log every 1 second
            RCLCPP_INFO(this->get_logger(), 
                "IMU Data - Accel: [%.3f, %.3f, %.3f], Gyro: [%.3f, %.3f, %.3f], Mag: [%.3f, %.3f, %.3f]",
                ax, ay, az, gx, gy, gz, mx, my, mz);
        }
    }
    
    void update_orientation(double ax, double ay, double az, 
                           double gx, double gy, double gz,
                           double mx, double my, double mz)
    {
        // Complementary filter for orientation estimation
        // This is a simplified version - in practice, you'd use more sophisticated algorithms
        
        // Calculate roll and pitch from accelerometer
        double accel_roll = atan2(ay, sqrt(ax*ax + az*az));
        double accel_pitch = atan2(-ax, sqrt(ay*ay + az*az));
        
        // Integrate gyroscope data
        double dt = 0.01; // 100Hz
        roll_ = alpha_ * (roll_ + gx * dt) + (1 - alpha_) * accel_roll;
        pitch_ = alpha_ * (pitch_ + gy * dt) + (1 - alpha_) * accel_pitch;
        yaw_ = yaw_ + gz * dt; // Yaw from gyroscope only (no magnetometer in this simple version)
        
        // Normalize angles
        roll_ = fmod(roll_ + M_PI, 2*M_PI) - M_PI;
        pitch_ = fmod(pitch_ + M_PI, 2*M_PI) - M_PI;
        yaw_ = fmod(yaw_ + M_PI, 2*M_PI) - M_PI;
    }
    
    void publish_transform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "imu_link";
        
        // Translation (assuming IMU is at origin)
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        
        // Rotation (quaternion)
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher_;
    
    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Orientation variables
    double roll_, pitch_, yaw_;
    double alpha_; // Complementary filter coefficient
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}
