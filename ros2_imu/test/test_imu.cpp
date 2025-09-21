#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>

class IMUTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node = rclcpp::Node::make_shared("test_imu_node");
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node;
};

TEST_F(GY87IMUTest, TestIMUMessageCreation)
{
    // Test IMU message creation
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = node->now();
    imu_msg.header.frame_id = "imu_link";
    
    // Test linear acceleration
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;
    
    // Test angular velocity
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    
    // Test orientation
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    
    // Verify message fields
    EXPECT_EQ(imu_msg.header.frame_id, "imu_link");
    EXPECT_EQ(imu_msg.linear_acceleration.z, 9.81);
    EXPECT_EQ(imu_msg.orientation.w, 1.0);
}

TEST_F(GY87IMUTest, TestMagneticFieldMessageCreation)
{
    // Test magnetic field message creation
    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp = node->now();
    mag_msg.header.frame_id = "imu_link";
    
    // Test magnetic field
    mag_msg.magnetic_field.x = 0.0001;
    mag_msg.magnetic_field.y = 0.0001;
    mag_msg.magnetic_field.z = 0.0001;
    
    // Verify message fields
    EXPECT_EQ(mag_msg.header.frame_id, "imu_link");
    EXPECT_EQ(mag_msg.magnetic_field.x, 0.0001);
    EXPECT_EQ(mag_msg.magnetic_field.y, 0.0001);
    EXPECT_EQ(mag_msg.magnetic_field.z, 0.0001);
}

TEST_F(GY87IMUTest, TestTemperatureMessageCreation)
{
    // Test temperature message creation
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = node->now();
    temp_msg.header.frame_id = "imu_link";
    temp_msg.temperature = 25.0;
    temp_msg.variance = 0.1;
    
    // Verify message fields
    EXPECT_EQ(temp_msg.header.frame_id, "imu_link");
    EXPECT_EQ(temp_msg.temperature, 25.0);
    EXPECT_EQ(temp_msg.variance, 0.1);
}

TEST_F(GY87IMUTest, TestQuaternionConversion)
{
    // Test Euler to quaternion conversion
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    
    // Convert to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;
    
    // Verify quaternion normalization
    double norm = sqrt(x*x + y*y + z*z + w*w);
    EXPECT_NEAR(norm, 1.0, 1e-6);
    
    // Verify identity quaternion
    EXPECT_NEAR(x, 0.0, 1e-6);
    EXPECT_NEAR(y, 0.0, 1e-6);
    EXPECT_NEAR(z, 0.0, 1e-6);
    EXPECT_NEAR(w, 1.0, 1e-6);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
