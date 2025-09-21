#!/usr/bin/env python3

"""
Demo script for GY87 IMU ROS2 package
Demonstrates sensor data reading and visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Vector3, Quaternion
import math
import time
import numpy as np

class IMUDemo(Node):
    def __init__(self):
        super().__init__('imu_demo')
        
        # Initialize subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        
        self.mag_subscriber = self.create_subscription(
            MagneticField,
            'imu/mag',
            self.mag_callback,
            10
        )
        
        self.temp_subscriber = self.create_subscription(
            Temperature,
            'imu/temp',
            self.temp_callback,
            10
        )
        
        # Data storage
        self.imu_data = None
        self.mag_data = None
        self.temp_data = None
        
        # Statistics
        self.data_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('IMU Demo started')
        self.get_logger().info('Waiting for sensor data...')
    
    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.imu_data = msg
        self.data_count += 1
        
        # Calculate orientation from quaternion
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Convert quaternion to Euler angles
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(2 * (qw * qy - qz * qx))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        
        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Log data every 50 messages (0.5 seconds at 100Hz)
        if self.data_count % 50 == 0:
            self.get_logger().info(
                f'IMU Data #{self.data_count}:\n'
                f'  Linear Accel: [{msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}] m/s²\n'
                f'  Angular Vel:  [{msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, {msg.angular_velocity.z:.3f}] rad/s\n'
                f'  Orientation:  [{roll_deg:.1f}°, {pitch_deg:.1f}°, {yaw_deg:.1f}°]'
            )
    
    def mag_callback(self, msg):
        """Callback for magnetometer data"""
        self.mag_data = msg
        
        # Calculate magnetic field strength
        strength = math.sqrt(
            msg.magnetic_field.x**2 + 
            msg.magnetic_field.y**2 + 
            msg.magnetic_field.z**2
        )
        
        # Log data every 50 messages
        if self.data_count % 50 == 0:
            self.get_logger().info(
                f'Magnetometer Data:\n'
                f'  Field: [{msg.magnetic_field.x:.6f}, {msg.magnetic_field.y:.6f}, {msg.magnetic_field.z:.6f}] T\n'
                f'  Strength: {strength:.6f} T'
            )
    
    def temp_callback(self, msg):
        """Callback for temperature data"""
        self.temp_data = msg
        
        # Log data every 50 messages
        if self.data_count % 50 == 0:
            self.get_logger().info(
                f'Temperature: {msg.temperature:.1f}°C (variance: {msg.variance:.3f})'
            )
    
    def print_statistics(self):
        """Print data statistics"""
        if self.data_count > 0:
            elapsed_time = time.time() - self.start_time
            data_rate = self.data_count / elapsed_time
            
            self.get_logger().info(
                f'\n=== Statistics ===\n'
                f'Total messages: {self.data_count}\n'
                f'Elapsed time: {elapsed_time:.1f}s\n'
                f'Data rate: {data_rate:.1f} Hz\n'
                f'==================\n'
            )
    
    def check_data_quality(self):
        """Check data quality and provide feedback"""
        if self.imu_data is None:
            self.get_logger().warn('No IMU data received')
            return
        
        # Check accelerometer data
        accel_magnitude = math.sqrt(
            self.imu_data.linear_acceleration.x**2 +
            self.imu_data.linear_acceleration.y**2 +
            self.imu_data.linear_acceleration.z**2
        )
        
        # Check if accelerometer magnitude is close to gravity
        gravity_error = abs(accel_magnitude - 9.81)
        if gravity_error > 0.5:
            self.get_logger().warn(f'Accelerometer magnitude error: {gravity_error:.3f} m/s²')
        else:
            self.get_logger().info(f'Accelerometer data quality: Good (error: {gravity_error:.3f} m/s²)')
        
        # Check gyroscope data
        gyro_magnitude = math.sqrt(
            self.imu_data.angular_velocity.x**2 +
            self.imu_data.angular_velocity.y**2 +
            self.imu_data.angular_velocity.z**2
        )
        
        if gyro_magnitude > 1.0:  # More than 1 rad/s
            self.get_logger().info(f'High angular velocity detected: {gyro_magnitude:.3f} rad/s')
        else:
            self.get_logger().info(f'Gyroscope data: Stable ({gyro_magnitude:.3f} rad/s)')
        
        # Check magnetometer data
        if self.mag_data is not None:
            mag_magnitude = math.sqrt(
                self.mag_data.magnetic_field.x**2 +
                self.mag_data.magnetic_field.y**2 +
                self.mag_data.magnetic_field.z**2
            )
            
            # Earth's magnetic field is typically 25-65 microTesla
            if 20e-6 < mag_magnitude < 70e-6:
                self.get_logger().info(f'Magnetometer data: Good ({mag_magnitude*1e6:.1f} μT)')
            else:
                self.get_logger().warn(f'Magnetometer magnitude unusual: {mag_magnitude*1e6:.1f} μT')

def main(args=None):
    rclpy.init(args=args)
    demo = IMUDemo()
    
    try:
        # Run for 30 seconds
        rclpy.spin_once(demo, timeout_sec=30.0)
        
        # Print statistics
        demo.print_statistics()
        demo.check_data_quality()
        
    except KeyboardInterrupt:
        demo.get_logger().info('Demo interrupted by user')
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
