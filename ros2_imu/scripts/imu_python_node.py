#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Vector3, Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time
import serial
import struct
import numpy as np

class IMUPythonNode(Node):
    def __init__(self):
        super().__init__('imu_python_node')
        
        # Initialize publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_publisher = self.create_publisher(Temperature, 'imu/temp', 10)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize serial communication
        self.serial_port = None
        self.serial_baudrate = 921600
        self.serial_timeout = 1.0
        
        # Initialize complementary filter variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.98  # Complementary filter coefficient
        
        # Initialize timer for data publishing
        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz
        
        # Initialize serial connection
        self.init_serial_connection()
        
        self.get_logger().info('IMU Python Node started')
        self.get_logger().info('Publishing IMU data at 100Hz')
    
    def init_serial_connection(self):
        """Initialize serial connection to STM32"""
        try:
            # Try common serial ports
            serial_ports = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']
            
            for port in serial_ports:
                try:
                    self.serial_port = serial.Serial(
                        port=port,
                        baudrate=self.serial_baudrate,
                        timeout=self.serial_timeout
                    )
                    self.get_logger().info(f'Connected to serial port: {port}')
                    break
                except serial.SerialException:
                    continue
            
            if self.serial_port is None:
                self.get_logger().warn('Could not connect to serial port. Using simulated data.')
                self.serial_port = None
                
        except Exception as e:
            self.get_logger().error(f'Serial connection error: {str(e)}')
            self.serial_port = None
    
    def read_serial_data(self):
        """Read data from serial port"""
        if self.serial_port is None:
            return self.get_simulated_data()
        
        try:
            # Read line from serial port
            line = self.serial_port.readline().decode('utf-8').strip()
            
            # Parse data (assuming format: "Axyz= ax ay az | Gxyz= gx gy gz | Mxyz= mx my mz | t=time")
            if 'Axyz=' in line and 'Gxyz=' in line and 'Mxyz=' in line:
                parts = line.split('|')
                
                # Parse accelerometer data
                accel_part = parts[0].split('Axyz=')[1].strip()
                accel_values = [float(x) for x in accel_part.split()]
                
                # Parse gyroscope data
                gyro_part = parts[1].split('Gxyz=')[1].strip()
                gyro_values = [float(x) for x in gyro_part.split()]
                
                # Parse magnetometer data
                mag_part = parts[2].split('Mxyz=')[1].strip()
                mag_values = [float(x) for x in mag_part.split()]
                
                return {
                    'accel': accel_values,
                    'gyro': gyro_values,
                    'mag': mag_values,
                    'temperature': 25.0  # Default temperature
                }
            else:
                return self.get_simulated_data()
                
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {str(e)}')
            return self.get_simulated_data()
    
    def get_simulated_data(self):
        """Generate simulated sensor data for testing"""
        import random
        
        # Simulate some movement
        t = time.time()
        ax = 0.1 * math.sin(t)
        ay = 0.1 * math.cos(t)
        az = 9.81 + 0.1 * math.sin(2*t)
        
        gx = 0.01 * math.sin(0.5*t)
        gy = 0.01 * math.cos(0.5*t)
        gz = 0.01 * math.sin(0.3*t)
        
        mx = 0.0001 * math.sin(0.1*t)
        my = 0.0001 * math.cos(0.1*t)
        mz = 0.0001 * math.sin(0.05*t)
        
        return {
            'accel': [ax, ay, az],
            'gyro': [gx, gy, gz],
            'mag': [mx, my, mz],
            'temperature': 25.0 + 2 * math.sin(0.1*t)
        }
    
    def update_orientation(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """Update orientation using complementary filter"""
        # Calculate roll and pitch from accelerometer
        accel_roll = math.atan2(ay, math.sqrt(ax*ax + az*az))
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        # Integrate gyroscope data
        dt = 0.01  # 100Hz
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch
        self.yaw = self.yaw + gz * dt  # Yaw from gyroscope only
        
        # Normalize angles
        self.roll = math.fmod(self.roll + math.pi, 2*math.pi) - math.pi
        self.pitch = math.fmod(self.pitch + math.pi, 2*math.pi) - math.pi
        self.yaw = math.fmod(self.yaw + math.pi, 2*math.pi) - math.pi
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    def publish_imu_data(self):
        """Publish IMU data"""
        # Read sensor data
        data = self.read_serial_data()
        
        ax, ay, az = data['accel']
        gx, gy, gz = data['gyro']
        mx, my, mz = data['mag']
        temperature = data['temperature']
        
        # Update orientation
        self.update_orientation(ax, ay, az, gx, gy, gz, mx, my, mz)
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # Orientation (quaternion)
        quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        
        # Covariance matrices - ROS2 standard compliance
        # If no covariance data available, set first element to -1
        # For known variances, set diagonal elements (3x3 matrix, row-major order)
        
        # Linear acceleration covariance
        imu_msg.linear_acceleration_covariance[0] = 0.01   # xx
        imu_msg.linear_acceleration_covariance[1] = 0.0    # xy
        imu_msg.linear_acceleration_covariance[2] = 0.0    # xz
        imu_msg.linear_acceleration_covariance[3] = 0.0    # yx
        imu_msg.linear_acceleration_covariance[4] = 0.01   # yy
        imu_msg.linear_acceleration_covariance[5] = 0.0    # yz
        imu_msg.linear_acceleration_covariance[6] = 0.0    # zx
        imu_msg.linear_acceleration_covariance[7] = 0.0    # zy
        imu_msg.linear_acceleration_covariance[8] = 0.01   # zz
        
        # Angular velocity covariance
        imu_msg.angular_velocity_covariance[0] = 0.01      # xx
        imu_msg.angular_velocity_covariance[1] = 0.0       # xy
        imu_msg.angular_velocity_covariance[2] = 0.0       # xz
        imu_msg.angular_velocity_covariance[3] = 0.0       # yx
        imu_msg.angular_velocity_covariance[4] = 0.01      # yy
        imu_msg.angular_velocity_covariance[5] = 0.0       # yz
        imu_msg.angular_velocity_covariance[6] = 0.0       # zx
        imu_msg.angular_velocity_covariance[7] = 0.0       # zy
        imu_msg.angular_velocity_covariance[8] = 0.01      # zz
        
        # Orientation covariance
        imu_msg.orientation_covariance[0] = 0.01           # xx
        imu_msg.orientation_covariance[1] = 0.0            # xy
        imu_msg.orientation_covariance[2] = 0.0            # xz
        imu_msg.orientation_covariance[3] = 0.0            # yx
        imu_msg.orientation_covariance[4] = 0.01           # yy
        imu_msg.orientation_covariance[5] = 0.0            # yz
        imu_msg.orientation_covariance[6] = 0.0            # zx
        imu_msg.orientation_covariance[7] = 0.0            # zy
        imu_msg.orientation_covariance[8] = 0.01           # zz
        
        # Create magnetic field message
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'imu_link'
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        
        # Create temperature message
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = 'imu_link'
        temp_msg.temperature = temperature
        temp_msg.variance = 0.1
        
        # Publish messages
        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)
        self.temp_publisher.publish(temp_msg)
        
        # Publish transform
        self.publish_transform()
        
        # Log data (optional, for debugging)
        if hasattr(self, 'log_counter'):
            self.log_counter += 1
        else:
            self.log_counter = 0
            
        if self.log_counter % 100 == 0:  # Log every 1 second
            self.get_logger().info(
                f'IMU Data - Accel: [{ax:.3f}, {ay:.3f}, {az:.3f}], '
                f'Gyro: [{gx:.3f}, {gy:.3f}, {gz:.3f}], '
                f'Mag: [{mx:.3f}, {my:.3f}, {mz:.3f}]'
            )
    
    def publish_transform(self):
        """Publish transform from base_link to imu_link"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'imu_link'
        
        # Translation (assuming IMU is at origin)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # Rotation (quaternion)
        quat = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = IMUPythonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
