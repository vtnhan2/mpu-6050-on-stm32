#!/usr/bin/env python3

"""
Validation script for sensor_msgs/Imu compliance
Checks if IMU messages follow ROS2 standard specifications
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion
import math
import numpy as np

class IMUMessageValidator(Node):
    def __init__(self):
        super().__init__('imu_message_validator')
        
        # Subscribe to IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/data',
            self.validate_imu_message,
            10
        )
        
        self.validation_count = 0
        self.error_count = 0
        
        self.get_logger().info('IMU Message Validator started')
        self.get_logger().info('Monitoring /imu/data topic for compliance...')
    
    def validate_imu_message(self, msg):
        """Validate IMU message for ROS2 standard compliance"""
        self.validation_count += 1
        errors = []
        warnings = []
        
        # 1. Check header
        if not msg.header.stamp.sec and not msg.header.stamp.nanosec:
            errors.append("Header timestamp is zero")
        
        if not msg.header.frame_id:
            errors.append("Header frame_id is empty")
        elif msg.header.frame_id != "imu_link":
            warnings.append(f"Frame ID is '{msg.header.frame_id}', expected 'imu_link'")
        
        # 2. Check quaternion normalization
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        quat_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if abs(quat_norm - 1.0) > 0.01:
            errors.append(f"Quaternion not normalized: norm = {quat_norm:.6f}")
        
        # 3. Check units
        # Linear acceleration should be in m/s²
        accel_magnitude = math.sqrt(
            msg.linear_acceleration.x**2 + 
            msg.linear_acceleration.y**2 + 
            msg.linear_acceleration.z**2
        )
        
        # Check if acceleration magnitude is reasonable (should be close to gravity when stationary)
        if 8.0 < accel_magnitude < 12.0:
            pass  # Reasonable range
        elif accel_magnitude < 0.1:
            warnings.append(f"Very low acceleration magnitude: {accel_magnitude:.3f} m/s²")
        elif accel_magnitude > 50.0:
            warnings.append(f"Very high acceleration magnitude: {accel_magnitude:.3f} m/s²")
        
        # Angular velocity should be in rad/s
        gyro_magnitude = math.sqrt(
            msg.angular_velocity.x**2 + 
            msg.angular_velocity.y**2 + 
            msg.angular_velocity.z**2
        )
        
        if gyro_magnitude > 10.0:  # More than 10 rad/s
            warnings.append(f"High angular velocity: {gyro_magnitude:.3f} rad/s")
        
        # 4. Check covariance matrices
        # Linear acceleration covariance
        accel_cov = np.array(msg.linear_acceleration_covariance).reshape(3, 3)
        if not self.is_valid_covariance_matrix(accel_cov):
            errors.append("Invalid linear acceleration covariance matrix")
        
        # Angular velocity covariance
        gyro_cov = np.array(msg.angular_velocity_covariance).reshape(3, 3)
        if not self.is_valid_covariance_matrix(gyro_cov):
            errors.append("Invalid angular velocity covariance matrix")
        
        # Orientation covariance
        orient_cov = np.array(msg.orientation_covariance).reshape(3, 3)
        if not self.is_valid_covariance_matrix(orient_cov):
            errors.append("Invalid orientation covariance matrix")
        
        # 5. Check for NaN or infinite values
        if math.isnan(qx) or math.isnan(qy) or math.isnan(qz) or math.isnan(qw):
            errors.append("NaN values in quaternion")
        
        if math.isnan(msg.linear_acceleration.x) or math.isnan(msg.linear_acceleration.y) or math.isnan(msg.linear_acceleration.z):
            errors.append("NaN values in linear acceleration")
        
        if math.isnan(msg.angular_velocity.x) or math.isnan(msg.angular_velocity.y) or math.isnan(msg.angular_velocity.z):
            errors.append("NaN values in angular velocity")
        
        # 6. Check for infinite values
        if math.isinf(qx) or math.isinf(qy) or math.isinf(qz) or math.isinf(qw):
            errors.append("Infinite values in quaternion")
        
        if math.isinf(msg.linear_acceleration.x) or math.isinf(msg.linear_acceleration.y) or math.isinf(msg.linear_acceleration.z):
            errors.append("Infinite values in linear acceleration")
        
        if math.isinf(msg.angular_velocity.x) or math.isinf(msg.angular_velocity.y) or math.isinf(msg.angular_velocity.z):
            errors.append("Infinite values in angular velocity")
        
        # Report results
        if errors:
            self.error_count += 1
            self.get_logger().error(f"Message #{self.validation_count} - ERRORS:")
            for error in errors:
                self.get_logger().error(f"  - {error}")
        
        if warnings:
            self.get_logger().warn(f"Message #{self.validation_count} - WARNINGS:")
            for warning in warnings:
                self.get_logger().warn(f"  - {warning}")
        
        # Log success every 100 messages
        if self.validation_count % 100 == 0 and not errors:
            self.get_logger().info(f"Message #{self.validation_count} - OK (Errors: {self.error_count})")
        
        # Log statistics every 500 messages
        if self.validation_count % 500 == 0:
            error_rate = (self.error_count / self.validation_count) * 100
            self.get_logger().info(f"Statistics: {self.validation_count} messages, {self.error_count} errors ({error_rate:.1f}% error rate)")
    
    def is_valid_covariance_matrix(self, cov_matrix):
        """Check if covariance matrix is valid"""
        try:
            # Check if matrix is symmetric
            if not np.allclose(cov_matrix, cov_matrix.T, atol=1e-10):
                return False
            
            # Check if matrix is positive semi-definite
            eigenvalues = np.linalg.eigvals(cov_matrix)
            if np.any(eigenvalues < -1e-10):  # Allow small negative values due to numerical errors
                return False
            
            # Check if matrix is not all zeros (unless it's intentionally set to -1)
            if np.allclose(cov_matrix, 0) and not np.allclose(cov_matrix[0, 0], -1):
                return False
            
            return True
        except:
            return False
    
    def print_validation_summary(self):
        """Print validation summary"""
        if self.validation_count > 0:
            error_rate = (self.error_count / self.validation_count) * 100
            self.get_logger().info(
                f"\n=== Validation Summary ===\n"
                f"Total messages validated: {self.validation_count}\n"
                f"Messages with errors: {self.error_count}\n"
                f"Error rate: {error_rate:.1f}%\n"
                f"Compliance: {'PASS' if error_rate < 5.0 else 'FAIL'}\n"
                f"========================\n"
            )

def main(args=None):
    rclpy.init(args=args)
    validator = IMUMessageValidator()
    
    try:
        # Run for 60 seconds
        rclpy.spin_once(validator, timeout_sec=60.0)
        
        # Print summary
        validator.print_validation_summary()
        
    except KeyboardInterrupt:
        validator.get_logger().info('Validation interrupted by user')
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
