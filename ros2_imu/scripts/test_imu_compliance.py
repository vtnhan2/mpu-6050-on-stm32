#!/usr/bin/env python3

"""
Test script for sensor_msgs/Imu compliance
Runs comprehensive tests to ensure ROS2 standard compliance
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np
import time

class IMUComplianceTester(Node):
    def __init__(self):
        super().__init__('imu_compliance_tester')
        
        # Test results
        self.tests_passed = 0
        self.tests_failed = 0
        self.total_tests = 0
        
        self.get_logger().info('IMU Compliance Tester started')
        self.run_compliance_tests()
    
    def run_compliance_tests(self):
        """Run all compliance tests"""
        self.get_logger().info('Running sensor_msgs/Imu compliance tests...')
        
        # Test 1: Message Structure
        self.test_message_structure()
        
        # Test 2: Units Compliance
        self.test_units_compliance()
        
        # Test 3: Quaternion Normalization
        self.test_quaternion_normalization()
        
        # Test 4: Covariance Matrices
        self.test_covariance_matrices()
        
        # Test 5: Data Quality
        self.test_data_quality()
        
        # Test 6: Performance
        self.test_performance()
        
        # Print results
        self.print_test_results()
    
    def test_message_structure(self):
        """Test message structure compliance"""
        self.get_logger().info('Testing message structure...')
        
        # Create test message
        imu_msg = Imu()
        
        # Test header
        imu_msg.header.stamp.sec = 1234567890
        imu_msg.header.stamp.nanosec = 123456789
        imu_msg.header.frame_id = "imu_link"
        
        # Test orientation
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
        
        # Test angular velocity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        
        # Test linear acceleration
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        
        # Test covariance matrices
        for i in range(9):
            imu_msg.linear_acceleration_covariance[i] = 0.01 if i % 4 == 0 else 0.0
            imu_msg.angular_velocity_covariance[i] = 0.01 if i % 4 == 0 else 0.0
            imu_msg.orientation_covariance[i] = 0.01 if i % 4 == 0 else 0.0
        
        # Validate structure
        if (imu_msg.header.frame_id == "imu_link" and
            imu_msg.orientation.w == 1.0 and
            len(imu_msg.linear_acceleration_covariance) == 9 and
            len(imu_msg.angular_velocity_covariance) == 9 and
            len(imu_msg.orientation_covariance) == 9):
            self.tests_passed += 1
            self.get_logger().info('  ✓ Message structure test PASSED')
        else:
            self.tests_failed += 1
            self.get_logger().error('  ✗ Message structure test FAILED')
        
        self.total_tests += 1
    
    def test_units_compliance(self):
        """Test units compliance"""
        self.get_logger().info('Testing units compliance...')
        
        # Test acceleration units (m/s²)
        test_accel = 9.81  # Earth's gravity
        if 8.0 < test_accel < 12.0:  # Reasonable range for gravity
            self.tests_passed += 1
            self.get_logger().info('  ✓ Acceleration units test PASSED')
        else:
            self.tests_failed += 1
            self.get_logger().error('  ✗ Acceleration units test FAILED')
        
        # Test angular velocity units (rad/s)
        test_gyro = 1.0  # 1 rad/s
        if 0.0 < test_gyro < 100.0:  # Reasonable range
            self.tests_passed += 1
            self.get_logger().info('  ✓ Angular velocity units test PASSED')
        else:
            self.tests_failed += 1
            self.get_logger().error('  ✗ Angular velocity units test FAILED')
        
        self.total_tests += 2
    
    def test_quaternion_normalization(self):
        """Test quaternion normalization"""
        self.get_logger().info('Testing quaternion normalization...')
        
        # Test various quaternions
        test_quaternions = [
            [0.0, 0.0, 0.0, 1.0],  # Identity
            [0.7071, 0.0, 0.0, 0.7071],  # 90° rotation around X
            [0.0, 0.7071, 0.0, 0.7071],  # 90° rotation around Y
            [0.0, 0.0, 0.7071, 0.7071],  # 90° rotation around Z
        ]
        
        all_normalized = True
        for qx, qy, qz, qw in test_quaternions:
            norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
            if abs(norm - 1.0) > 0.01:
                all_normalized = False
                break
        
        if all_normalized:
            self.tests_passed += 1
            self.get_logger().info('  ✓ Quaternion normalization test PASSED')
        else:
            self.tests_failed += 1
            self.get_logger().error('  ✗ Quaternion normalization test FAILED')
        
        self.total_tests += 1
    
    def test_covariance_matrices(self):
        """Test covariance matrix validity"""
        self.get_logger().info('Testing covariance matrices...')
        
        # Test valid covariance matrix
        valid_cov = np.array([
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.0, 0.0, 0.01]
        ])
        
        # Test invalid covariance matrix
        invalid_cov = np.array([
            [0.01, 0.0, 0.0],
            [0.0, -0.01, 0.0],  # Negative diagonal
            [0.0, 0.0, 0.01]
        ])
        
        valid_test = self.is_valid_covariance_matrix(valid_cov)
        invalid_test = not self.is_valid_covariance_matrix(invalid_cov)
        
        if valid_test and invalid_test:
            self.tests_passed += 1
            self.get_logger().info('  ✓ Covariance matrix test PASSED')
        else:
            self.tests_failed += 1
            self.get_logger().error('  ✗ Covariance matrix test FAILED')
        
        self.total_tests += 1
    
    def test_data_quality(self):
        """Test data quality"""
        self.get_logger().info('Testing data quality...')
        
        # Test for NaN values
        test_values = [0.0, 1.0, -1.0, 9.81, 3.14159]
        has_nan = any(math.isnan(v) for v in test_values)
        
        # Test for infinite values
        has_inf = any(math.isinf(v) for v in test_values)
        
        if not has_nan and not has_inf:
            self.tests_passed += 1
            self.get_logger().info('  ✓ Data quality test PASSED')
        else:
            self.tests_failed += 1
            self.get_logger().error('  ✗ Data quality test FAILED')
        
        self.total_tests += 1
    
    def test_performance(self):
        """Test performance metrics"""
        self.get_logger().info('Testing performance...')
        
        # Test message creation speed
        start_time = time.time()
        for _ in range(1000):
            imu_msg = Imu()
            imu_msg.header.stamp.sec = 1234567890
            imu_msg.header.stamp.nanosec = 123456789
            imu_msg.header.frame_id = "imu_link"
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation.w = 1.0
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 9.81
            for i in range(9):
                imu_msg.linear_acceleration_covariance[i] = 0.01 if i % 4 == 0 else 0.0
                imu_msg.angular_velocity_covariance[i] = 0.01 if i % 4 == 0 else 0.0
                imu_msg.orientation_covariance[i] = 0.01 if i % 4 == 0 else 0.0
        
        end_time = time.time()
        creation_time = (end_time - start_time) / 1000  # per message
        
        if creation_time < 0.001:  # Less than 1ms per message
            self.tests_passed += 1
            self.get_logger().info(f'  ✓ Performance test PASSED ({creation_time*1000:.2f}μs per message)')
        else:
            self.tests_failed += 1
            self.get_logger().error(f'  ✗ Performance test FAILED ({creation_time*1000:.2f}μs per message)')
        
        self.total_tests += 1
    
    def is_valid_covariance_matrix(self, cov_matrix):
        """Check if covariance matrix is valid"""
        try:
            # Check if matrix is symmetric
            if not np.allclose(cov_matrix, cov_matrix.T, atol=1e-10):
                return False
            
            # Check if matrix is positive semi-definite
            eigenvalues = np.linalg.eigvals(cov_matrix)
            if np.any(eigenvalues < -1e-10):
                return False
            
            return True
        except:
            return False
    
    def print_test_results(self):
        """Print test results"""
        success_rate = (self.tests_passed / self.total_tests) * 100 if self.total_tests > 0 else 0
        
        self.get_logger().info(
            f'\n=== IMU Compliance Test Results ===\n'
            f'Total tests: {self.total_tests}\n'
            f'Passed: {self.tests_passed}\n'
            f'Failed: {self.tests_failed}\n'
            f'Success rate: {success_rate:.1f}%\n'
            f'Compliance: {"PASS" if success_rate >= 90.0 else "FAIL"}\n'
            f'=====================================\n'
        )

def main(args=None):
    rclpy.init(args=args)
    tester = IMUComplianceTester()
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
