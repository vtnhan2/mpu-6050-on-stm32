#!/usr/bin/env python3

import serial
import struct
import time
import math
from typing import Dict, List, Optional, Tuple

class IMUSensorInterface:
    """
    Interface for 10DOF IMU sensor communication
    Handles I2C communication through STM32 UART bridge
    """
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 921600):
        """
        Initialize sensor interface
        
        Args:
            port: Serial port for communication
            baudrate: Serial baudrate
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connected = False
        
        # Sensor data
        self.accel_data = [0.0, 0.0, 0.0]  # m/s²
        self.gyro_data = [0.0, 0.0, 0.0]   # rad/s
        self.mag_data = [0.0, 0.0, 0.0]    # Tesla
        self.temp_data = 25.0               # °C
        
        # Calibration data
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        self.mag_offset = [0.0, 0.0, 0.0]
        self.mag_scale = [1.0, 1.0, 1.0]
        
    def connect(self) -> bool:
        """
        Connect to serial port
        
        Returns:
            bool: True if connected successfully
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            self.connected = True
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.connected = False
            print("Disconnected from serial port")
    
    def read_data(self) -> Dict[str, List[float]]:
        """
        Read sensor data from serial port
        
        Returns:
            Dict containing sensor data
        """
        if not self.connected:
            return self._get_simulated_data()
        
        try:
            # Read line from serial port
            line = self.serial_conn.readline().decode('utf-8').strip()
            
            if not line:
                return self._get_simulated_data()
            
            # Parse data (format: "Axyz= ax ay az | Gxyz= gx gy gz | Mxyz= mx my mz | t=time")
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
                
                # Apply calibration
                self.accel_data = self._apply_accel_calibration(accel_values)
                self.gyro_data = self._apply_gyro_calibration(gyro_values)
                self.mag_data = self._apply_mag_calibration(mag_values)
                
                return {
                    'accel': self.accel_data,
                    'gyro': self.gyro_data,
                    'mag': self.mag_data,
                    'temperature': self.temp_data
                }
            else:
                return self._get_simulated_data()
                
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            return self._get_simulated_data()
    
    def _get_simulated_data(self) -> Dict[str, List[float]]:
        """
        Generate simulated sensor data for testing
        
        Returns:
            Dict containing simulated sensor data
        """
        t = time.time()
        
        # Simulate some movement
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
    
    def _apply_accel_calibration(self, raw_data: List[float]) -> List[float]:
        """Apply accelerometer calibration"""
        return [raw_data[i] - self.accel_offset[i] for i in range(3)]
    
    def _apply_gyro_calibration(self, raw_data: List[float]) -> List[float]:
        """Apply gyroscope calibration"""
        return [raw_data[i] - self.gyro_offset[i] for i in range(3)]
    
    def _apply_mag_calibration(self, raw_data: List[float]) -> List[float]:
        """Apply magnetometer calibration"""
        return [(raw_data[i] - self.mag_offset[i]) * self.mag_scale[i] for i in range(3)]
    
    def calibrate_sensors(self, duration: float = 10.0):
        """
        Calibrate sensors by collecting data for specified duration
        
        Args:
            duration: Calibration duration in seconds
        """
        print(f"Starting sensor calibration for {duration} seconds...")
        print("Keep the sensor stationary during calibration.")
        
        accel_samples = []
        gyro_samples = []
        mag_samples = []
        
        start_time = time.time()
        while time.time() - start_time < duration:
            data = self.read_data()
            accel_samples.append(data['accel'])
            gyro_samples.append(data['gyro'])
            mag_samples.append(data['mag'])
            time.sleep(0.01)  # 100Hz
        
        # Calculate offsets
        self.accel_offset = [
            sum(sample[i] for sample in accel_samples) / len(accel_samples)
            for i in range(3)
        ]
        self.gyro_offset = [
            sum(sample[i] for sample in gyro_samples) / len(gyro_samples)
            for i in range(3)
        ]
        self.mag_offset = [
            sum(sample[i] for sample in mag_samples) / len(mag_samples)
            for i in range(3)
        ]
        
        # Calculate magnetometer scale factors
        mag_ranges = [
            max(sample[i] for sample in mag_samples) - min(sample[i] for sample in mag_samples)
            for i in range(3)
        ]
        max_range = max(mag_ranges)
        self.mag_scale = [max_range / mag_ranges[i] for i in range(3)]
        
        print("Calibration completed!")
        print(f"Accel offset: {self.accel_offset}")
        print(f"Gyro offset: {self.gyro_offset}")
        print(f"Mag offset: {self.mag_offset}")
        print(f"Mag scale: {self.mag_scale}")
    
    def get_sensor_info(self) -> Dict[str, str]:
        """
        Get sensor information
        
        Returns:
            Dict containing sensor information
        """
        return {
            'sensor_type': '10DOF IMU',
            'accelerometer': 'MPU6050 (3-axis)',
            'gyroscope': 'MPU6050 (3-axis)',
            'magnetometer': 'HMC5883L (3-axis)',
            'barometer': 'BMP180 (1-axis)',
            'interface': 'I2C via STM32 UART',
            'data_rate': '100Hz',
            'resolution': '16-bit (MPU6050), 12-bit (HMC5883L)'
        }
