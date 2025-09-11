#!/usr/bin/env python3
"""
MPU6050 Simple Data Streaming
Simple console-based data streaming from STM32F103C8T6
"""

import serial
import time
import re
import sys

class SimpleMPU6050Streamer:
    def __init__(self, port='COM19', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        
    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"✗ Error connecting to {self.port}: {e}")
            return False
    
    def parse_line(self, line):
        """Parse a single line of MPU6050 data"""
        data = {}
        
        # Parse accelerometer
        accel_match = re.search(r'Accelerometer:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
        if accel_match:
            data['accel'] = {
                'x': float(accel_match.group(1)),
                'y': float(accel_match.group(2)),
                'z': float(accel_match.group(3))
            }
        
        # Parse gyroscope
        gyro_match = re.search(r'Gyroscope:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
        if gyro_match:
            data['gyro'] = {
                'x': float(gyro_match.group(1)),
                'y': float(gyro_match.group(2)),
                'z': float(gyro_match.group(3))
            }
        
        # Parse temperature
        temp_match = re.search(r'Temperature:\s+([-\d.]+)', line)
        if temp_match:
            data['temperature'] = float(temp_match.group(1))
        
        # Parse angles
        angles_match = re.search(r'Angles:\s+Roll=([-\d.]+),\s+Pitch=([-\d.]+),\s+Yaw=([-\d.]+)', line)
        if angles_match:
            data['angles'] = {
                'roll': float(angles_match.group(1)),
                'pitch': float(angles_match.group(2)),
                'yaw': float(angles_match.group(3))
            }
        
        return data
    
    def display_data(self, data, timestamp):
        """Display parsed data in a formatted way"""
        print(f"\n{'='*60}")
        print(f"Timestamp: {timestamp}")
        print(f"{'='*60}")
        
        if 'accel' in data:
            accel = data['accel']
            print(f"📊 Accelerometer (m/s²):")
            print(f"   X: {accel['x']:8.3f}  Y: {accel['y']:8.3f}  Z: {accel['z']:8.3f}")
        
        if 'gyro' in data:
            gyro = data['gyro']
            print(f"🔄 Gyroscope (rad/s):")
            print(f"   X: {gyro['x']:8.3f}  Y: {gyro['y']:8.3f}  Z: {gyro['z']:8.3f}")
        
        if 'temperature' in data:
            temp = data['temperature']
            print(f"🌡️  Temperature: {temp:6.2f} °C")
        
        if 'angles' in data:
            angles = data['angles']
            print(f"📐 Euler Angles (deg):")
            print(f"   Roll: {angles['roll']:7.2f}  Pitch: {angles['pitch']:7.2f}  Yaw: {angles['yaw']:7.2f}")
    
    def stream_data(self):
        """Main data streaming loop"""
        if not self.connect():
            return
        
        self.running = True
        buffer = ""
        data_count = 0
        
        print("\n🚀 Starting data streaming...")
        print("Press Ctrl+C to stop")
        print("-" * 60)
        
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    # Read available data
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        # Skip header lines and separators
                        if line and not line.startswith('===') and not line.startswith('='):
                            parsed_data = self.parse_line(line)
                            
                            if parsed_data:
                                data_count += 1
                                timestamp = time.strftime("%H:%M:%S")
                                self.display_data(parsed_data, timestamp)
                                
                                # Show data count every 10 readings
                                if data_count % 10 == 0:
                                    print(f"\n📈 Total readings: {data_count}")
                
                time.sleep(0.01)  # Small delay
                
        except KeyboardInterrupt:
            print(f"\n\n🛑 Stopping streaming...")
            print(f"📊 Total readings received: {data_count}")
        except Exception as e:
            print(f"\n❌ Error during streaming: {e}")
        finally:
            self.disconnect()
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected from serial port")

def main():
    """Main function"""
    print("MPU6050 Simple Data Streaming")
    print("=" * 40)
    
    # Get COM port from user or use default
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = input("Enter COM port (default COM19): ").strip()
        if not port:
            port = 'COM19'
    
    print(f"Using COM port: {port}")
    
    # Create and start streamer
    streamer = SimpleMPU6050Streamer(port=port, baudrate=115200)
    streamer.stream_data()

if __name__ == "__main__":
    main()
