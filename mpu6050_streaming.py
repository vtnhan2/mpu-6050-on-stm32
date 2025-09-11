#!/usr/bin/env python3
"""
MPU6050 Data Streaming and Visualization
Streams data from STM32F103C8T6 via UART and displays real-time graphs
"""

import serial
import time
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import threading
import queue

class MPU6050Streamer:
    def __init__(self, port='COM19', baudrate=115200, max_points=200):
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        
        # Data storage
        self.data_queue = queue.Queue()
        self.running = False
        
        # Time series data
        self.timestamps = deque(maxlen=max_points)
        self.accel_x = deque(maxlen=max_points)
        self.accel_y = deque(maxlen=max_points)
        self.accel_z = deque(maxlen=max_points)
        self.gyro_x = deque(maxlen=max_points)
        self.gyro_y = deque(maxlen=max_points)
        self.gyro_z = deque(maxlen=max_points)
        self.temperature = deque(maxlen=max_points)
        self.roll = deque(maxlen=max_points)
        self.pitch = deque(maxlen=max_points)
        self.yaw = deque(maxlen=max_points)
        
        # Serial connection
        self.ser = None
        
        # Matplotlib setup
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('MPU6050 Real-time Data Streaming', fontsize=16)
        
        # Setup subplots
        self.setup_plots()
        
    def setup_plots(self):
        """Setup matplotlib subplots"""
        # Accelerometer plot
        self.ax_accel = self.axes[0, 0]
        self.ax_accel.set_title('Accelerometer (m/s²)')
        self.ax_accel.set_ylabel('Acceleration (m/s²)')
        self.ax_accel.grid(True)
        self.ax_accel.legend(['X', 'Y', 'Z'])
        
        # Gyroscope plot
        self.ax_gyro = self.axes[0, 1]
        self.ax_gyro.set_title('Gyroscope (rad/s)')
        self.ax_gyro.set_ylabel('Angular Velocity (rad/s)')
        self.ax_gyro.grid(True)
        self.ax_gyro.legend(['X', 'Y', 'Z'])
        
        # Temperature plot
        self.ax_temp = self.axes[1, 0]
        self.ax_temp.set_title('Temperature')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_xlabel('Time')
        self.ax_temp.grid(True)
        
        # Angles plot
        self.ax_angles = self.axes[1, 1]
        self.ax_angles.set_title('Euler Angles (deg)')
        self.ax_angles.set_ylabel('Angle (deg)')
        self.ax_angles.set_xlabel('Time')
        self.ax_angles.grid(True)
        self.ax_angles.legend(['Roll', 'Pitch', 'Yaw'])
        
        plt.tight_layout()
        
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)  # Wait for connection to stabilize
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def parse_data(self, line):
        """Parse MPU6050 data from UART"""
        try:
            # Look for accelerometer data
            accel_match = re.search(r'Accelerometer:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
            gyro_match = re.search(r'Gyroscope:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
            temp_match = re.search(r'Temperature:\s+([-\d.]+)', line)
            angles_match = re.search(r'Angles:\s+Roll=([-\d.]+),\s+Pitch=([-\d.]+),\s+Yaw=([-\d.]+)', line)
            
            data = {}
            
            if accel_match:
                data['accel'] = {
                    'x': float(accel_match.group(1)),
                    'y': float(accel_match.group(2)),
                    'z': float(accel_match.group(3))
                }
            
            if gyro_match:
                data['gyro'] = {
                    'x': float(gyro_match.group(1)),
                    'y': float(gyro_match.group(2)),
                    'z': float(gyro_match.group(3))
                }
            
            if temp_match:
                data['temperature'] = float(temp_match.group(1))
            
            if angles_match:
                data['angles'] = {
                    'roll': float(angles_match.group(1)),
                    'pitch': float(angles_match.group(2)),
                    'yaw': float(angles_match.group(3))
                }
            
            return data if data else None
            
        except (ValueError, AttributeError) as e:
            print(f"Error parsing data: {e}")
            return None
    
    def read_serial_data(self):
        """Read data from serial port in separate thread"""
        if not self.ser or not self.ser.is_open:
            return
        
        buffer = ""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line and not line.startswith('===') and not line.startswith('='):
                            parsed_data = self.parse_data(line)
                            if parsed_data:
                                self.data_queue.put(parsed_data)
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                print(f"Error reading serial data: {e}")
                break
    
    def update_data(self, data):
        """Update data arrays with new values"""
        current_time = time.time()
        self.timestamps.append(current_time)
        
        if 'accel' in data:
            self.accel_x.append(data['accel']['x'])
            self.accel_y.append(data['accel']['y'])
            self.accel_z.append(data['accel']['z'])
        
        if 'gyro' in data:
            self.gyro_x.append(data['gyro']['x'])
            self.gyro_y.append(data['gyro']['y'])
            self.gyro_z.append(data['gyro']['z'])
        
        if 'temperature' in data:
            self.temperature.append(data['temperature'])
        
        if 'angles' in data:
            self.roll.append(data['angles']['roll'])
            self.pitch.append(data['angles']['pitch'])
            self.yaw.append(data['angles']['yaw'])
    
    def animate(self, frame):
        """Animation function for matplotlib"""
        # Process all available data from queue
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.update_data(data)
            except queue.Empty:
                break
        
        # Clear and redraw plots
        self.ax_accel.clear()
        self.ax_gyro.clear()
        self.ax_temp.clear()
        self.ax_angles.clear()
        
        if len(self.timestamps) > 1:
            # Convert timestamps to relative time
            time_array = np.array(self.timestamps) - self.timestamps[0]
            
            # Plot accelerometer
            self.ax_accel.set_title('Accelerometer (m/s²)')
            self.ax_accel.set_ylabel('Acceleration (m/s²)')
            if len(self.accel_x) > 0:
                self.ax_accel.plot(time_array, self.accel_x, 'r-', label='X', linewidth=1)
                self.ax_accel.plot(time_array, self.accel_y, 'g-', label='Y', linewidth=1)
                self.ax_accel.plot(time_array, self.accel_z, 'b-', label='Z', linewidth=1)
            self.ax_accel.grid(True)
            self.ax_accel.legend()
            
            # Plot gyroscope
            self.ax_gyro.set_title('Gyroscope (rad/s)')
            self.ax_gyro.set_ylabel('Angular Velocity (rad/s)')
            if len(self.gyro_x) > 0:
                self.ax_gyro.plot(time_array, self.gyro_x, 'r-', label='X', linewidth=1)
                self.ax_gyro.plot(time_array, self.gyro_y, 'g-', label='Y', linewidth=1)
                self.ax_gyro.plot(time_array, self.gyro_z, 'b-', label='Z', linewidth=1)
            self.ax_gyro.grid(True)
            self.ax_gyro.legend()
            
            # Plot temperature
            self.ax_temp.set_title('Temperature')
            self.ax_temp.set_ylabel('Temperature (°C)')
            self.ax_temp.set_xlabel('Time (s)')
            if len(self.temperature) > 0:
                self.ax_temp.plot(time_array, self.temperature, 'purple', linewidth=2)
            self.ax_temp.grid(True)
            
            # Plot angles
            self.ax_angles.set_title('Euler Angles (deg)')
            self.ax_angles.set_ylabel('Angle (deg)')
            self.ax_angles.set_xlabel('Time (s)')
            if len(self.roll) > 0:
                self.ax_angles.plot(time_array, self.roll, 'r-', label='Roll', linewidth=1)
                self.ax_angles.plot(time_array, self.pitch, 'g-', label='Pitch', linewidth=1)
                self.ax_angles.plot(time_array, self.yaw, 'b-', label='Yaw', linewidth=1)
            self.ax_angles.grid(True)
            self.ax_angles.legend()
        
        plt.tight_layout()
    
    def start_streaming(self):
        """Start data streaming and visualization"""
        if not self.connect_serial():
            return
        
        self.running = True
        
        # Start serial reading thread
        serial_thread = threading.Thread(target=self.read_serial_data)
        serial_thread.daemon = True
        serial_thread.start()
        
        print("Starting data streaming...")
        print("Close the plot window to stop streaming")
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.animate, interval=50, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nStopping streaming...")
        finally:
            self.stop_streaming()
    
    def stop_streaming(self):
        """Stop data streaming"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("Streaming stopped")

def main():
    """Main function"""
    print("MPU6050 Data Streaming and Visualization")
    print("=" * 50)
    
    # You can change the COM port here
    port = 'COM19'  # Change this to your actual COM port
    
    # Create and start streamer
    streamer = MPU6050Streamer(port=port, baudrate=115200)
    
    try:
        streamer.start_streaming()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        streamer.stop_streaming()

if __name__ == "__main__":
    main()
