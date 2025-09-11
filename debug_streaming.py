#!/usr/bin/env python3
"""
Debug Streaming Script
Kiểm tra tại sao streaming không hiển thị sóng
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

class DebugStreamer:
    def __init__(self, port='COM19', baudrate=115200, max_points=50):
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
        
        # Serial connection
        self.ser = None
        
        # Debug counters
        self.data_count = 0
        self.parse_count = 0
        self.plot_count = 0
        
        # Matplotlib setup
        self.fig, self.axes = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('MPU6050 Debug Streaming', fontsize=16)
        
        # Setup subplots
        self.setup_plots()
        
    def setup_plots(self):
        """Setup matplotlib subplots"""
        # Accelerometer plot
        self.ax_accel = self.axes[0]
        self.ax_accel.set_title('Accelerometer (m/s²)')
        self.ax_accel.set_ylabel('Acceleration (m/s²)')
        self.ax_accel.grid(True)
        
        # Gyroscope plot
        self.ax_gyro = self.axes[1]
        self.ax_gyro.set_title('Gyroscope (rad/s)')
        self.ax_gyro.set_ylabel('Angular Velocity (rad/s)')
        self.ax_gyro.set_xlabel('Time (s)')
        self.ax_gyro.grid(True)
        
        plt.tight_layout()
        
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✅ Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"❌ Error connecting to {self.port}: {e}")
            return False
    
    def parse_data(self, line):
        """Parse MPU6050 data from UART"""
        try:
            # Look for accelerometer data
            accel_match = re.search(r'Accelerometer:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
            gyro_match = re.search(r'Gyroscope:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
            
            data = {}
            
            if accel_match:
                data['accel'] = {
                    'x': float(accel_match.group(1)),
                    'y': float(accel_match.group(2)),
                    'z': float(accel_match.group(3))
                }
                self.parse_count += 1
                print(f"📊 Parsed accelerometer data #{self.parse_count}")
            
            if gyro_match:
                data['gyro'] = {
                    'x': float(gyro_match.group(1)),
                    'y': float(gyro_match.group(2)),
                    'z': float(gyro_match.group(3))
                }
                self.parse_count += 1
                print(f"🔄 Parsed gyroscope data #{self.parse_count}")
            
            return data if data else None
            
        except (ValueError, AttributeError) as e:
            print(f"❌ Error parsing data: {e}")
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
                    self.data_count += 1
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line and not line.startswith('===') and not line.startswith('='):
                            parsed_data = self.parse_data(line)
                            if parsed_data:
                                self.data_queue.put(parsed_data)
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"❌ Error reading serial data: {e}")
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
    
    def animate(self, frame):
        """Animation function for matplotlib"""
        # Process all available data from queue
        queue_size = 0
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.update_data(data)
                queue_size += 1
            except queue.Empty:
                break
        
        if queue_size > 0:
            print(f"📈 Processed {queue_size} data points from queue")
        
        # Clear and redraw plots
        self.ax_accel.clear()
        self.ax_gyro.clear()
        
        if len(self.timestamps) > 1:
            # Convert timestamps to relative time
            time_array = np.array(self.timestamps) - self.timestamps[0]
            
            # Plot accelerometer
            self.ax_accel.set_title(f'Accelerometer (m/s²) - {len(self.accel_x)} points')
            self.ax_accel.set_ylabel('Acceleration (m/s²)')
            if len(self.accel_x) > 0:
                self.ax_accel.plot(time_array, self.accel_x, 'r-', label='X', linewidth=2)
                self.ax_accel.plot(time_array, self.accel_y, 'g-', label='Y', linewidth=2)
                self.ax_accel.plot(time_array, self.accel_z, 'b-', label='Z', linewidth=2)
                self.ax_accel.legend()
            self.ax_accel.grid(True)
            
            # Plot gyroscope
            self.ax_gyro.set_title(f'Gyroscope (rad/s) - {len(self.gyro_x)} points')
            self.ax_gyro.set_ylabel('Angular Velocity (rad/s)')
            self.ax_gyro.set_xlabel('Time (s)')
            if len(self.gyro_x) > 0:
                self.ax_gyro.plot(time_array, self.gyro_x, 'r-', label='X', linewidth=2)
                self.ax_gyro.plot(time_array, self.gyro_y, 'g-', label='Y', linewidth=2)
                self.ax_gyro.plot(time_array, self.gyro_z, 'b-', label='Z', linewidth=2)
                self.ax_gyro.legend()
            self.ax_gyro.grid(True)
            
            self.plot_count += 1
            if self.plot_count % 10 == 0:
                print(f"🎨 Plot updated #{self.plot_count}")
        else:
            # Show debug info when no data
            self.ax_accel.set_title('Accelerometer (m/s²) - Waiting for data...')
            self.ax_gyro.set_title('Gyroscope (rad/s) - Waiting for data...')
            self.ax_accel.text(0.5, 0.5, f'Data count: {self.data_count}\nParse count: {self.parse_count}', 
                             transform=self.ax_accel.transAxes, ha='center', va='center')
        
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
        
        print("🚀 Starting debug streaming...")
        print("📊 Debug info will be printed to console")
        print("🛑 Close the plot window to stop")
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.animate, interval=100, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n🛑 Stopping streaming...")
        finally:
            self.stop_streaming()
    
    def stop_streaming(self):
        """Stop data streaming"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print(f"📊 Final stats:")
        print(f"   Data received: {self.data_count}")
        print(f"   Data parsed: {self.parse_count}")
        print(f"   Plots updated: {self.plot_count}")

def main():
    """Main function"""
    print("MPU6050 Debug Streaming")
    print("=" * 30)
    
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM19'
    
    streamer = DebugStreamer(port=port, baudrate=115200)
    streamer.start_streaming()

if __name__ == "__main__":
    main()
