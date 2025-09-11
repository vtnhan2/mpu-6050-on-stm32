#!/usr/bin/env python3
"""
MPU6050 Choose Output
Script cho phép chọn loại dữ liệu hiển thị trên terminal
"""

import serial
import time
import re
import sys
import os

class MPU6050ChooseOutput:
    def __init__(self, port='COM19', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        
        # Output options
        self.show_accel = True
        self.show_gyro = True
        self.show_temp = True
        self.show_angles = True
        self.show_raw = False
        self.show_timestamp = True
        self.show_separator = True
        
        # Display options
        self.clear_screen = False
        self.compact_mode = False
        self.emoji_mode = True
        
    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"✅ Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"❌ Error connecting to {self.port}: {e}")
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
    
    def clear_terminal(self):
        """Clear terminal screen"""
        if self.clear_screen:
            os.system('cls' if os.name == 'nt' else 'clear')
    
    def display_data(self, data, timestamp, raw_line=""):
        """Display parsed data based on selected options"""
        if self.clear_screen:
            self.clear_terminal()
        
        if self.show_separator and not self.compact_mode:
            print(f"\n{'='*60}")
        
        if self.show_timestamp:
            print(f"⏰ {timestamp}")
        
        if self.show_separator and not self.compact_mode:
            print(f"{'='*60}")
        
        if self.show_raw and raw_line:
            print(f"📡 Raw: {raw_line}")
        
        if self.compact_mode:
            # Compact display
            output_parts = []
            
            if self.show_accel and 'accel' in data:
                accel = data['accel']
                output_parts.append(f"A:{accel['x']:6.2f},{accel['y']:6.2f},{accel['z']:6.2f}")
            
            if self.show_gyro and 'gyro' in data:
                gyro = data['gyro']
                output_parts.append(f"G:{gyro['x']:6.3f},{gyro['y']:6.3f},{gyro['z']:6.3f}")
            
            if self.show_temp and 'temperature' in data:
                temp = data['temperature']
                output_parts.append(f"T:{temp:5.1f}°C")
            
            if self.show_angles and 'angles' in data:
                angles = data['angles']
                output_parts.append(f"R:{angles['roll']:6.1f},P:{angles['pitch']:6.1f},Y:{angles['yaw']:6.1f}")
            
            print(" | ".join(output_parts))
        
        else:
            # Detailed display
            if self.show_accel and 'accel' in data:
                accel = data['accel']
                if self.emoji_mode:
                    print(f"📊 Accelerometer (m/s²):")
                else:
                    print(f"Accelerometer (m/s²):")
                print(f"   X: {accel['x']:8.3f}  Y: {accel['y']:8.3f}  Z: {accel['z']:8.3f}")
            
            if self.show_gyro and 'gyro' in data:
                gyro = data['gyro']
                if self.emoji_mode:
                    print(f"🔄 Gyroscope (rad/s):")
                else:
                    print(f"Gyroscope (rad/s):")
                print(f"   X: {gyro['x']:8.3f}  Y: {gyro['y']:8.3f}  Z: {gyro['z']:8.3f}")
            
            if self.show_temp and 'temperature' in data:
                temp = data['temperature']
                if self.emoji_mode:
                    print(f"🌡️  Temperature: {temp:6.2f} °C")
                else:
                    print(f"Temperature: {temp:6.2f} °C")
            
            if self.show_angles and 'angles' in data:
                angles = data['angles']
                if self.emoji_mode:
                    print(f"📐 Euler Angles (deg):")
                else:
                    print(f"Euler Angles (deg):")
                print(f"   Roll: {angles['roll']:7.2f}  Pitch: {angles['pitch']:7.2f}  Yaw: {angles['yaw']:7.2f}")
    
    def show_menu(self):
        """Show configuration menu"""
        print("\n" + "="*50)
        print("🔧 MPU6050 Output Configuration")
        print("="*50)
        print(f"1. Accelerometer: {'✅' if self.show_accel else '❌'}")
        print(f"2. Gyroscope:     {'✅' if self.show_gyro else '❌'}")
        print(f"3. Temperature:   {'✅' if self.show_temp else '❌'}")
        print(f"4. Angles:        {'✅' if self.show_angles else '❌'}")
        print(f"5. Raw Data:      {'✅' if self.show_raw else '❌'}")
        print(f"6. Timestamp:     {'✅' if self.show_timestamp else '❌'}")
        print(f"7. Separator:     {'✅' if self.show_separator else '❌'}")
        print(f"8. Clear Screen:  {'✅' if self.clear_screen else '❌'}")
        print(f"9. Compact Mode:  {'✅' if self.compact_mode else '❌'}")
        print(f"10. Emoji Mode:   {'✅' if self.emoji_mode else '❌'}")
        print("="*50)
        print("Press number to toggle, 's' to start, 'q' to quit")
    
    def configure_output(self):
        """Configure output options"""
        while True:
            self.show_menu()
            choice = input("Choice: ").strip().lower()
            
            if choice == 'q':
                return False
            elif choice == 's':
                return True
            elif choice == '1':
                self.show_accel = not self.show_accel
            elif choice == '2':
                self.show_gyro = not self.show_gyro
            elif choice == '3':
                self.show_temp = not self.show_temp
            elif choice == '4':
                self.show_angles = not self.show_angles
            elif choice == '5':
                self.show_raw = not self.show_raw
            elif choice == '6':
                self.show_timestamp = not self.show_timestamp
            elif choice == '7':
                self.show_separator = not self.show_separator
            elif choice == '8':
                self.clear_screen = not self.clear_screen
            elif choice == '9':
                self.compact_mode = not self.compact_mode
            elif choice == '10':
                self.emoji_mode = not self.emoji_mode
            else:
                print("❌ Invalid choice!")
    
    def stream_data(self):
        """Main data streaming loop"""
        if not self.connect():
            return
        
        self.running = True
        buffer = ""
        data_count = 0
        
        print("\n🚀 Starting data streaming...")
        print("Press Ctrl+C to stop")
        if self.clear_screen:
            print("Screen will be cleared on each update")
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
                                self.display_data(parsed_data, timestamp, line)
                                
                                # Show data count every 20 readings
                                if data_count % 20 == 0 and not self.compact_mode:
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
            print("✅ Disconnected from serial port")

def main():
    """Main function"""
    print("MPU6050 Choose Output")
    print("=" * 30)
    
    # Get COM port from user or use default
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = input("Enter COM port (default COM19): ").strip()
        if not port:
            port = 'COM19'
    
    print(f"Using COM port: {port}")
    
    # Create streamer
    streamer = MPU6050ChooseOutput(port=port, baudrate=115200)
    
    # Configure output
    if streamer.configure_output():
        streamer.stream_data()

if __name__ == "__main__":
    main()
