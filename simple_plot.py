#!/usr/bin/env python3
"""
Simple Plot Test
Test plotting without animation
"""

import serial
import time
import re
import matplotlib.pyplot as plt
import numpy as np

def simple_plot_test(port='COM19', baudrate=115200):
    """Simple plot test without animation"""
    print(f"🔍 Testing simple plot with {port}")
    
    try:
        # Connect to serial
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✅ Connected to {port}")
        time.sleep(2)
        
        # Collect data for 10 seconds
        data_points = []
        start_time = time.time()
        
        print("📊 Collecting data for 10 seconds...")
        
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                lines = data.split('\n')
                
                for line in lines:
                    line = line.strip()
                    if line and not line.startswith('===') and not line.startswith('='):
                        # Parse accelerometer
                        accel_match = re.search(r'Accelerometer:\s+X=([-\d.]+),\s+Y=([-\d.]+),\s+Z=([-\d.]+)', line)
                        if accel_match:
                            data_points.append({
                                'time': time.time() - start_time,
                                'ax': float(accel_match.group(1)),
                                'ay': float(accel_match.group(2)),
                                'az': float(accel_match.group(3))
                            })
                            print(f"📈 Collected {len(data_points)} points")
            
            time.sleep(0.01)
        
        ser.close()
        
        if len(data_points) > 0:
            print(f"✅ Collected {len(data_points)} data points")
            
            # Create plot
            times = [p['time'] for p in data_points]
            ax_values = [p['ax'] for p in data_points]
            ay_values = [p['ay'] for p in data_points]
            az_values = [p['az'] for p in data_points]
            
            plt.figure(figsize=(12, 6))
            plt.plot(times, ax_values, 'r-', label='X', linewidth=2)
            plt.plot(times, ay_values, 'g-', label='Y', linewidth=2)
            plt.plot(times, az_values, 'b-', label='Z', linewidth=2)
            plt.title('MPU6050 Accelerometer Data (m/s²)')
            plt.xlabel('Time (s)')
            plt.ylabel('Acceleration (m/s²)')
            plt.legend()
            plt.grid(True)
            plt.show()
            
        else:
            print("❌ No data collected")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM19'
    simple_plot_test(port)
