#!/usr/bin/env python3
"""
Test UART với các baud rate khác nhau
"""

import serial
import time

def test_baud_rates(port='COM19'):
    """Test different baud rates"""
    baud_rates = [9600, 19200, 38400, 57600, 115200, 230400]
    
    for baud in baud_rates:
        print(f"\n🔍 Testing {port} at {baud} baud...")
        try:
            ser = serial.Serial(port, baud, timeout=2)
            time.sleep(1)
            
            # Read for 3 seconds
            start_time = time.time()
            data_received = False
            
            while time.time() - start_time < 3:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    if data.strip():
                        print(f"✅ Data received: {data[:50]}...")
                        data_received = True
                        break
                time.sleep(0.1)
            
            if not data_received:
                print(f"❌ No data at {baud} baud")
            
            ser.close()
            
        except Exception as e:
            print(f"❌ Error at {baud} baud: {e}")

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM19'
    test_baud_rates(port)
