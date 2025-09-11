#!/usr/bin/env python3
"""
Debug UART Connection
Kiểm tra kết nối UART và dữ liệu thô
"""

import serial
import time
import sys

def debug_uart(port='COM19', baudrate=115200):
    """Debug UART connection"""
    print(f"🔍 Debugging UART connection...")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print("-" * 50)
    
    try:
        # Kết nối UART
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✅ Connected to {port}")
        
        # Đợi dữ liệu
        print("⏳ Waiting for data... (Press Ctrl+C to stop)")
        print("Raw data from STM32:")
        print("-" * 50)
        
        buffer = ""
        timeout_count = 0
        
        while True:
            if ser.in_waiting > 0:
                # Đọc dữ liệu thô
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                
                # Hiển thị dữ liệu thô
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        print(f"📡 {line}")
                        timeout_count = 0
            else:
                timeout_count += 1
                if timeout_count > 100:  # 10 giây timeout
                    print("⏰ No data received for 10 seconds...")
                    print("🔧 Troubleshooting:")
                    print("   1. Check STM32 is running")
                    print("   2. Check wiring (PA2→RX, PA3→TX)")
                    print("   3. Check baud rate (115200)")
                    print("   4. Try different COM port")
                    timeout_count = 0
                
                time.sleep(0.1)
                
    except serial.SerialException as e:
        print(f"❌ Serial Error: {e}")
        print("🔧 Try these solutions:")
        print("   1. Check COM port number")
        print("   2. Close other programs using the port")
        print("   3. Try different COM port")
        print("   4. Check USB-UART driver")
        
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user")
        
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("✅ Serial port closed")

def list_com_ports():
    """List available COM ports"""
    print("🔍 Available COM ports:")
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if ports:
            for port in ports:
                print(f"   {port.device} - {port.description}")
        else:
            print("   No COM ports found")
    except ImportError:
        print("   Install pyserial: pip install pyserial")

if __name__ == "__main__":
    print("MPU6050 UART Debug Tool")
    print("=" * 30)
    
    # List available ports
    list_com_ports()
    print()
    
    # Get port from user
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = input("Enter COM port (default COM19): ").strip()
        if not port:
            port = 'COM19'
    
    debug_uart(port)
