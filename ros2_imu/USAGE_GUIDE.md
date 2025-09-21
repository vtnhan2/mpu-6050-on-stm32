# GY87 IMU ROS2 Package - Hướng dẫn sử dụng

## 🚀 **Quick Start**

### **1. Cài đặt Dependencies:**
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-tf2-ros ros-humble-rviz2
sudo apt install python3-serial python3-numpy

# Windows (WSL2)
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-tf2-ros ros-humble-rviz2
pip install pyserial numpy
```

### **2. Build Package:**
```bash
cd ~/ros2_ws/src
git clone <repository_url> gy87_imu
cd ~/ros2_ws
colcon build --packages-select gy87_imu
source install/setup.bash
```

### **3. Chạy Demo:**
```bash
# Terminal 1: Launch IMU node
ros2 launch gy87_imu gy87_imu_simple.launch.py

# Terminal 2: Monitor data
ros2 topic echo /imu/data

# Terminal 3: Run demo script
ros2 run gy87_imu demo_gy87_imu.py
```

## 🔌 **Hardware Setup**

### **Kết nối STM32F103C8T6:**
```
VCC  → 3.3V
GND  → GND
SCL  → PB8 (I2C1_SCL)
SDA  → PB9 (I2C1_SDA)
```

### **Kết nối UART:**
```
STM32 TX (PA9) → PC RX
STM32 RX (PA10) ← PC TX
STM32 GND → PC GND
```

### **Kiểm tra kết nối:**
```bash
# Linux
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# Windows
# Check Device Manager for COM ports
```

## 📊 **Data Monitoring**

### **1. Xem tất cả topics:**
```bash
ros2 topic list
```

### **2. Monitor IMU data:**
```bash
ros2 topic echo /imu/data
```

### **3. Monitor magnetometer:**
```bash
ros2 topic echo /imu/mag
```

### **4. Monitor temperature:**
```bash
ros2 topic echo /imu/temp
```

### **5. Check data rate:**
```bash
ros2 topic hz /imu/data
```

## 🎯 **Advanced Usage**

### **1. Custom Serial Port:**
```bash
ros2 launch gy87_imu gy87_imu.launch.py serial_port:=/dev/ttyUSB0
```

### **2. Custom Baudrate:**
```bash
ros2 launch gy87_imu gy87_imu.launch.py serial_baudrate:=115200
```

### **3. With RViz Visualization:**
```bash
ros2 launch gy87_imu gy87_imu.launch.py
```

### **4. Python Node Directly:**
```bash
ros2 run gy87_imu gy87_imu_python_node.py
```

## 🔧 **Configuration**

### **1. Launch Parameters:**
```yaml
# gy87_imu/config/gy87_imu.yaml
gy87_imu_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    serial_baudrate: 921600
    use_sim_time: false
    data_rate: 100.0
    calibration_duration: 10.0
```

### **2. RViz Configuration:**
```bash
# Load custom RViz config
ros2 run rviz2 rviz2 -d install/gy87_imu/share/gy87_imu/config/gy87_imu.rviz
```

## 🛠️ **Troubleshooting**

### **1. Không có data:**
```bash
# Check serial connection
ros2 run gy87_imu demo_gy87_imu.py

# Check topics
ros2 topic list
ros2 topic info /imu/data

# Check node status
ros2 node list
ros2 node info /gy87_imu_node
```

### **2. Data không chính xác:**
```bash
# Calibrate sensors
ros2 run gy87_imu gy87_imu_python_node.py --calibrate

# Check for interference
ros2 topic echo /imu/mag
```

### **3. Performance issues:**
```bash
# Check data rate
ros2 topic hz /imu/data

# Monitor CPU usage
htop

# Check memory usage
ros2 node info /gy87_imu_node
```

## 📈 **Data Analysis**

### **1. Record data:**
```bash
ros2 bag record /imu/data /imu/mag /imu/temp
```

### **2. Playback data:**
```bash
ros2 bag play <bag_file>
```

### **3. Analyze with Python:**
```python
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt

# Load bag file
reader = rosbag2_py.SequentialReader()
reader.open_uri("path/to/bag", "sqlite3")

# Extract data
accel_data = []
gyro_data = []
mag_data = []

for topic, msg, timestamp in reader.read_messages():
    if topic == "/imu/data":
        accel_data.append([msg.linear_acceleration.x, 
                          msg.linear_acceleration.y, 
                          msg.linear_acceleration.z])
        gyro_data.append([msg.angular_velocity.x, 
                         msg.angular_velocity.y, 
                         msg.angular_velocity.z])
    elif topic == "/imu/mag":
        mag_data.append([msg.magnetic_field.x, 
                        msg.magnetic_field.y, 
                        msg.magnetic_field.z])

# Plot data
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(accel_data)
plt.title('Accelerometer Data')
plt.ylabel('m/s²')

plt.subplot(3, 1, 2)
plt.plot(gyro_data)
plt.title('Gyroscope Data')
plt.ylabel('rad/s')

plt.subplot(3, 1, 3)
plt.plot(mag_data)
plt.title('Magnetometer Data')
plt.ylabel('T')

plt.tight_layout()
plt.show()
```

## 🧪 **Testing**

### **1. Unit Tests:**
```bash
cd ~/ros2_ws
colcon test --packages-select gy87_imu
```

### **2. Integration Tests:**
```bash
# Test with simulated data
ros2 launch gy87_imu gy87_imu_simple.launch.py

# Test with real hardware
ros2 launch gy87_imu gy87_imu.launch.py serial_port:=/dev/ttyUSB0
```

### **3. Performance Tests:**
```bash
# Test data rate
ros2 topic hz /imu/data

# Test latency
ros2 topic echo /imu/data --once
```

## 📚 **API Reference**

### **C++ Node:**
- **Class**: `GY87IMUNode`
- **Publishes**: `/imu/data`, `/imu/mag`, `/imu/temp`
- **Broadcasts**: `base_link` → `imu_link`

### **Python Node:**
- **Class**: `GY87IMUPythonNode`
- **Features**: Serial communication, calibration, error handling
- **Dependencies**: `pyserial`, `numpy`

### **Sensor Interface:**
- **Class**: `GY87SensorInterface`
- **Methods**: `connect()`, `read_data()`, `calibrate_sensors()`
- **Features**: Calibration, error handling, simulated data

## 🤝 **Contributing**

### **1. Fork Repository:**
```bash
git clone <your_fork_url>
cd gy87_imu
```

### **2. Create Feature Branch:**
```bash
git checkout -b feature/new-feature
```

### **3. Make Changes:**
```bash
# Edit code
git add .
git commit -m "Add new feature"
```

### **4. Push and Create PR:**
```bash
git push origin feature/new-feature
# Create Pull Request on GitHub
```

## 📞 **Support**

### **Issues:**
- GitHub Issues: [Create Issue](https://github.com/username/gy87_imu/issues)
- Email: developer@example.com

### **Documentation:**
- Wiki: [Project Wiki](https://github.com/username/gy87_imu/wiki)
- API Docs: [API Documentation](https://github.com/username/gy87_imu/docs)

### **Community:**
- ROS2 Discourse: [Community Forum](https://discourse.ros.org/)
- Stack Overflow: [ROS2 Tag](https://stackoverflow.com/questions/tagged/ros2)
