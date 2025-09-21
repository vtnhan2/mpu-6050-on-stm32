# IMU ROS2 Package

## 📋 **Tổng quan**

ROS2 package cho 10DOF IMU (MPU6050 + HMC5883L + BMP180) với STM32F103C8T6 microcontroller.

## 🚀 **Tính năng**

- **10DOF IMU**: Accelerometer, Gyroscope, Magnetometer, Barometer
- **ROS2 Integration**: sensor_msgs/Imu, sensor_msgs/MagneticField, sensor_msgs/Temperature
- **Standard Compliance**: Full sensor_msgs/Imu ROS2 standard compliance
- **Real-time Data**: 100Hz data rate
- **Complementary Filter**: Orientation estimation
- **TF2 Support**: Transform broadcasting
- **RViz Visualization**: Real-time sensor data display
- **Message Validation**: Built-in compliance testing and validation

## 📦 **Cài đặt**

### **Dependencies:**
```bash
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-rviz2
sudo apt install python3-serial
```

### **Build Package:**
```bash
cd ~/ros2_ws/src
git clone <repository_url> imu
cd ~/ros2_ws
colcon build --packages-select imu
source install/setup.bash
```

## 🔌 **Kết nối Hardware**

### **STM32F103C8T6 Pinout:**
```
VCC  → 3.3V
GND  → GND
SCL  → PB8 (I2C1_SCL)
SDA  → PB9 (I2C1_SDA)
```

### **I2C Addresses:**
- **MPU6050**: 0x68 (AD0=GND)
- **HMC5883L**: 0x1E
- **BMP180**: 0x77

## 🚀 **Sử dụng**

### **1. Launch Simple Version:**
```bash
ros2 launch imu imu_simple.launch.py
```

### **2. Launch with RViz:**
```bash
ros2 launch imu imu.launch.py
```

### **3. Launch with Custom Serial Port:**
```bash
ros2 launch imu imu.launch.py serial_port:=/dev/ttyUSB0
```

### **4. Run Python Node Directly:**
```bash
ros2 run imu imu_python_node.py
```

### **5. Test Compliance:**
```bash
# Run compliance tests
ros2 run imu test_imu_compliance.py

# Validate messages in real-time
ros2 run imu validate_imu_message.py
```

## 📊 **Topics**

### **Published Topics:**
- `/imu/data` (sensor_msgs/Imu): IMU data (accel, gyro, orientation)
- `/imu/mag` (sensor_msgs/MagneticField): Magnetometer data
- `/imu/temp` (sensor_msgs/Temperature): Temperature data

### **TF Frames:**
- `base_link` → `imu_link`: Transform from base to IMU

## 🔧 **Cấu hình**

### **Parameters:**
- `serial_port`: Serial port for communication (default: /dev/ttyUSB0)
- `serial_baudrate`: Serial baudrate (default: 921600)
- `use_sim_time`: Use simulation time (default: false)

### **Launch Arguments:**
```bash
ros2 launch gy87_imu gy87_imu.launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=921600 \
    use_sim_time:=false
```

## 📈 **Data Format**

### **Accelerometer (m/s²):**
- X-axis: Linear acceleration
- Y-axis: Linear acceleration
- Z-axis: Linear acceleration (gravity)

### **Gyroscope (rad/s):**
- X-axis: Angular velocity (roll)
- Y-axis: Angular velocity (pitch)
- Z-axis: Angular velocity (yaw)

### **Magnetometer (Tesla):**
- X-axis: Magnetic field strength
- Y-axis: Magnetic field strength
- Z-axis: Magnetic field strength

### **Temperature (°C):**
- MPU6050 internal temperature

## 🛠️ **Troubleshooting**

### **Common Issues:**

#### **1. Serial Connection Failed:**
```bash
# Check available ports
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyUSB0
```

#### **2. No Data Published:**
- Check STM32 firmware is running
- Verify serial port and baudrate
- Check I2C connections

#### **3. Incorrect Data:**
- Calibrate sensors
- Check for magnetic interference
- Verify sensor orientation

### **Debug Commands:**
```bash
# Check topics
ros2 topic list
ros2 topic echo /imu/data

# Check transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link imu_link

# Monitor node
ros2 node info /gy87_imu_node
```

## 📚 **API Reference**

### **C++ Node:**
- `gy87_imu_node`: Main C++ node
- Publishes: Imu, MagneticField, Temperature
- Broadcasts: TF transforms

### **Python Node:**
- `gy87_imu_python_node.py`: Main Python node
- `sensor_interface.py`: Sensor communication interface
- Features: Calibration, error handling

### **Launch Files:**
- `gy87_imu.launch.py`: Full launch with RViz
- `gy87_imu_simple.launch.py`: Simple launch

## 🔬 **Testing**

### **Unit Tests:**
```bash
cd ~/ros2_ws
colcon test --packages-select gy87_imu
```

### **Integration Tests:**
```bash
# Test with simulated data
ros2 launch gy87_imu gy87_imu_simple.launch.py

# Test with real hardware
ros2 launch gy87_imu gy87_imu.launch.py serial_port:=/dev/ttyUSB0
```

