# GY87 IMU Connection Diagram

## 🔌 **Hardware Connection**

### **STM32F103C8T6 Pinout:**
```
STM32F103C8T6
├── VCC  → 3.3V Power Supply
├── GND  → Ground
├── PB8  → I2C1_SCL (Serial Clock)
└── PB9  → I2C1_SDA (Serial Data)
```

### **I2C Bus Topology:**
```
STM32F103C8T6 (I2C Master)
├── MPU6050 (0x68) - 6-axis IMU
├── HMC5883L (0x1E) - 3-axis Magnetometer
└── BMP180 (0x77) - Barometric Pressure Sensor
```

### **UART Communication:**
```
STM32F103C8T6 ←→ PC/Raspberry Pi
├── TX (PA9) → RX (PC)
├── RX (PA10) ← TX (PC)
└── GND → GND
```

## 📊 **Data Flow**

### **Sensor Data Flow:**
```
Sensors → STM32 → UART → ROS2 Node → Topics
```

### **ROS2 Topics:**
```
/imu/data (sensor_msgs/Imu)
├── linear_acceleration (geometry_msgs/Vector3)
├── angular_velocity (geometry_msgs/Vector3)
└── orientation (geometry_msgs/Quaternion)

/imu/mag (sensor_msgs/MagneticField)
└── magnetic_field (geometry_msgs/Vector3)

/imu/temp (sensor_msgs/Temperature)
└── temperature (float64)
```

### **TF Frames:**
```
base_link → imu_link
```

## 🔧 **Configuration**

### **I2C Configuration:**
- **Clock Speed**: 400kHz (Fast Mode)
- **Pull-up Resistors**: 4.7kΩ
- **Voltage Level**: 3.3V

### **UART Configuration:**
- **Baudrate**: 921600
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None

### **Data Format:**
```
Axyz= ax ay az | Gxyz= gx gy gz | Mxyz= mx my mz | t=time
```

## 🚀 **Launch Configuration**

### **Simple Launch:**
```bash
ros2 launch gy87_imu gy87_imu_simple.launch.py
```

### **Full Launch with RViz:**
```bash
ros2 launch gy87_imu gy87_imu.launch.py
```

### **Custom Serial Port:**
```bash
ros2 launch gy87_imu gy87_imu.launch.py serial_port:=/dev/ttyUSB0
```

## 📈 **Performance Metrics**

### **Data Rates:**
- **MPU6050**: 100Hz (configurable)
- **HMC5883L**: 15Hz (continuous mode)
- **BMP180**: On-demand reading
- **Total I2C Read Time**: ~3ms
- **UART Transmission**: ~1ms

### **Latency:**
- **Sensor to STM32**: <1ms
- **STM32 to PC**: <1ms
- **PC to ROS2**: <1ms
- **Total Latency**: <3ms

## 🛠️ **Troubleshooting**

### **Common Issues:**

#### **1. I2C Communication:**
- Check wiring connections
- Verify pull-up resistors
- Check I2C addresses
- Use I2C scanner

#### **2. UART Communication:**
- Check serial port permissions
- Verify baudrate settings
- Check cable connections
- Test with serial terminal

#### **3. ROS2 Integration:**
- Check topic publishing
- Verify message types
- Check transform broadcasting
- Monitor node status

### **Debug Commands:**
```bash
# Check I2C devices
i2cdetect -y 1

# Check serial port
ls /dev/ttyUSB*

# Check ROS2 topics
ros2 topic list
ros2 topic echo /imu/data

# Check transforms
ros2 run tf2_tools view_frames
```

## 📚 **References**

- [STM32F103C8T6 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
- [MPU6050 Datasheet](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- [HMC5883L Datasheet](https://www.mouser.com/datasheet/2/744/HMC5883L-127305.pdf)
- [BMP180 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp180-ds000-09.pdf)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
