# IMU GY-87 10DOF Research Documentation

## 📋 **Tổng quan IMU GY-87**

### **Thông số kỹ thuật:**
- **MPU6050**: 6-axis IMU (3-axis Accelerometer + 3-axis Gyroscope)
- **HMC5883L**: 3-axis Magnetometer (Digital Compass)
- **BMP180**: Barometric Pressure Sensor
- **Tổng cộng**: 10DOF (Degrees of Freedom)

### **Giao thức kết nối:**
- **I2C Communication**: Tất cả sensors sử dụng I2C
- **MPU6050**: Address 0x68 (AD0=GND) hoặc 0x69 (AD0=VCC)
- **HMC5883L**: Address 0x1E
- **BMP180**: Address 0x77

### **Đặc điểm kỹ thuật:**

#### **MPU6050 (6-axis IMU):**
- **Accelerometer**: ±2g, ±4g, ±8g, ±16g
- **Gyroscope**: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Resolution**: 16-bit ADC
- **Sample Rate**: 1kHz (configurable)
- **Interface**: I2C, SPI

#### **HMC5883L (Magnetometer):**
- **Range**: ±1.3 Gauss (default), ±1.9, ±2.5, ±4.0, ±4.7, ±5.6, ±8.1 Gauss
- **Resolution**: 12-bit ADC
- **Sample Rate**: 15Hz (configurable)
- **Interface**: I2C only

#### **BMP180 (Barometric Pressure):**
- **Pressure Range**: 300-1100 hPa
- **Temperature Range**: -40°C to +85°C
- **Resolution**: 0.1 hPa
- **Interface**: I2C only

## 🔌 **Sơ đồ kết nối**

### **STM32F103C8T6 Pinout:**
```
VCC  → 3.3V
GND  → GND
SCL  → PB8 (I2C1_SCL)
SDA  → PB9 (I2C1_SDA)
```

### **I2C Bus Topology:**
```
STM32F103C8T6
    ├── MPU6050 (0x68)
    ├── HMC5883L (0x1E)
    └── BMP180 (0x77)
```

## 📊 **Dữ liệu đầu ra**

### **Accelerometer (m/s²):**
- X-axis: Linear acceleration
- Y-axis: Linear acceleration  
- Z-axis: Linear acceleration (gravity)

### **Gyroscope (rad/s):**
- X-axis: Angular velocity (roll)
- Y-axis: Angular velocity (pitch)
- Z-axis: Angular velocity (yaw)

### **Magnetometer (microTesla):**
- X-axis: Magnetic field strength
- Y-axis: Magnetic field strength
- Z-axis: Magnetic field strength

### **Temperature (°C):**
- MPU6050 internal temperature

## 🎯 **Ứng dụng**

### **Robotics:**
- Navigation và localization
- Attitude estimation
- Motion control

### **IoT và Wearables:**
- Activity monitoring
- Gesture recognition
- Orientation tracking

### **Aerospace:**
- Flight control systems
- Attitude determination
- Navigation systems

## 🔧 **Cấu hình I2C**

### **STM32F103C8T6 I2C1:**
- **SCL**: PB8 (Alternate Function)
- **SDA**: PB9 (Alternate Function)
- **Clock Speed**: 400kHz (Fast Mode)
- **Pull-up Resistors**: 4.7kΩ (external)

### **I2C Scanner Results:**
```
I2C Scanner Results:
Device found at address: 0x68 (MPU6050)
Device found at address: 0x1E (HMC5883L)
Device found at address: 0x77 (BMP180)
```

## 📈 **Performance Metrics**

### **Data Rate:**
- **MPU6050**: 100Hz (configurable)
- **HMC5883L**: 15Hz (continuous mode)
- **BMP180**: On-demand reading

### **Latency:**
- **I2C Communication**: ~1ms per sensor
- **Total Read Time**: ~3ms for all sensors
- **Processing Time**: ~1ms

### **Accuracy:**
- **Accelerometer**: ±0.1g
- **Gyroscope**: ±0.1°/s
- **Magnetometer**: ±0.1° heading
- **Temperature**: ±1°C

## 🛠️ **Troubleshooting**

### **Common Issues:**
1. **I2C Communication Failures**
   - Check wiring connections
   - Verify pull-up resistors
   - Check I2C addresses

2. **Magnetometer Data Issues**
   - Calibrate for hard/soft iron effects
   - Check for magnetic interference
   - Verify gain settings

3. **Temperature Drift**
   - Implement temperature compensation
   - Use complementary filter
   - Regular calibration

## 📚 **References**

- [MPU6050 Datasheet](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- [HMC5883L Datasheet](https://www.mouser.com/datasheet/2/744/HMC5883L-127305.pdf)
- [BMP180 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp180-ds000-09.pdf)
- [STM32F103C8T6 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
