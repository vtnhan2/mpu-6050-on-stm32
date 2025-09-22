**Version**: 2.0.2
**Author**: Nhan Vo
**Update**: 2025-09-19

# MPU6050 + HMC5883L + BMP180 IMU Sensor Library

## Overview

This project provides a comprehensive library for interfacing with the GY-87 10DOF IMU module, which combines three high-precision sensors:

- **MPU6050**: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
- **HMC5883L**: 3-axis digital magnetometer/compass
- **BMP180**: Barometric pressure sensor

The library is designed for STM32F103C8T6 microcontrollers and provides real-time sensor data acquisition with precise timing control.

## Hardware Specifications

### MPU6050 (6-axis IMU)
- **Accelerometer**: ±2g, ±4g, ±8g, ±16g ranges
- **Gyroscope**: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s ranges
- **Communication**: I2C (Address: 0x68)
- **Resolution**: 16-bit ADC
- **Update Rate**: Up to 1kHz

### HMC5883L (3-axis Magnetometer)
- **Measurement Range**: ±1.3 Gauss (130 microTesla)
- **Resolution**: 0.73 mGauss per LSB
- **Communication**: I2C (Address: 0x1E)
- **Update Rate**: Up to 160Hz
- **Current Consumption**: 1.0mA (continuous mode)

### BMP180 (Barometric Pressure Sensor)
- **Pressure Range**: 300-1100 hPa
- **Temperature Range**: -40°C to +85°C
- **Communication**: I2C (Address: 0x77)
- **Resolution**: 0.1 hPa (pressure), 0.1°C (temperature)

## Features

- **Real-time Data Acquisition**: 100Hz sensor reading rate
- **Precise Timing Control**: 10ms interval with counter-based timing
- **Multiple Output Formats**: Raw data, physical units, formatted display
- **Error Handling**: Comprehensive error detection and reporting
- **UART Communication**: High-speed data transmission (921600 baud)
- **Modular Design**: Easy integration and customization

## Project Structure

```
MPU6050_Read/
├── Core/
│   ├── Inc/
│   │   ├── framework.h          # Framework utilities
│   │   ├── gy87_mpu6050.h      # Main sensor library header
│   │   ├── main.h              # Main application header
│   │   └── ...
│   └── Src/
│       ├── framework.c         # Framework implementation
│       ├── gy87_mpu6050.c     # Main sensor library
│       ├── main.c              # Main application
│       └── ...
├── Debug/                      # Build output directory
├── Drivers/                    # STM32 HAL drivers
├── ros2_imu/                   # ROS2 package for IMU data
└── README.md                   # This file
```

## Quick Start

### 1. Hardware Connection

| STM32F103C8T6 | GY-87 Module | Description |
|---------------|--------------|-------------|
| PB6 (SCL)     | SCL          | I2C Clock   |
| PB7 (SDA)     | SDA          | I2C Data    |
| 3.3V          | VCC          | Power       |
| GND           | GND          | Ground      |
| PA2 (TX)      | RX           | UART TX     |
| PA3 (RX)      | TX           | UART RX     |

### 2. Build and Flash

```bash
# Build the project
make clean && make

# Flash to STM32 (using ST-Link)
st-flash write Debug/MPU6050_Read.hex 0x8000000
```

### 3. Monitor Output

Connect to UART2 at **921600 baud** to see sensor data:

```
Axyz= 0.697 -0.536 9.578 m/s² | Gxyz= -0.039 -0.032 0.011 rad/s | Mxyz= -0.0600 0.0093 0.0606 microTesla | t=10ms
```

## API Reference

### Initialization Functions

```c
// Initialize all sensors
uint8_t gy87_init_all_sensors(void);

// Initialize individual sensors
uint8_t gy87_mpu6050_init(void);
uint8_t gy87_hmc5883l_init(void);
uint8_t gy87_bmp180_init(void);
```

### Data Reading Functions

```c
// Read all sensor data
uint8_t gy87_read_all_sensors(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz,
                              float *mx, float *my, float *mz);

// Read individual sensor data
float gy87_mpu6050_get_ax(void);
float gy87_mpu6050_get_ay(void);
float gy87_mpu6050_get_az(void);
float gy87_mpu6050_get_gx(void);
float gy87_mpu6050_get_gy(void);
float gy87_mpu6050_get_gz(void);
float gy87_mpu6050_get_temperature(void);

float gy87_hmc5883l_get_mx(void);
float gy87_hmc5883l_get_my(void);
float gy87_hmc5883l_get_mz(void);
```

### Display Functions

```c
// Display formatted sensor data
void gy87_display_agm(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      uint32_t period_ms);

// Display all sensors with timing
void gy87_display_all_sensors_agm(uint32_t period_ms);
```

## Data Units

- **Accelerometer**: m/s² (meters per second squared)
- **Gyroscope**: rad/s (radians per second)
- **Magnetometer**: microTesla (microTesla)
- **Temperature**: °C (Celsius)
- **Pressure**: hPa (hectopascal)

## Timing Control

The library uses a counter-based timing system for precise 10ms intervals:

```c
// Timing variables
uint32_t last_display_time = 0;
uint32_t target_interval = 10; // 10ms target

// Main loop timing
uint32_t current_time = HAL_GetTick();
if ((current_time - last_display_time) >= target_interval)
{
  // Read and display sensor data
  gy87_display_all_sensors_agm(current_time - last_display_time);
  last_display_time = current_time;
}
```

## Error Handling

The library provides comprehensive error handling:

```c
// Error logging functions
void gy87_log_error(const char* function, const char* operation, int status);
void gy87_log_info(const char* message);

// Example usage
if (HAL_I2C_IsDeviceReady(&hi2c1, HMC5883L_ADDRESS, 3, 100) != HAL_OK)
{
  gy87_log_error("gy87_hmc5883l_init", "device_ready_check", HAL_TIMEOUT);
}
```

## ROS2 Integration

The project includes a complete ROS2 package for IMU data publishing:

```bash
# Build ROS2 package
cd ros2_imu
colcon build

# Run IMU node
ros2 launch imu imu.launch.py

# Monitor topics
ros2 topic echo /imu/data
ros2 topic echo /imu/magnetic_field
ros2 topic echo /imu/temperature
```

## Troubleshooting

### Common Issues

1. **No sensor data**: Check I2C connections and addresses
2. **Incorrect values**: Verify sensor initialization sequence
3. **Timing issues**: Ensure proper SysTick configuration
4. **UART problems**: Check baud rate and pin configuration

### Debug Tools

```c
// I2C scanner
void gy87_scan_i2c_devices(void);

// Test individual sensors
void gy87_test_mpu6050_only(void);
void gy87_test_hmc5883l_only(void);
void gy87_test_bmp180_only(void);
```

## License

Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Support

For issues and questions:
- GitHub: [@vtnhan2/mpu-6050-on-stm32](https://github.com/vtnhan2/mpu-6050-on-stm32)
- Email: [Your Email]

---

**Last Updated**: 2025-09-19
**Version**: 2.0.2
