**Version**: 2.0.2
**Author**: Nhan Vo
**Update**: 2025-09-19

# GY87 Library Usage Guide

## Overview

This guide provides comprehensive documentation for using the GY87 sensor library, which interfaces with the GY-87 10DOF IMU module containing MPU6050, HMC5883L, and BMP180 sensors.

## Library Structure

### Header Files

```c
#include "gy87_mpu6050.h"    // Main sensor library
#include "framework.h"        // Framework utilities
#include "main.h"             // System configuration
```

### Source Files

- `gy87_mpu6050.c` - Main sensor library implementation
- `framework.c` - Framework utilities implementation
- `main.c` - Application main loop

## Initialization

### System Initialization

```c
int main(void)
{
  // Initialize HAL and system clock
  HAL_Init();
  SystemClock_Config();
  
  // Initialize peripherals
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  
  // Initialize all sensors
  if (gy87_init_all_sensors() == HAL_OK)
  {
    gy87_log_info("All sensors initialized successfully!");
  }
  else
  {
    gy87_log_error("main", "sensor_initialization", HAL_ERROR);
  }
  
  // Main loop
  while (1)
  {
    // Sensor reading and display
  }
}
```

### Individual Sensor Initialization

```c
// Initialize MPU6050 (accelerometer + gyroscope)
if (gy87_mpu6050_init() == HAL_OK)
{
  gy87_log_info("MPU6050 initialized successfully!");
}

// Initialize HMC5883L (magnetometer)
if (gy87_hmc5883l_init() == HAL_OK)
{
  gy87_log_info("HMC5883L initialized successfully!");
}

// Initialize BMP180 (pressure + temperature)
if (gy87_bmp180_init() == HAL_OK)
{
  gy87_log_info("BMP180 initialized successfully!");
}
```

## Sensor Data Reading

### Read All Sensors

```c
float accel_x, accel_y, accel_z;    // Accelerometer data (m/s²)
float gyro_x, gyro_y, gyro_z;       // Gyroscope data (rad/s)
float mag_x, mag_y, mag_z;          // Magnetometer data (microTesla)

if (gy87_read_all_sensors(&accel_x, &accel_y, &accel_z,
                          &gyro_x, &gyro_y, &gyro_z,
                          &mag_x, &mag_y, &mag_z) == HAL_OK)
{
  // Process sensor data
  gy87_display_agm(accel_x, accel_y, accel_z,
                   gyro_x, gyro_y, gyro_z,
                   mag_x, mag_y, mag_z, 10);
}
```

### Read Individual Sensors

#### Accelerometer Data

```c
float ax = gy87_mpu6050_get_ax();  // X-axis acceleration (m/s²)
float ay = gy87_mpu6050_get_ay();  // Y-axis acceleration (m/s²)
float az = gy87_mpu6050_get_az();  // Z-axis acceleration (m/s²)

// Example output: ax = 0.697, ay = -0.536, az = 9.578
```

#### Gyroscope Data

```c
float gx = gy87_mpu6050_get_gx();  // X-axis angular velocity (rad/s)
float gy = gy87_mpu6050_get_gy();  // Y-axis angular velocity (rad/s)
float gz = gy87_mpu6050_get_gz();  // Z-axis angular velocity (rad/s)

// Example output: gx = -0.039, gy = -0.032, gz = 0.011
```

#### Magnetometer Data

```c
float mx = gy87_hmc5883l_get_mx(); // X-axis magnetic field (microTesla)
float my = gy87_hmc5883l_get_my(); // Y-axis magnetic field (microTesla)
float mz = gy87_hmc5883l_get_mz(); // Z-axis magnetic field (microTesla)

// Example output: mx = -0.0600, my = 0.0093, mz = 0.0606
```

#### Temperature Data

```c
float temperature = gy87_mpu6050_get_temperature(); // Temperature (°C)

// Example output: temperature = 25.3
```

## Data Display Functions

### Basic Display

```c
// Display all sensor data with timing
gy87_display_all_sensors_agm(10); // 10ms period

// Output format:
// Axyz= 0.697 -0.536 9.578 m/s² | Gxyz= -0.039 -0.032 0.011 rad/s | Mxyz= -0.0600 0.0093 0.0606 microTesla | t=10ms
```

### Custom Display

```c
// Display specific sensor data
gy87_display_agm(accel_x, accel_y, accel_z,
                 gyro_x, gyro_y, gyro_z,
                 mag_x, mag_y, mag_z,
                 period_ms);
```

### Formatted Display

```c
// Display with custom formatting
gy87_display_formatted_data(accel_x, accel_y, accel_z,
                           gyro_x, gyro_y, gyro_z,
                           mag_x, mag_y, mag_z,
                           period_ms);
```

## I2C Communication

### I2C Scanner

```c
// Scan for I2C devices
gy87_scan_i2c_devices();

// Expected output for GY-87 module:
// === I2C Scanner Started ===
// Scanning I2C bus for devices...
// Device found at address: 0x1E (30)    # HMC5883L
// Device found at address: 0x68 (104)   # MPU6050
// Device found at address: 0x77 (119)   # BMP180
// Total devices found: 3
// === I2C Scanner Completed ===
```

### I2C Device Detection

```c
// Check if specific device is ready
if (gy87_i2c_is_device_ready(MPU6050_ADDRESS) == HAL_OK)
{
  gy87_log_info("MPU6050 is ready");
}
else
{
  gy87_log_error("gy87_i2c_is_device_ready", "mpu6050_check", HAL_ERROR);
}
```

## Error Handling

### Error Logging

```c
// Log error with function, operation, and status
gy87_log_error("gy87_mpu6050_init", "device_ready_check", HAL_TIMEOUT);

// Log informational message
gy87_log_info("Sensor initialization completed");
```

### Error Recovery

```c
// Handle sensor errors
uint8_t gy87_handle_sensor_error(uint8_t sensor_id, int error_code)
{
  switch (error_code)
  {
    case HAL_I2C_ERROR_TIMEOUT:
      // Retry I2C operation
      return gy87_retry_i2c_operation(sensor_id);
      
    case HAL_I2C_ERROR_AF:
      // Address not found, reinitialize
      return gy87_reinitialize_sensor(sensor_id);
      
    default:
      // Log error and continue
      gy87_log_error("gy87_handle_sensor_error", "unknown_error", error_code);
      return 0;
  }
}
```

## Timing Control

### Precise Timing

```c
// Timing variables
uint32_t last_display_time = 0;
uint32_t target_interval = 10; // 10ms target

// Main loop with precise timing
while (1)
{
  uint32_t current_time = HAL_GetTick();
  
  if ((current_time - last_display_time) >= target_interval)
  {
    uint32_t actual_period = current_time - last_display_time;
    
    // Read and display sensor data
    gy87_display_all_sensors_agm(actual_period);
    
    last_display_time = current_time;
  }
  
  __NOP(); // Minimal CPU usage
}
```

### Timing Characteristics

- **Target Interval**: 10ms (100Hz)
- **Actual Interval**: 10ms ± 1ms
- **Jitter**: < 1ms
- **CPU Usage**: < 5%

## Data Units and Ranges

### Accelerometer

- **Unit**: m/s² (meters per second squared)
- **Range**: ±19.62 m/s² (±2g)
- **Resolution**: 0.061 mg
- **Example**: 9.81 m/s² = 1g (gravity)

### Gyroscope

- **Unit**: rad/s (radians per second)
- **Range**: ±4.36 rad/s (±250°/s)
- **Resolution**: 0.0076°/s
- **Example**: 0.017 rad/s = 1°/s

### Magnetometer

- **Unit**: microTesla (μT)
- **Range**: ±130 microTesla (±1.3 Gauss)
- **Resolution**: 0.73 mGauss
- **Example**: 25 microTesla = 0.25 Gauss

### Temperature

- **Unit**: °C (Celsius)
- **Range**: -40°C to +85°C
- **Resolution**: 0.1°C
- **Example**: 25.3°C = room temperature

## Configuration Constants

### Sensor Addresses

```c
#define MPU6050_ADDRESS          0x68  // MPU6050 I2C address
#define HMC5883L_ADDRESS         0x1E  // HMC5883L I2C address
#define BMP180_ADDRESS           0x77  // BMP180 I2C address
```

### Timing Configuration

```c
#define TARGET_INTERVAL_MS       10    // Target display interval
#define I2C_TIMEOUT_MS           100   // I2C timeout
#define UART_BAUDRATE            921600 // UART baud rate
```

### Data Conversion Constants

```c
#define MPU6050_ACCEL_SCALE      16384.0f  // LSB per g for ±2g range
#define MPU6050_GYRO_SCALE       131.0f    // LSB per deg/s for ±250deg/s range
#define HMC5883L_GAIN_LSB_PER_GAUSS 1090.0f // LSB per Gauss for 1.3Ga gain
#define HMC5883L_GAUSS_TO_MICROTESLA 0.1f   // Conversion factor
```

## Testing Functions

### Individual Sensor Testing

```c
// Test MPU6050 only
gy87_test_mpu6050_only();

// Test HMC5883L only
gy87_test_hmc5883l_only();

// Test BMP180 only
gy87_test_bmp180_only();
```

### Test Output Examples

#### MPU6050 Test
```
=== MPU6050 Test ===
MPU6050 Raw Data: 0A 1B 2C 3D 4E 5F 60 71 82 93 A4 B5 C6 D7 E8 F9
MPU6050 Test Result: Axyz= 0.697 -0.536 9.578 m/s² | Gxyz= -0.039 -0.032 0.011 rad/s | T=25.3°C
=== End MPU6050 Test ===
```

#### HMC5883L Test
```
=== HMC5883L Test ===
HMC5883L Raw Data: FD 79 02 BD 00 66
HMC5883L Raw Values: X=-647 Y=102 Z=701
HMC5883L Test Result: X=-0.0600 Y=0.0093 Z=0.0606 microTesla
=== End HMC5883L Test ===
```

## Usage Examples

### Basic Sensor Reading

```c
#include "gy87_mpu6050.h"

int main(void)
{
  // Initialize system
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  
  // Initialize sensors
  gy87_init_all_sensors();
  
  // Main loop
  while (1)
  {
    // Read all sensor data
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    if (gy87_read_all_sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz))
    {
      // Display sensor data
      gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, 10);
    }
    
    HAL_Delay(10);
  }
}
```

### Advanced Sensor Processing

```c
#include "gy87_mpu6050.h"

int main(void)
{
  // Initialize system
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  
  // Initialize sensors
  gy87_init_all_sensors();
  
  // Timing variables
  uint32_t last_display_time = 0;
  uint32_t target_interval = 10;
  
  // Main loop with precise timing
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    
    if ((current_time - last_display_time) >= target_interval)
    {
      uint32_t actual_period = current_time - last_display_time;
      
      // Read sensor data
      float ax, ay, az, gx, gy, gz, mx, my, mz;
      
      if (gy87_read_all_sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz))
      {
        // Process sensor data
        float magnitude = sqrt(ax*ax + ay*ay + az*az);
        float temperature = gy87_mpu6050_get_temperature();
        
        // Display processed data
        gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, actual_period);
        
        // Additional processing
        if (magnitude > 15.0f) // High acceleration detected
        {
          gy87_log_info("High acceleration detected!");
        }
      }
      
      last_display_time = current_time;
    }
    
    __NOP(); // Minimal CPU usage
  }
}
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
gy87_scan_i2c_devices();

// Test individual sensors
gy87_test_mpu6050_only();
gy87_test_hmc5883l_only();
gy87_test_bmp180_only();

// Error logging
gy87_log_error("function_name", "operation", status);
gy87_log_info("message");
```

## License

Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.

---

**Last Updated**: 2025-09-19
**Version**: 2.0.2