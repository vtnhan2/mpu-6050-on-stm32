# GY87_MPU6050 Library Usage Guide

## Overview

The GY87_MPU6050 library provides comprehensive support for the GY-87 10DOF sensor module, implementing the exact data format specification required for Jetson ↔ STM32 communication.

## Data Format Specification

### Frame Structure (40 bytes total)
```
┌─────────┬──────────────┬─────────────┬─────────┐
│ START   │ DATA PAYLOAD │ CHECKSUM    │ END     │
│ 1 byte  │ 36 bytes     │ 2 bytes     │ 1 byte  │
│ (0xAA)  │ SensorData_t │ uint16_t    │ (0x55)  │
└─────────┴──────────────┴─────────────┴─────────┘
```

### Sensor Data Structure (36 bytes)
```c
typedef struct {
    // Accelerometer data (12 bytes) - m/s²
    float accel_x;  // 4 bytes IEEE 754
    float accel_y;  // 4 bytes IEEE 754
    float accel_z;  // 4 bytes IEEE 754
    
    // Gyroscope data (12 bytes) - rad/s
    float gyro_x;   // 4 bytes IEEE 754
    float gyro_y;   // 4 bytes IEEE 754
    float gyro_z;   // 4 bytes IEEE 754
    
    // Magnetometer data (12 bytes) - Tesla
    float mag_x;    // 4 bytes IEEE 754
    float mag_y;    // 4 bytes IEEE 754
    float mag_z;    // 4 bytes IEEE 754
} SensorData_t;
```

## Quick Start

### 1. Initialization
```c
#include "gy87_mpu6050.h"

int main(void) {
    // Initialize HAL and peripherals
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    
    // Scan I2C devices and initialize sensors
    GY87_I2C_Scanner();
    GY87_Init_All_Sensors();
    
    // Main loop...
}
```

### 2. Basic Data Reading
```c
SensorData_t sensor_data;

if (GY87_Read_All_Sensors(&sensor_data)) {
    // Data successfully read
    printf("Accel X: %.3f m/s²\\n", sensor_data.accel_x);
    printf("Gyro X: %.3f rad/s\\n", sensor_data.gyro_x);
    printf("Mag X: %.6f Tesla\\n", sensor_data.mag_x);
}
```

### 3. Data Frame Creation and Transmission
```c
SensorData_t sensor_data;
DataFrame_t data_frame;

// Read sensors
if (GY87_Read_All_Sensors(&sensor_data)) {
    // Create 40-byte frame
    if (GY87_Create_DataFrame(&data_frame, &sensor_data)) {
        // Send binary frame via UART
        GY87_Send_DataFrame_UART(&data_frame);
    }
}
```

### 4. High-Frequency Sampling (100Hz)
```c
uint32_t last_time = 0;

while (1) {
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_time >= 10) { // 100Hz = 10ms period
        last_time = current_time;
        
        SensorData_t sensor_data;
        DataFrame_t data_frame;
        
        if (GY87_Read_All_Sensors(&sensor_data)) {
            if (GY87_Create_DataFrame(&data_frame, &sensor_data)) {
                GY87_Send_DataFrame_UART(&data_frame);
            }
        }
    }
}
```

## API Reference

### Initialization Functions
```c
void GY87_MPU6050_Init(void);          // Initialize MPU6050 only
void GY87_HMC5883L_Init(void);         // Initialize HMC5883L only
void GY87_BMP180_Init(void);           // Initialize BMP180 only
void GY87_Init_All_Sensors(void);      // Initialize all sensors
```

### Data Reading Functions
```c
uint8_t GY87_Read_All_Sensors(SensorData_t* sensor_data);  // Read all sensors
uint8_t GY87_MPU6050_Read_Data(void);                      // Read MPU6050 only
uint8_t GY87_HMC5883L_Read_Data(void);                     // Read HMC5883L only
```

### Data Frame Functions
```c
uint8_t GY87_Create_DataFrame(DataFrame_t* frame, const SensorData_t* sensor_data);
uint16_t GY87_Calculate_Checksum(const SensorData_t* data);
uint8_t GY87_Validate_DataFrame(const DataFrame_t* frame);
void GY87_Send_DataFrame_UART(const DataFrame_t* frame);
void GY87_Print_SensorData_UART(const SensorData_t* data);
```

### Individual Sensor Access (Legacy)
```c
// MPU6050 Accelerometer (m/s²)
float GY87_MPU6050_Get_Ax(void);
float GY87_MPU6050_Get_Ay(void);
float GY87_MPU6050_Get_Az(void);

// MPU6050 Gyroscope (rad/s)
float GY87_MPU6050_Get_Gx(void);
float GY87_MPU6050_Get_Gy(void);
float GY87_MPU6050_Get_Gz(void);

// HMC5883L Magnetometer (Tesla)
float GY87_HMC5883L_Get_Mx(void);
float GY87_HMC5883L_Get_My(void);
float GY87_HMC5883L_Get_Mz(void);

// Temperature
float GY87_MPU6050_Get_Temperature(void);
```

## Data Conversion Details

### Accelerometer (MPU6050)
- **Range**: ±2g (default)
- **Resolution**: 16-bit
- **Conversion**: `(raw_value / 16384) * 9.81` → m/s²
- **LSB Sensitivity**: 16384 LSB/g

### Gyroscope (MPU6050)
- **Range**: ±250°/s (default)
- **Resolution**: 16-bit
- **Conversion**: `(raw_value / 131) * (π/180)` → rad/s
- **LSB Sensitivity**: 131 LSB/(°/s)

### Magnetometer (HMC5883L)
- **Range**: Up to ±8 Gauss (gain configurable)
- **Sensitivity (default gain 1.3 Ga)**: 1090 LSB/Gauss
- **Conversion**: `raw_value / 1090 * 0.0001` → Tesla
- **I2C Address**: 0x1E

## I2C Addresses

| Sensor | I2C Address | Notes |
|--------|-------------|-------|
| MPU6050 | 0x68 | AD0=GND (default), 0x69 if AD0=VCC |
| HMC5883L | 0x1E | Fixed address |
| BMP180 | 0x77 | Fixed address |

## Performance Specifications

- **Sample Rate**: 100 Hz (10ms period)
- **Data Latency**: < 5 ms
- **UART Speed**: 921600 baud (configurable)
- **I2C Speed**: 100 kHz (Standard Mode)
- **Frame Size**: 40 bytes per transmission

## Error Handling

The library provides comprehensive error checking:

```c
// Check sensor initialization
if (!GY87_Read_All_Sensors(&sensor_data)) {
    UART_SendString("Error: Failed to read sensors!\\n");
    return;
}

// Validate data frame
if (!GY87_Validate_DataFrame(&data_frame)) {
    UART_SendString("Error: Invalid data frame!\\n");
    return;
}
```

## Debugging

### I2C Scanner
```c
GY87_I2C_Scanner(); // Automatically scans and reports all I2C devices
```

### Human-Readable Output
```c
GY87_Print_SensorData_UART(&sensor_data); // Print formatted sensor data
```

### Expected I2C Scanner Output
```
=== I2C Scanner Started ===
Scanning I2C bus for devices...
Device found at address: 0x1E (30)    # HMC5883L
Device found at address: 0x68 (104)   # MPU6050
Device found at address: 0x77 (119)   # BMP180
Total devices found: 3
=== I2C Scanner Completed ===
```

## Integration with Jetson

The library outputs binary data frames that can be directly consumed by Jetson:

```python
# Python receiver example
import serial

ser = serial.Serial('/dev/ttyUSB0', 921600)

while True:
    # Read 40-byte frame
    frame = ser.read(40)
    
    if len(frame) == 40:
        # Parse frame
        start_byte = frame[0]
        sensor_data = frame[1:37]  # 36 bytes
        checksum = int.from_bytes(frame[37:39], 'little')
        end_byte = frame[39]
        
        if start_byte == 0xAA and end_byte == 0x55:
            # Valid frame - process sensor_data
            process_sensor_data(sensor_data)
```

---

**Note**: This library fully implements the specified data format requirements for seamless Jetson ↔ STM32 communication at 100Hz with IEEE 754 float precision.
