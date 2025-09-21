# Framework Architecture Documentation

## Overview

The GY87 sensor system has been restructured into a clean, modular architecture with clear separation of concerns:

- **gy87_mpu6050**: Pure sensor library for hardware abstraction
- **framework**: Communication protocol and data frame processing
- **main**: Application logic and system integration

## Architecture Diagram

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Application   │    │   Framework     │    │  GY87 Sensors   │
│    (main.c)     │    │ (framework.c)   │    │(gy87_mpu6050.c) │
│                 │    │                 │    │                 │
│ • System Init   │◄──►│ • Data Frames   │◄──►│ • MPU6050       │
│ • 100Hz Loop    │    │ • Checksum      │    │ • HMC5883L      │
│ • Statistics    │    │ • UART Comm     │    │ • BMP180        │
│ • Error Handle  │    │ • Validation    │    │ • I2C Scanner   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Module Responsibilities

### 1. GY87_MPU6050 Library (`gy87_mpu6050.h/.c`)

**Purpose**: Pure sensor hardware abstraction layer

**Responsibilities**:
- Initialize sensors (MPU6050, HMC5883L, BMP180)
- Read raw sensor data via I2C
- Convert raw data to physical units
- Provide individual sensor access functions
- I2C device scanning and detection

**Key Functions**:
```c
// Initialization
void GY87_Init_All_Sensors(void);
void GY87_MPU6050_Init(void);
void GY87_HMC5883L_Init(void);

// Data Reading
uint8_t GY87_Read_All_Sensors(float* accel_x, float* accel_y, float* accel_z,
                              float* gyro_x, float* gyro_y, float* gyro_z,
                              float* mag_x, float* mag_y, float* mag_z);

// Individual Access
float GY87_MPU6050_Get_Ax(void);  // m/s²
float GY87_MPU6050_Get_Gx(void);  // rad/s
float GY87_HMC5883L_Get_Mx(void); // Tesla

// Utility
void GY87_I2C_Scanner(void);
```

### 2. Framework Library (`framework.h/.c`)

**Purpose**: Communication protocol and data frame processing

**Responsibilities**:
- Define data structures (SensorData_t, DataFrame_t)
- Create and validate 40-byte data frames
- Handle checksum calculation
- Manage UART communication
- Provide statistics and monitoring
- Data conversion utilities

**Key Functions**:
```c
// Data Frame Processing
uint8_t Framework_Create_DataFrame(DataFrame_t* frame, const SensorData_t* data);
uint8_t Framework_Validate_DataFrame(const DataFrame_t* frame);
void Framework_Send_DataFrame_UART(const DataFrame_t* frame);

// High-level Processing
uint8_t Framework_Process_And_Send(const SensorData_t* sensor_data);

// Statistics
void Framework_Print_Statistics(void);
void Framework_Reset_Statistics(void);

// Utilities
void Framework_Print_SensorData_UART(const SensorData_t* data);
```

### 3. Application Layer (`main.c`)

**Purpose**: System integration and application logic

**Responsibilities**:
- System initialization and configuration
- 100Hz sampling loop implementation
- Error handling and recovery
- Statistics reporting
- Integration between sensor and framework layers

## Data Flow

### 1. Initialization Sequence
```c
// 1. Hardware initialization
HAL_Init();
MX_GPIO_Init();
MX_I2C1_Init();
MX_USART2_UART_Init();

// 2. Scan I2C devices
GY87_I2C_Scanner();

// 3. Initialize all sensors
GY87_Init_All_Sensors();

// 4. Initialize framework
Framework_Reset_Statistics();
```

### 2. Runtime Data Flow (100Hz Loop)
```c
// 1. Read sensors (gy87_mpu6050 layer)
if (GY87_Read_All_Sensors(&accel_x, &accel_y, &accel_z,
                          &gyro_x, &gyro_y, &gyro_z,
                          &mag_x, &mag_y, &mag_z))
{
    // 2. Populate data structure
    SensorData_t sensor_data = {
        .accel_x = accel_x, .accel_y = accel_y, .accel_z = accel_z,
        .gyro_x = gyro_x, .gyro_y = gyro_y, .gyro_z = gyro_z,
        .mag_x = mag_x, .mag_y = mag_y, .mag_z = mag_z
    };
    
    // 3. Process and send (framework layer)
    Framework_Process_And_Send(&sensor_data);
}
```

## Data Structures

### SensorData_t (36 bytes)
```c
typedef struct {
    float accel_x, accel_y, accel_z;  // 12 bytes - m/s²
    float gyro_x, gyro_y, gyro_z;     // 12 bytes - rad/s
    float mag_x, mag_y, mag_z;        // 12 bytes - Tesla
} SensorData_t;
```

### DataFrame_t (40 bytes)
```c
typedef struct {
    uint8_t start_byte;        // 1 byte - 0xAA
    SensorData_t sensor_data;  // 36 bytes
    uint16_t checksum;         // 2 bytes
    uint8_t end_byte;          // 1 byte - 0x55
} DataFrame_t;
```

## Benefits of This Architecture

### 1. **Separation of Concerns**
- **Hardware Layer**: Pure sensor operations
- **Protocol Layer**: Communication and framing
- **Application Layer**: Business logic and integration

### 2. **Maintainability**
- Clear module boundaries
- Easy to modify one layer without affecting others
- Testable components

### 3. **Reusability**
- Sensor library can be used in other projects
- Framework can support different sensors
- Modular design allows selective usage

### 4. **Extensibility**
- Easy to add new sensors to gy87_mpu6050
- Framework can support multiple communication protocols
- Statistics and monitoring capabilities

## Usage Examples

### Basic Sensor Reading
```c
#include "gy87_mpu6050.h"

float ax, ay, az, gx, gy, gz, mx, my, mz;

if (GY87_Read_All_Sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz)) {
    printf("Accel: %.3f, %.3f, %.3f m/s²\n", ax, ay, az);
    printf("Gyro: %.3f, %.3f, %.3f rad/s\n", gx, gy, gz);
    printf("Mag: %.6f, %.6f, %.6f Tesla\n", mx, my, mz);
}
```

### Data Frame Communication
```c
#include "framework.h"

SensorData_t data = {
    .accel_x = 1.0f, .accel_y = 2.0f, .accel_z = 9.81f,
    .gyro_x = 0.1f, .gyro_y = 0.2f, .gyro_z = 0.3f,
    .mag_x = 0.000001f, .mag_y = 0.000002f, .mag_z = 0.000003f
};

// High-level processing
Framework_Process_And_Send(&data);

// Or manual control
DataFrame_t frame;
if (Framework_Create_DataFrame(&frame, &data)) {
    if (Framework_Validate_DataFrame(&frame)) {
        Framework_Send_DataFrame_UART(&frame);
    }
}
```

### Integration Example
```c
#include "gy87_mpu6050.h"
#include "framework.h"

void sensor_task_100hz(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_time >= 10) {  // 100Hz
        last_time = current_time;
        
        SensorData_t sensor_data;
        
        // Read from sensors
        if (GY87_Read_All_Sensors(&sensor_data.accel_x, &sensor_data.accel_y, &sensor_data.accel_z,
                                  &sensor_data.gyro_x, &sensor_data.gyro_y, &sensor_data.gyro_z,
                                  &sensor_data.mag_x, &sensor_data.mag_y, &sensor_data.mag_z)) {
            
            // Process and send via framework
            Framework_Process_And_Send(&sensor_data);
        }
    }
}
```

## File Structure

```
Core/
├── Inc/
│   ├── framework.h          # Communication framework header
│   ├── gy87_mpu6050.h      # Sensor library header
│   └── main.h              # Application header
└── Src/
    ├── framework.c          # Communication framework implementation
    ├── gy87_mpu6050.c      # Sensor library implementation
    └── main.c              # Application implementation
```

## Migration Guide

### From Previous Architecture
1. Replace `GY87_Read_All_Sensors(SensorData_t*)` with individual parameters
2. Use `Framework_Process_And_Send()` instead of manual frame creation
3. Include both `gy87_mpu6050.h` and `framework.h`
4. Initialize framework statistics with `Framework_Reset_Statistics()`

### Key Changes
- **Data Frame Functions**: Moved from `gy87_mpu6050` to `framework`
- **Data Structures**: Moved to `framework.h`
- **Statistics**: New monitoring capabilities in framework
- **Error Handling**: Improved validation and error reporting

This architecture provides a clean, maintainable, and extensible foundation for the GY87 sensor system while maintaining all original functionality and performance requirements.
