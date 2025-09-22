**Version**: 2.0.2
**Author**: Nhan Vo
**Update**: 2025-09-19

# Framework Architecture Documentation

## Overview

This document describes the architecture of the GY-87 IMU sensor framework, which provides a modular and extensible system for interfacing with multiple sensors on the STM32F103C8T6 microcontroller.

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
├─────────────────────────────────────────────────────────────┤
│  main.c                                                    │
│  ├── Sensor Data Processing                                │
│  ├── Timing Control                                        │
│  └── UART Communication                                    │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                   Framework Layer                          │
├─────────────────────────────────────────────────────────────┤
│  framework.h / framework.c                                 │
│  ├── Data Validation                                       │
│  ├── Frame Processing                                      │
│  ├── Error Handling                                        │
│  └── Utility Functions                                     │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                   Sensor Library Layer                     │
├─────────────────────────────────────────────────────────────┤
│  gy87_mpu6050.h / gy87_mpu6050.c                          │
│  ├── MPU6050 Driver (Accelerometer + Gyroscope)           │
│  ├── HMC5883L Driver (Magnetometer)                       │
│  ├── BMP180 Driver (Pressure + Temperature)               │
│  ├── I2C Communication                                     │
│  └── Data Conversion                                       │
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                          │
├─────────────────────────────────────────────────────────────┤
│  STM32 HAL Drivers                                         │
│  ├── I2C (PB6-SCL, PB7-SDA)                               │
│  ├── UART (PA2-TX, PA3-RX)                                │
│  ├── GPIO Configuration                                    │
│  └── System Clock                                          │
└─────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Application Layer (`main.c`)

**Responsibilities:**
- System initialization
- Main control loop
- Timing management
- UART communication

**Key Functions:**
```c
int main(void)
{
  // System initialization
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  
  // Sensor initialization
  gy87_init_all_sensors();
  
  // Main control loop
  while (1)
  {
    // Timing control
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_display_time) >= target_interval)
    {
      // Read and display sensor data
      gy87_display_all_sensors_agm(current_time - last_display_time);
      last_display_time = current_time;
    }
  }
}
```

### 2. Framework Layer (`framework.h` / `framework.c`)

**Responsibilities:**
- Data validation and integrity checking
- Frame processing and parsing
- Error handling and logging
- Utility functions

**Key Functions:**
```c
// Data validation
uint8_t framework_validate_frame(const uint8_t *frame, size_t length);

// Frame processing
uint8_t framework_receive_frame(uint8_t *buffer, size_t max_length);

// Error handling
void framework_log_error(const char *function, const char *operation, int status);
void framework_log_info(const char *message);

// Utility functions
void framework_delay_ms(uint32_t delay);
uint32_t framework_get_timestamp(void);
```

### 3. Sensor Library Layer (`gy87_mpu6050.h` / `gy87_mpu6050.c`)

**Responsibilities:**
- Sensor initialization and configuration
- Data reading and conversion
- I2C communication management
- Physical unit conversions

**Key Functions:**
```c
// Initialization
uint8_t gy87_init_all_sensors(void);
uint8_t gy87_mpu6050_init(void);
uint8_t gy87_hmc5883l_init(void);
uint8_t gy87_bmp180_init(void);

// Data reading
uint8_t gy87_read_all_sensors(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz,
                              float *mx, float *my, float *mz);

// Individual sensor data
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

// Display functions
void gy87_display_agm(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      uint32_t period_ms);
```

### 4. Hardware Layer (STM32 HAL)

**Responsibilities:**
- Low-level hardware abstraction
- Peripheral configuration
- Interrupt handling
- System clock management

**Key Components:**
- **I2C**: PB6 (SCL), PB7 (SDA) for sensor communication
- **UART**: PA2 (TX), PA3 (RX) for data transmission
- **GPIO**: Pin configuration and control
- **SysTick**: System timing and delays

## Data Flow

### 1. Initialization Flow

```
System Start
    │
    ▼
HAL_Init()
    │
    ▼
SystemClock_Config()
    │
    ▼
MX_GPIO_Init()
    │
    ▼
MX_I2C1_Init()
    │
    ▼
MX_USART2_UART_Init()
    │
    ▼
gy87_init_all_sensors()
    │
    ├── gy87_mpu6050_init()
    ├── gy87_hmc5883l_init()
    └── gy87_bmp180_init()
    │
    ▼
System Ready
```

### 2. Data Reading Flow

```
Main Loop
    │
    ▼
Check Timing (10ms interval)
    │
    ▼
gy87_read_all_sensors()
    │
    ├── gy87_mpu6050_read_data()
    │   ├── I2C Read (0x68)
    │   └── Data Conversion
    │
    ├── gy87_hmc5883l_read_data()
    │   ├── I2C Read (0x1E)
    │   └── Data Conversion
    │
    └── gy87_bmp180_read_data()
        ├── I2C Read (0x77)
        └── Data Conversion
    │
    ▼
gy87_display_all_sensors_agm()
    │
    ▼
UART Output
```

## Error Handling Architecture

### Error Types

1. **Hardware Errors**
   - I2C communication failures
   - Sensor initialization failures
   - UART transmission errors

2. **Data Errors**
   - Invalid sensor readings
   - Frame validation failures
   - Checksum mismatches

3. **Timing Errors**
   - Missed timing intervals
   - System clock issues
   - Delay function failures

### Error Handling Strategy

```c
// Error logging system
void gy87_log_error(const char* function, const char* operation, int status)
{
  char error_msg[128];
  snprintf(error_msg, sizeof(error_msg), 
           "ERROR: %s - %s failed with status: %d\r\n", 
           function, operation, status);
  UART_SendString(error_msg);
}

// Error recovery
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

## Timing Architecture

### Timing Control System

```c
// Timing variables
volatile uint32_t system_tick_counter = 0;
uint32_t last_display_time = 0;
uint32_t target_interval = 10; // 10ms target

// SysTick interrupt handler
void SysTick_Handler(void)
{
  HAL_IncTick();
  system_tick_counter++;
}

// Main loop timing
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
- **CPU Usage**: < 5% (with __NOP() optimization)

## Memory Architecture

### Memory Layout

```
Flash Memory (64KB)
├── Vector Table (0x08000000)
├── Program Code (0x08000000 - 0x0800FFFF)
│   ├── main.c
│   ├── framework.c
│   ├── gy87_mpu6050.c
│   └── HAL Drivers
└── Configuration (0x0800FF00 - 0x0800FFFF)

RAM Memory (20KB)
├── Stack (0x20005000 - 0x20005000)
├── Heap (0x20004000 - 0x20004FFF)
├── Global Variables (0x20000000 - 0x20003FFF)
│   ├── Sensor data buffers
│   ├── Timing variables
│   └── Error handling buffers
└── Local Variables (Stack)
```

### Memory Usage

- **Flash Usage**: ~45KB (70% of total)
- **RAM Usage**: ~8KB (40% of total)
- **Stack Usage**: ~2KB (10% of total)
- **Heap Usage**: ~1KB (5% of total)

## Configuration Management

### Compile-Time Configuration

```c
// Sensor addresses
#define MPU6050_ADDRESS          0x68
#define HMC5883L_ADDRESS         0x1E
#define BMP180_ADDRESS           0x77

// Timing configuration
#define TARGET_INTERVAL_MS       10
#define I2C_TIMEOUT_MS           100
#define UART_BAUDRATE            921600

// Data conversion constants
#define MPU6050_ACCEL_SCALE      16384.0f
#define MPU6050_GYRO_SCALE       131.0f
#define HMC5883L_GAIN_LSB_PER_GAUSS 1090.0f
```

### Runtime Configuration

```c
// Sensor configuration
typedef struct
{
  uint8_t mpu6050_enabled;
  uint8_t hmc5883l_enabled;
  uint8_t bmp180_enabled;
  uint8_t display_enabled;
  uint32_t target_interval;
} sensor_config_t;

// Global configuration
sensor_config_t g_sensor_config = {
  .mpu6050_enabled = 1,
  .hmc5883l_enabled = 1,
  .bmp180_enabled = 1,
  .display_enabled = 1,
  .target_interval = 10
};
```

## Extensibility

### Adding New Sensors

1. **Create sensor driver**:
```c
// In gy87_mpu6050.h
uint8_t gy87_new_sensor_init(void);
float gy87_new_sensor_get_data(void);
```

2. **Implement sensor functions**:
```c
// In gy87_mpu6050.c
uint8_t gy87_new_sensor_init(void)
{
  // Sensor initialization code
  return HAL_OK;
}
```

3. **Update main loop**:
```c
// In main.c
if (gy87_new_sensor_init() == HAL_OK)
{
  // Add to sensor reading loop
}
```

### Adding New Communication Protocols

1. **Create protocol handler**:
```c
// In framework.h
uint8_t framework_handle_new_protocol(const uint8_t *data, size_t length);
```

2. **Implement protocol functions**:
```c
// In framework.c
uint8_t framework_handle_new_protocol(const uint8_t *data, size_t length)
{
  // Protocol implementation
  return HAL_OK;
}
```

## Performance Characteristics

### Throughput

- **Sensor Reading Rate**: 100Hz
- **Data Processing Rate**: 100Hz
- **UART Transmission Rate**: 921600 baud
- **I2C Communication Rate**: 100kHz

### Latency

- **Sensor Reading Latency**: < 1ms
- **Data Processing Latency**: < 0.5ms
- **UART Transmission Latency**: < 2ms
- **Total System Latency**: < 3.5ms

### Resource Usage

- **CPU Usage**: < 5% (idle state)
- **Memory Usage**: 8KB RAM, 45KB Flash
- **Power Consumption**: < 50mA (all sensors active)

## Testing and Validation

### Unit Testing

```c
// Test individual sensor functions
void test_mpu6050_init(void)
{
  assert(gy87_mpu6050_init() == HAL_OK);
}

void test_hmc5883l_init(void)
{
  assert(gy87_hmc5883l_init() == HAL_OK);
}
```

### Integration Testing

```c
// Test complete system
void test_system_integration(void)
{
  // Initialize all sensors
  assert(gy87_init_all_sensors() == HAL_OK);
  
  // Read sensor data
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  assert(gy87_read_all_sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz) == HAL_OK);
  
  // Validate data ranges
  assert(ax >= -20.0f && ax <= 20.0f); // m/s²
  assert(gx >= -5.0f && gx <= 5.0f);   // rad/s
  assert(mx >= -0.2f && mx <= 0.2f);   // Tesla
}
```

## License

Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.

---

**Last Updated**: 2025-09-19
**Version**: 2.0.2
