**Version**: 2.0.2
**Author**: Nhan Vo
**Update**: 2025-09-19

# UART Display Functions Guide

## Overview

This guide provides comprehensive documentation for the UART display functions used in the GY87 IMU sensor system, including data formatting, output options, and configuration parameters.

## UART Configuration

### Hardware Connection

```
STM32F103C8T6    USB-UART Converter
PA2 (TX)     →   RX
PA3 (RX)     →   TX
GND          →   GND
3.3V         →   VCC (if needed)
```

### UART Settings

- **Baud Rate**: 921600
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### UART Initialization

```c
// UART2 initialization
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}
```

## Display Functions

### 1. Basic Sensor Data Display

#### Function: `gy87_display_agm()`

**Purpose**: Display accelerometer, gyroscope, and magnetometer data in a formatted string.

**Syntax**:
```c
void gy87_display_agm(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      uint32_t period_ms);
```

**Parameters**:
- `ax, ay, az`: Accelerometer data (m/s²)
- `gx, gy, gz`: Gyroscope data (rad/s)
- `mx, my, mz`: Magnetometer data (microTesla)
- `period_ms`: Timing period in milliseconds

**Output Format**:
```
Axyz= 0.697 -0.536 9.578 m/s² | Gxyz= -0.039 -0.032 0.011 rad/s | Mxyz= -0.0600 0.0093 0.0606 microTesla | t=10ms
```

**Example Usage**:
```c
float accel_x = 0.697f, accel_y = -0.536f, accel_z = 9.578f;
float gyro_x = -0.039f, gyro_y = -0.032f, gyro_z = 0.011f;
float mag_x = -0.0600f, mag_y = 0.0093f, mag_z = 0.0606f;

gy87_display_agm(accel_x, accel_y, accel_z,
                 gyro_x, gyro_y, gyro_z,
                 mag_x, mag_y, mag_z, 10);
```

### 2. All Sensors Display

#### Function: `gy87_display_all_sensors_agm()`

**Purpose**: Read and display all sensor data with timing control.

**Syntax**:
```c
void gy87_display_all_sensors_agm(uint32_t period_ms);
```

**Parameters**:
- `period_ms`: Timing period in milliseconds

**Output Format**:
```
Axyz= 0.697 -0.536 9.578 m/s² | Gxyz= -0.039 -0.032 0.011 rad/s | Mxyz= -0.0600 0.0093 0.0606 microTesla | t=10ms
```

**Example Usage**:
```c
// Display all sensors with 10ms timing
gy87_display_all_sensors_agm(10);
```

### 3. Formatted Data Display

#### Function: `gy87_display_formatted_data()`

**Purpose**: Display sensor data with custom formatting and additional information.

**Syntax**:
```c
void gy87_display_formatted_data(float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float mx, float my, float mz,
                                 uint32_t period_ms);
```

**Parameters**:
- `ax, ay, az`: Accelerometer data (m/s²)
- `gx, gy, gz`: Gyroscope data (rad/s)
- `mx, my, mz`: Magnetometer data (microTesla)
- `period_ms`: Timing period in milliseconds

**Output Format**:
```
=== Sensor Data ===
Accelerometer: X=0.697, Y=-0.536, Z=9.578 m/s²
Gyroscope: X=-0.039, Y=-0.032, Z=0.011 rad/s
Magnetometer: X=-0.0600, Y=0.0093, Z=0.0606 microTesla
Period: 10ms
==================
```

**Example Usage**:
```c
gy87_display_formatted_data(accel_x, accel_y, accel_z,
                           gyro_x, gyro_y, gyro_z,
                           mag_x, mag_y, mag_z, 10);
```

### 4. Individual Sensor Display

#### Function: `gy87_display_accelerometer()`

**Purpose**: Display only accelerometer data.

**Syntax**:
```c
void gy87_display_accelerometer(float ax, float ay, float az, uint32_t period_ms);
```

**Output Format**:
```
Accelerometer: X=0.697, Y=-0.536, Z=9.578 m/s² | t=10ms
```

#### Function: `gy87_display_gyroscope()`

**Purpose**: Display only gyroscope data.

**Syntax**:
```c
void gy87_display_gyroscope(float gx, float gy, float gz, uint32_t period_ms);
```

**Output Format**:
```
Gyroscope: X=-0.039, Y=-0.032, Z=0.011 rad/s | t=10ms
```

#### Function: `gy87_display_magnetometer()`

**Purpose**: Display only magnetometer data.

**Syntax**:
```c
void gy87_display_magnetometer(float mx, float my, float mz, uint32_t period_ms);
```

**Output Format**:
```
Magnetometer: X=-0.0600, Y=0.0093, Z=0.0606 microTesla | t=10ms
```

## Data Formatting

### Number Formatting

#### Float Precision

```c
// Accelerometer and Gyroscope: 3 decimal places
printf("%.3f", value);

// Magnetometer: 4 decimal places
printf("%.4f", value);

// Temperature: 1 decimal place
printf("%.1f", value);
```

#### Scientific Notation

```c
// For very small values
printf("%.2e", value);

// Example: 1.23e-06
```

### Unit Display

#### Standard Units

- **Accelerometer**: m/s² (meters per second squared)
- **Gyroscope**: rad/s (radians per second)
- **Magnetometer**: microTesla (microTesla)
- **Temperature**: °C (Celsius)
- **Pressure**: hPa (hectopascal)

#### Unit Conversion

```c
// Convert Gauss to microTesla
float microtesla = gauss * 0.1f;

// Convert degrees to radians
float radians = degrees * (M_PI / 180.0f);

// Convert g to m/s²
float ms2 = g * 9.81f;
```

## Output Control

### Display Rate Control

```c
// Control display rate
uint32_t last_display_time = 0;
uint32_t target_interval = 10; // 10ms = 100Hz

while (1)
{
  uint32_t current_time = HAL_GetTick();
  
  if ((current_time - last_display_time) >= target_interval)
  {
    gy87_display_all_sensors_agm(current_time - last_display_time);
    last_display_time = current_time;
  }
  
  __NOP(); // Minimal CPU usage
}
```

### Display Enable/Disable

```c
// Global display control
uint8_t display_enabled = 1;

void gy87_display_all_sensors_agm(uint32_t period_ms)
{
  if (!display_enabled) return;
  
  // Display sensor data
  gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}

// Toggle display
void gy87_toggle_display(void)
{
  display_enabled = !display_enabled;
  gy87_log_info("Display %s", display_enabled ? "enabled" : "disabled");
}
```

### Output Buffering

```c
// Buffer for UART output
#define UART_BUFFER_SIZE 256
char uart_buffer[UART_BUFFER_SIZE];

void gy87_display_agm(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      uint32_t period_ms)
{
  // Format data into buffer
  snprintf(uart_buffer, UART_BUFFER_SIZE,
           "Axyz= %.3f %.3f %.3f m/s² | Gxyz= %.3f %.3f %.3f rad/s | Mxyz= %.4f %.4f %.4f microTesla | t=%lums\r\n",
           ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
  
  // Send buffer via UART
  UART_SendString(uart_buffer);
}
```

## Error Handling

### UART Error Handling

```c
// UART transmission with error handling
HAL_StatusTypeDef gy87_uart_transmit(const char* data, uint32_t timeout)
{
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), timeout);
  
  if (status != HAL_OK)
  {
    gy87_log_error("gy87_uart_transmit", "transmission_failed", status);
  }
  
  return status;
}
```

### Buffer Overflow Protection

```c
// Safe string formatting
void gy87_safe_sprintf(char* buffer, size_t buffer_size, const char* format, ...)
{
  va_list args;
  va_start(args, format);
  
  int result = vsnprintf(buffer, buffer_size, format, args);
  
  if (result >= buffer_size)
  {
    gy87_log_error("gy87_safe_sprintf", "buffer_overflow", result);
    buffer[buffer_size - 1] = '\0'; // Ensure null termination
  }
  
  va_end(args);
}
```

## Performance Optimization

### Efficient String Formatting

```c
// Optimized display function
void gy87_display_agm_optimized(float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float mx, float my, float mz,
                                uint32_t period_ms)
{
  // Use static buffer to avoid stack allocation
  static char buffer[128];
  
  // Format directly to buffer
  int len = snprintf(buffer, sizeof(buffer),
                     "Axyz= %.3f %.3f %.3f m/s² | Gxyz= %.3f %.3f %.3f rad/s | Mxyz= %.4f %.4f %.4f microTesla | t=%lums\r\n",
                     ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
  
  // Send via UART
  if (len > 0 && len < sizeof(buffer))
  {
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
  }
}
```

### Conditional Compilation

```c
// Compile-time display control
#define DISPLAY_ENABLED 1
#define DISPLAY_DEBUG 0

#if DISPLAY_ENABLED
  #define DISPLAY_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, t) \
    gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, t)
#else
  #define DISPLAY_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, t)
#endif

#if DISPLAY_DEBUG
  #define DEBUG_DISPLAY(msg) gy87_log_info(msg)
#else
  #define DEBUG_DISPLAY(msg)
#endif
```

## Custom Display Formats

### CSV Format

```c
// Display data in CSV format
void gy87_display_csv(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float mx, float my, float mz,
                      uint32_t period_ms)
{
  printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%lu\r\n",
         ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}
```

### JSON Format

```c
// Display data in JSON format
void gy87_display_json(float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float mx, float my, float mz,
                       uint32_t period_ms)
{
  printf("{\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"mag\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},\"period\":%lu}\r\n",
         ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}
```

### Binary Format

```c
// Display data in binary format
void gy87_display_binary(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float mx, float my, float mz,
                         uint32_t period_ms)
{
  // Send binary data packet
  struct {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    uint32_t period_ms;
  } packet = {ax, ay, az, gx, gy, gz, mx, my, mz, period_ms};
  
  HAL_UART_Transmit(&huart2, (uint8_t*)&packet, sizeof(packet), 100);
}
```

## Testing and Validation

### Display Function Testing

```c
// Test display functions
void gy87_test_display_functions(void)
{
  gy87_log_info("=== Testing Display Functions ===");
  
  // Test data
  float ax = 0.697f, ay = -0.536f, az = 9.578f;
  float gx = -0.039f, gy = -0.032f, gz = 0.011f;
  float mx = -0.0600f, my = 0.0093f, mz = 0.0606f;
  uint32_t period = 10;
  
  // Test basic display
  gy87_log_info("Testing gy87_display_agm:");
  gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, period);
  
  // Test formatted display
  gy87_log_info("Testing gy87_display_formatted_data:");
  gy87_display_formatted_data(ax, ay, az, gx, gy, gz, mx, my, mz, period);
  
  // Test individual displays
  gy87_log_info("Testing individual displays:");
  gy87_display_accelerometer(ax, ay, az, period);
  gy87_display_gyroscope(gx, gy, gz, period);
  gy87_display_magnetometer(mx, my, mz, period);
  
  gy87_log_info("=== Display Function Tests Complete ===");
}
```

### Performance Testing

```c
// Test display performance
void gy87_test_display_performance(void)
{
  gy87_log_info("=== Testing Display Performance ===");
  
  uint32_t start_time = HAL_GetTick();
  uint32_t test_count = 1000;
  
  for (uint32_t i = 0; i < test_count; i++)
  {
    gy87_display_agm(0.697f, -0.536f, 9.578f,
                     -0.039f, -0.032f, 0.011f,
                     -0.0600f, 0.0093f, 0.0606f, 10);
  }
  
  uint32_t end_time = HAL_GetTick();
  uint32_t total_time = end_time - start_time;
  uint32_t avg_time = total_time / test_count;
  
  gy87_log_info("Performance: %lu displays in %lu ms (avg: %lu ms per display)",
                test_count, total_time, avg_time);
  
  gy87_log_info("=== Display Performance Test Complete ===");
}
```

## License

Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.

---

**Last Updated**: 2025-09-19
**Version**: 2.0.2