# UART Display Functions Guide

## Overview

Các hàm UART display đã được di chuyển vào thư viện gy87_mpu6050 để tạo ra kiến trúc sạch hơn và dễ sử dụng.

## New Functions Added

### 1. GY87_Display_MPU6050_Individual()
**Purpose**: Hiển thị dữ liệu MPU6050 với format cũ (tương thích)

**Signature**:
```c
void GY87_Display_MPU6050_Individual(uint32_t period_ms);
```

**Output Format**:
```
MPU6050: Ax=0.160 Ay=-0.386 Az=9.743 m/s² | Temp=28.2°C | t=10ms
```

**Usage**:
```c
uint32_t uart_period = 10; // milliseconds
GY87_Display_MPU6050_Individual(uart_period);
```

### 2. GY87_Display_AGM() ⭐ NEW FORMAT
**Purpose**: Hiển thị dữ liệu AGM (Accelerometer, Gyroscope, Magnetometer) với format mới

**Signature**:
```c
void GY87_Display_AGM(float ax, float ay, float az, 
                      float gx, float gy, float gz, 
                      float mx, float my, float mz, 
                      uint32_t period_ms);
```

**Output Format**:
```
Axyz= 0.160 -0.386 9.743 m/s² | Gxyz= 0.001 -0.002 0.003 rad/s | Mxyz= 0.000001 0.000002 0.000003 Tesla | t=10ms
```

**Usage**:
```c
float ax = 1.0f, ay = 2.0f, az = 9.81f;
float gx = 0.1f, gy = 0.2f, gz = 0.3f;
float mx = 0.000001f, my = 0.000002f, mz = 0.000003f;
uint32_t period = 10;

GY87_Display_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, period);
```

### 3. GY87_Display_All_Sensors_AGM()
**Purpose**: Đọc tất cả sensor và hiển thị với format AGM

**Signature**:
```c
void GY87_Display_All_Sensors_AGM(uint32_t period_ms);
```

**Functionality**:
- Tự động đọc tất cả sensors
- Gọi `GY87_Display_AGM()` với dữ liệu đã đọc
- Xử lý lỗi đọc sensor

**Usage**:
```c
uint32_t uart_period = 10;
GY87_Display_All_Sensors_AGM(uart_period);
```

## Integration in main.c

### Current Implementation
```c
// Calculate UART transmission period
uint32_t uart_send_time = HAL_GetTick();
uart_period = uart_send_time - last_uart_time;
last_uart_time = uart_send_time;

// Option 1: Display only MPU6050 (current working version)
GY87_Display_MPU6050_Individual(uart_period);

// Option 2: Display all sensors in AGM format (uncomment when all sensors work)
// GY87_Display_All_Sensors_AGM(uart_period);
```

### Migration Path

#### From Old Code:
```c
// Old approach - manual implementation in main.c
float ax = GY87_MPU6050_Get_Ax();
float ay = GY87_MPU6050_Get_Ay();
float az = GY87_MPU6050_Get_Az();
float temp = GY87_MPU6050_Get_Temperature();

char buffer[128];
snprintf(buffer, sizeof(buffer), 
    "MPU6050: Ax=%.3f Ay=%.3f Az=%.3f m/s² | Temp=%.1f°C | t=%lums\r\n",
    ax, ay, az, temp, uart_period);
UART_SendString(buffer);
```

#### To New Code:
```c
// New approach - library function
GY87_Display_MPU6050_Individual(uart_period);
```

## Format Specifications

### AGM Format Details
```
Axyz= %.3f %.3f %.3f m/s² | Gxyz= %.3f %.3f %.3f rad/s | Mxyz= %.3f %.3f %.3f Tesla | t=%lums\r\n
```

**Field Descriptions**:
- **Axyz**: Accelerometer X, Y, Z axes in m/s²
- **Gxyz**: Gyroscope X, Y, Z axes in rad/s
- **Mxyz**: Magnetometer X, Y, Z axes in Tesla
- **t**: Transmission period in milliseconds

### Precision Settings
- **Accelerometer**: 3 decimal places (±0.001 m/s²)
- **Gyroscope**: 3 decimal places (±0.001 rad/s)
- **Magnetometer**: 6 decimal places (±0.000001 Tesla)
- **Time**: Integer milliseconds

## Error Handling

### Automatic Error Messages
```c
// If sensor reading fails:
"Error reading MPU6050 data!\r\n"
"Error reading sensor data!\r\n"
```

### Error Recovery
- Functions return gracefully on sensor errors
- No system crash or hang
- Clear error indication via UART

## Usage Examples

### Example 1: Debug MPU6050 Only
```c
void debug_mpu6050_loop(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_time >= 10) {
        uint32_t period = current_time - last_time;
        last_time = current_time;
        
        GY87_Display_MPU6050_Individual(period);
    }
}
```

### Example 2: Full AGM Display
```c
void full_sensor_display(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_time >= 10) {
        uint32_t period = current_time - last_time;
        last_time = current_time;
        
        GY87_Display_All_Sensors_AGM(period);
    }
}
```

### Example 3: Custom AGM Display
```c
void custom_agm_display(void) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    if (GY87_Read_All_Sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz)) {
        // Custom processing here...
        
        uint32_t period = 10; // or calculated value
        GY87_Display_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, period);
    }
}
```

## Benefits

### 1. **Clean Architecture**
- Display logic moved to sensor library
- Main.c focuses on application logic
- Reusable functions across projects

### 2. **Consistent Formatting**
- Standardized output formats
- Easy to parse by external systems
- Professional presentation

### 3. **Error Handling**
- Built-in error detection
- Graceful failure handling
- Clear error messages

### 4. **Flexibility**
- Multiple display options
- Easy to switch between formats
- Extensible for new requirements

### 5. **Maintainability**
- Centralized display logic
- Easy to modify formats
- Reduced code duplication

## Future Enhancements

### Planned Features
- JSON output format
- Binary data display
- Configurable precision
- Multiple UART interfaces
- Data logging capabilities

---

**Note**: Currently using `GY87_Display_MPU6050_Individual()` for debugging. Switch to `GY87_Display_All_Sensors_AGM()` when all sensors are working properly.
