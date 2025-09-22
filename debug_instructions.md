**Version**: 2.0.2
**Author**: Nhan Vo
**Update**: 2025-09-19

# Debug Instructions for GY87 IMU Sensor System

## Overview

This document provides comprehensive debugging instructions for the GY87 IMU sensor system, including common issues, troubleshooting steps, and diagnostic tools.

## System Requirements

### Hardware Requirements

- **STM32F103C8T6** (Blue Pill) microcontroller
- **GY-87 10DOF IMU Module** with MPU6050, HMC5883L, and BMP180
- **USB-UART Converter** (CP2102, CH340, etc.)
- **Breadboard and jumper wires**
- **Multimeter** for voltage measurements
- **Oscilloscope** (optional, for advanced debugging)

### Software Requirements

- **STM32CubeIDE** or **Keil uVision**
- **Serial Terminal** (PuTTY, Tera Term, etc.)
- **ST-Link Utility** for flashing
- **Python 3.7+** (for testing scripts)

## Common Issues and Solutions

### 1. No Sensor Data Output

#### Symptoms
```
=== I2C Scanner Started ===
Scanning I2C bus for devices...
No I2C devices found!
=== I2C Scanner Completed ===
```

#### Possible Causes
- **Wiring Issues**: Incorrect I2C connections
- **Power Issues**: Insufficient power supply
- **Pull-up Resistors**: Missing or incorrect pull-up resistors
- **I2C Configuration**: Incorrect I2C settings

#### Debugging Steps

1. **Check Wiring Connections**
   ```
   STM32F103C8T6    GY-87 Module
   PB6 (SCL)    →   SCL
   PB7 (SDA)    →   SDA
   3.3V         →   VCC
   GND          →   GND
   ```

2. **Verify Power Supply**
   ```c
   // Check 3.3V supply voltage
   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
   {
     gy87_log_info("3.3V supply is present");
   }
   else
   {
     gy87_log_error("debug", "power_supply", HAL_ERROR);
   }
   ```

3. **Test I2C Pull-up Resistors**
   - Measure voltage on SCL and SDA lines
   - Should read 3.3V when idle
   - If 0V, pull-up resistors are missing

4. **Check I2C Configuration**
   ```c
   // Verify I2C configuration
   if (hi2c1.Instance == I2C1)
   {
     gy87_log_info("I2C1 is configured");
   }
   else
   {
     gy87_log_error("debug", "i2c_config", HAL_ERROR);
   }
   ```

### 2. Wrong I2C Addresses

#### Symptoms
```
=== I2C Scanner Started ===
Scanning I2C bus for devices...
Device found at address: 0x69 (105)   # Wrong address
Total devices found: 1
=== I2C Scanner Completed ===
```

#### Possible Causes
- **AD0 Pin**: Connected to VCC instead of GND
- **Address Conflict**: Multiple devices on same address
- **I2C Bus Issues**: Bus not properly configured

#### Debugging Steps

1. **Check AD0 Pin Connection**
   ```c
   // MPU6050 address check
   if (gy87_i2c_is_device_ready(0x68) == HAL_OK)
   {
     gy87_log_info("MPU6050 found at 0x68 (AD0=GND)");
   }
   else if (gy87_i2c_is_device_ready(0x69) == HAL_OK)
   {
     gy87_log_info("MPU6050 found at 0x69 (AD0=VCC)");
   }
   else
   {
     gy87_log_error("debug", "mpu6050_address", HAL_ERROR);
   }
   ```

2. **Update Address Definitions**
   ```c
   // In gy87_mpu6050.h
   #define MPU6050_ADDRESS 0x69  // Change from 0x68 to 0x69
   ```

3. **Verify All Sensor Addresses**
   ```c
   // Check all expected addresses
   uint8_t addresses[] = {0x1E, 0x68, 0x69, 0x77};
   char* names[] = {"HMC5883L", "MPU6050 (AD0=GND)", "MPU6050 (AD0=VCC)", "BMP180"};
   
   for (int i = 0; i < 4; i++)
   {
     if (gy87_i2c_is_device_ready(addresses[i]) == HAL_OK)
     {
       gy87_log_info("Found %s at 0x%02X", names[i], addresses[i]);
     }
   }
   ```

### 3. UART Output Not Visible

#### Symptoms
- No output in serial terminal
- Garbled characters
- Intermittent data

#### Possible Causes
- **Baud Rate Mismatch**: Incorrect baud rate setting
- **UART Configuration**: Wrong UART settings
- **Wiring Issues**: Incorrect UART connections
- **Buffer Overflow**: Data transmission too fast

#### Debugging Steps

1. **Check UART Configuration**
   ```c
   // Verify UART configuration
   if (huart2.Init.BaudRate == 921600)
   {
     gy87_log_info("UART2 baud rate: %lu", huart2.Init.BaudRate);
   }
   else
   {
     gy87_log_error("debug", "uart_baud_rate", HAL_ERROR);
   }
   ```

2. **Test UART Transmission**
   ```c
   // Simple UART test
   char test_msg[] = "UART Test Message\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);
   ```

3. **Check UART Connections**
   ```
   STM32F103C8T6    USB-UART
   PA2 (TX)     →   RX
   PA3 (RX)     →   TX
   GND          →   GND
   ```

4. **Verify Serial Terminal Settings**
   - **Baud Rate**: 921600
   - **Data Bits**: 8
   - **Stop Bits**: 1
   - **Parity**: None
   - **Flow Control**: None

### 4. Incorrect Sensor Values

#### Symptoms
- Sensor values are always zero
- Values are out of expected range
- Values are not changing

#### Possible Causes
- **Sensor Initialization**: Sensors not properly initialized
- **Data Conversion**: Incorrect conversion formulas
- **I2C Communication**: Data not being read correctly
- **Sensor Configuration**: Wrong sensor settings

#### Debugging Steps

1. **Check Sensor Initialization**
   ```c
   // Test individual sensor initialization
   if (gy87_mpu6050_init() == HAL_OK)
   {
     gy87_log_info("MPU6050 initialized successfully");
   }
   else
   {
     gy87_log_error("debug", "mpu6050_init", HAL_ERROR);
   }
   ```

2. **Verify Raw Data Reading**
   ```c
   // Read raw sensor data
   uint8_t raw_data[14];
   if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS, raw_data, 14, 100) == HAL_OK)
   {
     gy87_log_info("Raw data: %02X %02X %02X %02X %02X %02X", 
                   raw_data[0], raw_data[1], raw_data[2], 
                   raw_data[3], raw_data[4], raw_data[5]);
   }
   ```

3. **Test Data Conversion**
   ```c
   // Test conversion formulas
   int16_t raw_accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
   float accel_x = (float)raw_accel_x / 16384.0f * 9.81f;
   gy87_log_info("Raw: %d, Converted: %.3f m/s²", raw_accel_x, accel_x);
   ```

4. **Check Sensor Configuration**
   ```c
   // Verify sensor configuration registers
   uint8_t config_reg;
   if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, 0x1A, 1, &config_reg, 1, 100) == HAL_OK)
   {
     gy87_log_info("MPU6050 Config Register: 0x%02X", config_reg);
   }
   ```

### 5. Timing Issues

#### Symptoms
- Inconsistent timing intervals
- Missed timing cycles
- System hangs or freezes

#### Possible Causes
- **SysTick Configuration**: Incorrect SysTick settings
- **Interrupt Priority**: Wrong interrupt priority
- **Clock Configuration**: Incorrect system clock
- **Blocking Operations**: Long blocking operations

#### Debugging Steps

1. **Check SysTick Configuration**
   ```c
   // Verify SysTick configuration
   if (SysTick->LOAD == 8999) // 72MHz / 9000 = 8kHz
   {
     gy87_log_info("SysTick configured for 1ms ticks");
   }
   else
   {
     gy87_log_error("debug", "systick_config", HAL_ERROR);
   }
   ```

2. **Monitor Timing Variables**
   ```c
   // Debug timing variables
   static uint32_t debug_counter = 0;
   debug_counter++;
   
   if (debug_counter % 1000 == 0) // Every 1000 cycles
   {
     gy87_log_info("Timing debug: counter=%lu, tick=%lu", 
                   debug_counter, HAL_GetTick());
   }
   ```

3. **Check Interrupt Priority**
   ```c
   // Verify interrupt priority
   if (NVIC_GetPriority(SysTick_IRQn) == 15)
   {
     gy87_log_info("SysTick priority: %lu", NVIC_GetPriority(SysTick_IRQn));
   }
   ```

## Diagnostic Tools

### 1. I2C Scanner

```c
// Comprehensive I2C scanner
void gy87_debug_i2c_scanner(void)
{
  gy87_log_info("=== I2C Debug Scanner ===");
  
  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 3, 100) == HAL_OK)
    {
      gy87_log_info("Device found at 0x%02X (%d)", addr, addr);
      
      // Try to read device ID if possible
      if (addr == 0x68 || addr == 0x69) // MPU6050
      {
        uint8_t who_am_i;
        if (HAL_I2C_Mem_Read(&hi2c1, addr << 1, 0x75, 1, &who_am_i, 1, 100) == HAL_OK)
        {
          gy87_log_info("  MPU6050 WHO_AM_I: 0x%02X", who_am_i);
        }
      }
    }
  }
  
  gy87_log_info("=== I2C Debug Scanner Complete ===");
}
```

### 2. Sensor Status Checker

```c
// Check all sensor status
void gy87_debug_sensor_status(void)
{
  gy87_log_info("=== Sensor Status Check ===");
  
  // Check MPU6050
  if (gy87_i2c_is_device_ready(MPU6050_ADDRESS) == HAL_OK)
  {
    gy87_log_info("MPU6050: Ready");
    
    // Check WHO_AM_I register
    uint8_t who_am_i;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, 0x75, 1, &who_am_i, 1, 100) == HAL_OK)
    {
      gy87_log_info("  WHO_AM_I: 0x%02X (expected: 0x68)", who_am_i);
    }
  }
  else
  {
    gy87_log_error("debug", "mpu6050_status", HAL_ERROR);
  }
  
  // Check HMC5883L
  if (gy87_i2c_is_device_ready(HMC5883L_ADDRESS) == HAL_OK)
  {
    gy87_log_info("HMC5883L: Ready");
    
    // Check ID registers
    uint8_t id_a, id_b, id_c;
    if (HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDRESS, 0x0A, 1, &id_a, 1, 100) == HAL_OK &&
        HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDRESS, 0x0B, 1, &id_b, 1, 100) == HAL_OK &&
        HAL_I2C_Mem_Read(&hi2c1, HMC5883L_ADDRESS, 0x0C, 1, &id_c, 1, 100) == HAL_OK)
    {
      gy87_log_info("  ID: %c%c%c (expected: H43)", id_a, id_b, id_c);
    }
  }
  else
  {
    gy87_log_error("debug", "hmc5883l_status", HAL_ERROR);
  }
  
  gy87_log_info("=== Sensor Status Check Complete ===");
}
```

### 3. Data Validation

```c
// Validate sensor data ranges
void gy87_debug_validate_data(void)
{
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  
  if (gy87_read_all_sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz))
  {
    gy87_log_info("=== Data Validation ===");
    
    // Check accelerometer range
    if (ax >= -20.0f && ax <= 20.0f && ay >= -20.0f && ay <= 20.0f && az >= -20.0f && az <= 20.0f)
    {
      gy87_log_info("Accelerometer: OK (%.3f, %.3f, %.3f)", ax, ay, az);
    }
    else
    {
      gy87_log_error("debug", "accelerometer_range", HAL_ERROR);
    }
    
    // Check gyroscope range
    if (gx >= -5.0f && gx <= 5.0f && gy >= -5.0f && gy <= 5.0f && gz >= -5.0f && gz <= 5.0f)
    {
      gy87_log_info("Gyroscope: OK (%.3f, %.3f, %.3f)", gx, gy, gz);
    }
    else
    {
      gy87_log_error("debug", "gyroscope_range", HAL_ERROR);
    }
    
    // Check magnetometer range
    if (mx >= -0.2f && mx <= 0.2f && my >= -0.2f && my <= 0.2f && mz >= -0.2f && mz <= 0.2f)
    {
      gy87_log_info("Magnetometer: OK (%.6f, %.6f, %.6f)", mx, my, mz);
    }
    else
    {
      gy87_log_error("debug", "magnetometer_range", HAL_ERROR);
    }
    
    gy87_log_info("=== Data Validation Complete ===");
  }
  else
  {
    gy87_log_error("debug", "sensor_data_read", HAL_ERROR);
  }
}
```

## Debug Configuration

### Enable Debug Output

```c
// In main.c, add debug functions
int main(void)
{
  // System initialization
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  
  // Debug output
  gy87_log_info("=== GY87 Debug Mode ===");
  gy87_log_info("System initialized");
  
  // Run diagnostic tools
  gy87_debug_i2c_scanner();
  gy87_debug_sensor_status();
  
  // Initialize sensors
  gy87_init_all_sensors();
  
  // Main loop with debug
  while (1)
  {
    // Regular sensor reading
    gy87_display_all_sensors_agm(10);
    
    // Periodic debug validation
    static uint32_t debug_counter = 0;
    if (++debug_counter % 1000 == 0)
    {
      gy87_debug_validate_data();
    }
  }
}
```

### Debug Compilation Flags

```c
// In gy87_mpu6050.h
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
  #define DEBUG_LOG(msg) gy87_log_info(msg)
  #define DEBUG_ERROR(func, op, status) gy87_log_error(func, op, status)
#else
  #define DEBUG_LOG(msg)
  #define DEBUG_ERROR(func, op, status)
#endif
```

## Performance Monitoring

### Timing Analysis

```c
// Monitor timing performance
void gy87_debug_timing_analysis(void)
{
  static uint32_t last_time = 0;
  static uint32_t min_interval = 0xFFFFFFFF;
  static uint32_t max_interval = 0;
  static uint32_t total_intervals = 0;
  static uint32_t total_time = 0;
  
  uint32_t current_time = HAL_GetTick();
  
  if (last_time != 0)
  {
    uint32_t interval = current_time - last_time;
    
    if (interval < min_interval) min_interval = interval;
    if (interval > max_interval) max_interval = interval;
    
    total_intervals++;
    total_time += interval;
  }
  
  last_time = current_time;
  
  // Report every 1000 intervals
  if (total_intervals % 1000 == 0)
  {
    uint32_t avg_interval = total_time / total_intervals;
    gy87_log_info("Timing: min=%lu, max=%lu, avg=%lu ms", 
                  min_interval, max_interval, avg_interval);
  }
}
```

### Memory Usage Monitoring

```c
// Monitor memory usage
void gy87_debug_memory_usage(void)
{
  extern uint32_t _end;
  extern uint32_t _sdata;
  extern uint32_t _estack;
  
  uint32_t stack_used = (uint32_t)&_estack - (uint32_t)__get_MSP();
  uint32_t heap_used = (uint32_t)&_end - (uint32_t)&_sdata;
  
  gy87_log_info("Memory: Stack=%lu bytes, Heap=%lu bytes", stack_used, heap_used);
}
```

## License

Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.

---

**Last Updated**: 2025-09-19
**Version**: 2.0.2
