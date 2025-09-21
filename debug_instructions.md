# Debug Instructions for Sensor Reading Error

## Current Error
The system is showing "Error reading sensor data!" repeatedly, which indicates that either `GY87_MPU6050_Read_Data()` or `GY87_HMC5883L_Read_Data()` is failing.

## Debug Steps Added

### 1. Enhanced Error Reporting
Added detailed error messages in sensor reading functions:
- MPU6050 TX/RX error codes
- HMC5883L TX/RX error codes

### 2. I2C Debug Function
Added `GY87_Debug_I2C_Status()` to check device readiness:
- MPU6050 (0x68)
- HMC5883L (0x1E) 
- BMP180 (0x77)

### 3. Simplified Testing
Modified main.c to test only MPU6050 first:
- Removed HMC5883L from main loop
- Added individual sensor value printing
- Added debug status calls

## What to Check

### 1. I2C Scanner Output
Look for this in UART output:
```
=== I2C Scanner Started ===
Scanning I2C bus for devices...
Device found at address: 0x68 (104)
Total devices found: 1
=== I2C Scanner Completed ===
```

### 2. Debug Status Output
Look for this:
```
=== I2C Debug Status ===
MPU6050 (0x68): READY
HMC5883L (0x1E): NOT READY
BMP180 (0x77): NOT READY
========================
```

### 3. Specific Error Messages
Look for these error messages:
- "MPU6050 TX Error: X"
- "MPU6050 RX Error: X"
- "HMC5883L TX Error: X"
- "HMC5883L RX Error: X"

## Common HAL Status Codes
- HAL_OK = 0
- HAL_ERROR = 1
- HAL_BUSY = 2
- HAL_TIMEOUT = 3

## Possible Causes

### 1. Hardware Issues
- Loose connections
- Wrong I2C pins (should be PB6=SCL, PB7=SDA)
- Power supply issues
- Pull-up resistors missing

### 2. I2C Configuration Issues
- Wrong I2C speed
- Incorrect addressing mode
- Clock stretching issues

### 3. Sensor Issues
- HMC5883L may not be present on your GY87 module
- MPU6050 may need different initialization sequence
- Timing issues between sensor reads

## Next Steps

1. **Check Hardware**: Verify connections and power
2. **Run Debug Version**: Flash the current debug version
3. **Analyze Output**: Look at I2C scanner and debug status
4. **Isolate Problem**: Determine if it's MPU6050, HMC5883L, or both
5. **Fix Addressing**: May need to adjust I2C addresses or initialization

## Quick Fix Options

### If only MPU6050 works:
Comment out HMC5883L calls in `GY87_Read_All_Sensors()`:
```c
// Read MPU6050 data
if (!GY87_MPU6050_Read_Data()) return 0;

// Skip HMC5883L for now
// if (!GY87_HMC5883L_Read_Data()) return 0;

// Set magnetometer values to zero
*mag_x = 0.0f;
*mag_y = 0.0f; 
*mag_z = 0.0f;
```

### If I2C addressing is wrong:
Check if your module uses different addresses:
- Some modules use 0x69 for MPU6050 (AD0=VCC)
- Some modules may not have HMC5883L

The debug version will help identify the exact issue.
