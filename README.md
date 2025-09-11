# MPU6050 Sensor Reading System

A comprehensive system for reading MPU6050 sensor data using STM32F103C8T6 microcontroller and Python streaming scripts with real-time visualization.

## 📋 Overview

This project includes:

- **STM32 Firmware**: Reads MPU6050 data via I2C and transmits via UART
- **Python Scripts**: Real-time data display and analysis
- **Visualization**: Real-time graphs and console output
- **Mathematical Calculations**: Detailed sensor data processing and angle calculations

## 🛠️ Hardware

### STM32F103C8T6 (Blue Pill)

- **MCU**: ARM Cortex-M3, 72MHz
- **Flash**: 64KB
- **RAM**: 20KB
- **I2C**: PB6 (SCL), PB7 (SDA) - MPU6050 connection
- **UART**: PA2 (TX), PA3 (RX) - PC communication

### MPU6050

- **Accelerometer**: 3-axis, ±2g/±4g/±8g/±16g
- **Gyroscope**: 3-axis, ±250/±500/±1000/±2000°/s
- **Temperature**: -40°C to +85°C
- **Communication**: I2C (address 0x68)

### Connections

```
STM32F103C8T6    MPU6050
PB6 (SCL)    →   SCL
PB7 (SDA)    →   SDA
3.3V         →   VCC
GND          →   GND

STM32F103C8T6    USB-UART
PA2 (TX)     →   RX
PA3 (RX)     →   TX
GND          →   GND
3.3V         →   VCC (if needed)
```

## 🧮 Mathematical Calculations

### Raw Data Conversion

The MPU6050 provides raw 16-bit signed integer values that need to be converted to physical units.

#### Accelerometer Conversion

**Formula:**

```
Acceleration (m/s²) = (Raw_Value / 16384) × 9.81
```

**Where:**

- `Raw_Value`: 16-bit signed integer from sensor
- `16384`: LSB per g for ±2g range (2^15 / 2g = 16384)
- `9.81`: Conversion factor from g to m/s²

**Code Implementation:**

```c
float MPU6050_Get_Ax(void) {
    return (float)(((int16_t)(data_rx[0]<<8 | data_rx[1]))/(float)16384) * 9.81f;
}
```

#### Gyroscope Conversion

**Formula:**

```
Angular Velocity (rad/s) = (Raw_Value / 131) × (π / 180)
```

**Where:**

- `Raw_Value`: 16-bit signed integer from sensor
- `131`: LSB per °/s for ±250°/s range (2^15 / 250 = 131)
- `π/180`: Conversion factor from degrees to radians

**Code Implementation:**

```c
float MPU6050_Get_Gx(void) {
    return (float)(((int16_t)(data_rx[10]<<8 | data_rx[11]))/(float)131) * (M_PI / 180.0f);
}
```

#### Temperature Conversion

**Formula:**

```
Temperature (°C) = (Raw_Value / 340) + 36.53
```

**Where:**

- `Raw_Value`: 16-bit signed integer from sensor
- `340`: LSB per °C (from datasheet)
- `36.53`: Offset temperature (°C)

**Code Implementation:**

```c
float MPU6050_Get_Temperature(void) {
    return (float)(((int16_t)(data_rx[6]<<8 | data_rx[7]))/(float)340 + (float)36.53);
}
```

### Angle Calculations

#### Roll and Pitch from Accelerometer

**Formulas:**

```
Roll = atan2(Ay, √(Ax² + Az²)) × (180/π)
Pitch = atan2(-Ax, √(Ay² + Az²)) × (180/π)
```

**Where:**

- `Ax, Ay, Az`: Accelerometer values in m/s²
- `atan2`: Four-quadrant arctangent function
- `180/π`: Conversion from radians to degrees

**Code Implementation:**

```c
void MPU6050_CalculateAngles(float* roll, float* pitch, float* yaw) {
    float ax = MPU6050_Get_Ax();
    float ay = MPU6050_Get_Ay();
    float az = MPU6050_Get_Az();

    *roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / M_PI;
    *pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
    *yaw = 0.0f; // Cannot be calculated from accelerometer alone
}
```

#### Complementary Filter

**Formula:**

```
Angle = α × (Previous_Angle + Gyro_Rate × dt) + (1-α) × Acc_Angle
```

**Where:**

- `α`: Filter coefficient (0.98 for gyro, 0.02 for acc)
- `Previous_Angle`: Previous calculated angle
- `Gyro_Rate`: Gyroscope reading in rad/s
- `dt`: Time step in seconds
- `Acc_Angle`: Angle calculated from accelerometer

**Code Implementation:**

```c
void MPU6050_ComplementaryFilter(float* roll, float* pitch, float* yaw, float dt) {
    float ax = MPU6050_Get_Ax();
    float ay = MPU6050_Get_Ay();
    float az = MPU6050_Get_Az();
    float gx = MPU6050_Get_Gx(); // rad/s
    float gy = MPU6050_Get_Gy(); // rad/s
    float gz = MPU6050_Get_Gz(); // rad/s

    // Calculate angles from accelerometer
    float acc_roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / M_PI;
    float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;

    // Complementary filter
    float alpha = 0.98f;
    *roll = alpha * (prev_roll + gx * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_roll;
    *pitch = alpha * (prev_pitch + gy * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_pitch;
    *yaw = prev_yaw + gz * dt * 180.0f / M_PI; // Yaw from gyro only

    // Update previous values
    prev_roll = *roll;
    prev_pitch = *pitch;
    prev_yaw = *yaw;
}
```

### Sensor Specifications and Ranges

#### Accelerometer Ranges

| Range | LSB/g | Resolution | Max Value    |
| ----- | ----- | ---------- | ------------ |
| ±2g   | 16384 | 0.061 mg   | ±19.62 m/s²  |
| ±4g   | 8192  | 0.122 mg   | ±39.24 m/s²  |
| ±8g   | 4096  | 0.244 mg   | ±78.48 m/s²  |
| ±16g  | 2048  | 0.488 mg   | ±156.96 m/s² |

#### Gyroscope Ranges

| Range    | LSB/(°/s) | Resolution | Max Value    |
| -------- | --------- | ---------- | ------------ |
| ±250°/s  | 131       | 0.0076°/s  | ±4.36 rad/s  |
| ±500°/s  | 65.5      | 0.015°/s   | ±8.73 rad/s  |
| ±1000°/s | 32.8      | 0.031°/s   | ±17.45 rad/s |
| ±2000°/s | 16.4      | 0.061°/s   | ±34.91 rad/s |

### Error Analysis

#### Accelerometer Error Sources

1. **Noise**: ±0.1 mg RMS (typical)
2. **Offset**: ±60 mg (typical)
3. **Scale Factor**: ±2% (typical)
4. **Temperature Drift**: ±0.02%/°C

#### Gyroscope Error Sources

1. **Noise**: ±0.005°/s RMS (typical)
2. **Offset**: ±20°/s (typical)
3. **Scale Factor**: ±2% (typical)
4. **Temperature Drift**: ±0.02%/°C

#### Angle Calculation Errors

1. **Accelerometer Tilt**: ±1° (static conditions)
2. **Gyroscope Drift**: ±0.1°/s (without calibration)
3. **Complementary Filter**: Depends on α value and sampling rate

### Calibration Formulas

#### Accelerometer Calibration

```
Calibrated_Value = (Raw_Value - Offset) × Scale_Factor
```

#### Gyroscope Calibration

```
Calibrated_Value = (Raw_Value - Offset) × Scale_Factor
```

#### Offset Calculation (Static Calibration)

```
Offset = (Sum of 1000 samples) / 1000
```

#### Scale Factor Calculation

```
Scale_Factor = Known_Value / (Measured_Value - Offset)
```

## 🚀 Installation

### 1. Environment Setup

**Requirements:**

- STM32CubeIDE or Keil uVision
- Python 3.7+
- USB-UART converter (CP2102, CH340, etc.)
- Breadboard and jumper wires

**Install Python dependencies:**

```bash
pip install -r requirements.txt
```

### 2. Flash STM32 Firmware

1. Open project in STM32CubeIDE
2. Build project (Ctrl+B)
3. Flash to STM32F103C8T6
4. Connect UART to computer (baud rate: 115200)

### 3. Test Connection

Run simple script to test:

```bash
python mpu6050_simple.py
```

## 📊 Python Scripts

### ✅ Working Scripts

#### 1. `mpu6050_chooseOutput.py` - Configurable Output Script

**Features:**

- ✅ Interactive configuration menu
- ✅ Select data types to display
- ✅ Compact/detailed modes
- ✅ Clear screen, emoji mode
- ✅ Raw data display

**Usage:**

```bash
python mpu6050_chooseOutput.py
# Or specify COM port
python mpu6050_chooseOutput.py COM3
```

**Configuration Menu:**

```
==================================================
🔧 MPU6050 Output Configuration
==================================================
1. Accelerometer: ✅
2. Gyroscope:     ✅
3. Temperature:   ✅
4. Angles:        ✅
5. Raw Data:      ❌
6. Timestamp:     ✅
7. Separator:     ✅
8. Clear Screen:  ❌
9. Compact Mode:  ❌
10. Emoji Mode:   ✅
==================================================
Press number to toggle, 's' to start, 'q' to quit
```

#### 2. `mpu6050_simple.py` - Simple Console Script

**Features:**

- ✅ Basic data display
- ✅ Beautiful formatting with emojis
- ✅ Reading counter
- ✅ Easy debugging

**Usage:**

```bash
python mpu6050_simple.py
# Or specify COM port
python mpu6050_simple.py COM3
```

**Sample Output:**

```
============================================================
Timestamp: 14:30:25
============================================================
📊 Accelerometer (m/s²):
   X:    1.207  Y:   -4.475  Z:    7.744
🔄 Gyroscope (rad/s):
   X:    0.215  Y:   -0.991  Z:    1.573
🌡️  Temperature:  25.67 °C
📐 Euler Angles (deg):
   Roll:   15.23  Pitch:   -8.45  Yaw:  123.67
```

### ⚠️ Scripts to Check

#### 3. `mpu6050_streaming.py` - Real-time Graph Script

**Features:**

- 📊 Real-time 4-subplot graphs
- 📊 Accelerometer, Gyroscope, Temperature, Angles
- 📊 Multi-threading
- ⚠️ **May need debugging**

**Usage:**

```bash
python mpu6050_streaming.py
```

**Graph Interface:**

```
┌─────────────────┬─────────────────┐
│ Accelerometer   │ Gyroscope       │
│ (m/s²)          │ (rad/s)         │
├─────────────────┼─────────────────┤
│ Temperature     │ Euler Angles    │
│ (°C)            │ (deg)           │
└─────────────────┴─────────────────┘
```

## 🔧 Configuration

### Change COM Port

```python
# In Python file
streamer = MPU6050Streamer(port='COM3', baudrate=115200)
```

### Change Baud Rate

```python
# If STM32 uses different baud rate
streamer = MPU6050Streamer(port='COM19', baudrate=9600)
```

### Change Display Points

```python
# Increase/decrease points on graph
streamer = MPU6050Streamer(port='COM19', max_points=500)
```

## 📈 Data Display

### Accelerometer

- **Unit**: m/s² (meters per second squared)
- **Range**: ±19.62, ±39.24, ±78.48, ±156.96 m/s² (corresponding to ±2g, ±4g, ±8g, ±16g)
- **Resolution**: 160,727 LSB/(m/s²) (±2g range)

### Gyroscope

- **Unit**: rad/s (radians per second)
- **Range**: ±4.36, ±8.73, ±17.45, ±34.91 rad/s (corresponding to ±250, ±500, ±1000, ±2000 deg/s)
- **Resolution**: 2.29 LSB/(rad/s) (±250 deg/s range)

### Temperature

- **Unit**: °C
- **Range**: -40°C to +85°C
- **Resolution**: 340 LSB/°C

### Euler Angles

- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis
- **Unit**: degrees

## 🎯 Common Usage Modes

### 1. Debug Mode

- Use: `mpu6050_simple.py`
- Raw Data: ON
- Clear Screen: OFF
- Compact Mode: OFF

### 2. Monitoring Mode

- Use: `mpu6050_chooseOutput.py`
- Clear Screen: ON
- Compact Mode: ON
- Emoji Mode: OFF

### 3. Accelerometer Only Mode

- Accelerometer: ON
- Gyroscope: OFF
- Temperature: OFF
- Angles: OFF

### 4. Angle Only Mode

- Accelerometer: OFF
- Gyroscope: OFF
- Temperature: OFF
- Angles: ON

## 🔧 Troubleshooting

### "COM port not found" Error

1. Check Device Manager (Windows) or `ls /dev/tty*` (Linux)
2. Install USB-UART driver
3. Change COM port in script

### "No data received" Error

1. Check wire connections
2. Check baud rate (115200)
3. Check if STM32 is flashed with code
4. Try resetting STM32

### Graph not updating (streaming.py)

1. Check if data format is correct
2. Try running `mpu6050_simple.py` first for debugging
3. Check regex parsing in code
4. Check matplotlib installation

### Performance Issues

1. Reduce `max_points` in script
2. Increase `interval` in animation
3. Close other applications

## 📁 Project Structure

```
MPU6050_Read/
├── Core/                          # STM32 Firmware
│   ├── Inc/                       # Header files
│   │   ├── mpu6050.h             # MPU6050 driver
│   │   ├── i2c.h                 # I2C configuration
│   │   ├── usart.h               # UART configuration
│   │   └── main.h                # Main header
│   └── Src/                       # Source files
│       ├── main.c                # Main application
│       ├── mpu6050.c             # MPU6050 implementation
│       ├── i2c.c                 # I2C implementation
│       └── usart.c               # UART implementation
├── Drivers/                       # STM32 HAL drivers
├── Debug/                         # Build output
├── mpu6050_chooseOutput.py       # ✅ Working - Configurable output
├── mpu6050_simple.py             # ✅ Working - Simple console
├── mpu6050_streaming.py          # ⚠️ Check - Real-time graphs
├── requirements.txt              # Python dependencies
├── README.md                     # This file
├── README_ChooseOutput.md        # Detailed chooseOutput guide
└── README_Python_Streaming.md    # Detailed streaming guide
```

## 🎯 Advanced Usage

### Save Data to File

```python
# Add to script
import csv
with open('mpu6050_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ', 'Temp', 'Roll', 'Pitch', 'Yaw'])
    # ... write data
```

### Filter Data

```python
# Add low-pass filter
from scipy import signal
filtered_data = signal.savgol_filter(raw_data, window_length=5, polyorder=2)
```

### Export Graphs

```python
# Save graph
plt.savefig('mpu6050_plot.png', dpi=300, bbox_inches='tight')
```

## 📞 Support

If you encounter issues:

1. Check hardware connections
2. Run `mpu6050_simple.py` for debugging
3. Check data format from STM32
4. Try changing COM port and baud rate

## 📝 Notes

- **Shortcuts**: Press number to toggle, 's' to start, 'q' to quit
- **Stop streaming**: Ctrl+C
- **Configuration**: Can be changed anytime before starting
- **Save configuration**: Scripts don't save configuration, need to reconfigure each run

## 🔄 Updates

- **v1.0**: Basic MPU6050 reading
- **v1.1**: Added Python streaming scripts
- **v1.2**: Added configurable output options
- **v1.3**: Added real-time visualization (experimental)

---

**Author**: Nhan Vo  
**Created**: 2025  
**Version**: 1.0.0
