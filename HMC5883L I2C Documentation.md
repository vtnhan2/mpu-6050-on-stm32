<img src="https://r2cdn.perplexity.ai/pplx-full-logo-primary-dark%402x.png" style="height:64px;margin-right:32px"/>

# HMC5883L I2C Documentation

Đây là file .txt đầy đủ về I2C của HMC5883L:

```txt
================================================================================
                         HMC5883L I2C DOCUMENTATION
================================================================================

TỔNG QUAN:
- HMC5883L là 3-axis digital compass/magnetometer
- Giao tiếp: I2C interface
- Điện áp: 2.16V - 3.6V
- Dòng tiêu thụ: 100μA (measurement mode), 2.5μA (idle mode)
- Độ phân giải: 12-bit ADC
- Dải đo: ±0.88 Ga đến ±8.1 Ga

================================================================================
                              I2C ADDRESSING
================================================================================

ĐỊA CHỈ I2C:
- 7-bit address: 0x1E (30 decimal)
- 8-bit write address: 0x3C (60 decimal) 
- 8-bit read address: 0x3D (61 decimal)
- Địa chỉ CỐ ĐỊNH - không thể thay đổi

CÔNG THỨC CHUYỂN ĐỔI:
- 8-bit write = 7-bit << 1 + 0
- 8-bit read = 7-bit << 1 + 1
- Ví dụ: 0x1E << 1 = 0x3C (write), 0x3C | 1 = 0x3D (read)

================================================================================
                              REGISTER MAP
================================================================================

DANH SÁCH REGISTERS:
+----------+----------+-------------------+----------+------------------------+
| Address  | Name     | Description       | Access   | Default                |
+----------+----------+-------------------+----------+------------------------+
| 0x00     | CRA      | Config Register A | R/W      | 0x10                   |
| 0x01     | CRB      | Config Register B | R/W      | 0x20                   |
| 0x02     | MR       | Mode Register     | R/W      | 0x03                   |
| 0x03     | DXRA     | Data X MSB        | R        | Output                 |
| 0x04     | DXRB     | Data X LSB        | R        | Output                 |
| 0x05     | DZRA     | Data Z MSB        | R        | Output                 |
| 0x06     | DZRB     | Data Z LSB        | R        | Output                 |
| 0x07     | DYRA     | Data Y MSB        | R        | Output                 |
| 0x08     | DYRB     | Data Y LSB        | R        | Output                 |
| 0x09     | SR       | Status Register   | R        | 0x00                   |
| 0x0A     | IRA      | ID Register A     | R        | 0x48 ('H')             |
| 0x0B     | IRB      | ID Register B     | R        | 0x34 ('4')             |
| 0x0C     | IRC      | ID Register C     | R        | 0x33 ('3')             |
+----------+----------+-------------------+----------+------------------------+

LƯU Ý: Data register order là X, Z, Y (không phải X, Y, Z!)

================================================================================
                         CONFIGURATION REGISTER A (0x00)
================================================================================

CÁC BIT:
+-------+-------+-------+-------+-------+-------+-------+-------+
| Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
+-------+-------+-------+-------+-------+-------+-------+-------+
|   0   |  MA1  |  MA0  |  DO2  |  DO1  |  DO0  |  MS1  |  MS0  |
+-------+-------+-------+-------+-------+-------+-------+-------+

MA[1:0] - MEASUREMENT CONFIGURATION:
- 00: Normal measurement (default)
- 01: Positive bias configuration
- 10: Negative bias configuration
- 11: Reserved

DO[2:0] - DATA OUTPUT RATE:
- 000: 0.75 Hz
- 001: 1.5 Hz
- 010: 3 Hz
- 011: 7.5 Hz
- 100: 15 Hz (default)
- 101: 30 Hz
- 110: 75 Hz
- 111: Reserved

MS[1:0] - MEASUREMENT SAMPLES:
- 00: 1 sample (default)
- 01: 2 samples average
- 10: 4 samples average
- 11: 8 samples average

TYPICAL SETTINGS:
- Normal operation: 0x70 (8 samples, 15Hz, normal)
- High accuracy: 0x78 (8 samples, 75Hz, normal)
- Low power: 0x10 (1 sample, 15Hz, normal)

================================================================================
                         CONFIGURATION REGISTER B (0x01)
================================================================================

CÁC BIT:
+-------+-------+-------+-------+-------+-------+-------+-------+
| Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
+-------+-------+-------+-------+-------+-------+-------+-------+
|  GN2  |  GN1  |  GN0  |   0   |   0   |   0   |   0   |   0   |
+-------+-------+-------+-------+-------+-------+-------+-------+

GN[2:0] - GAIN CONFIGURATION:
+-------+--------+----------------+---------------+
| GN    | Range  | Scale (mG/LSB) | Register Value|
+-------+--------+----------------+---------------+
| 000   | ±0.88Ga| 0.73           | 0x00          |
| 001   | ±1.3Ga | 0.92           | 0x20 (default)|
| 010   | ±1.9Ga | 1.22           | 0x40          |
| 011   | ±2.5Ga | 1.52           | 0x60          |
| 100   | ±4.0Ga | 2.27           | 0x80          |
| 101   | ±4.7Ga | 2.56           | 0xA0          |
| 110   | ±5.6Ga | 3.03           | 0xC0          |
| 111   | ±8.1Ga | 4.35           | 0xE0          |
+-------+--------+----------------+---------------+

KHUYẾN NGHỊ:
- Môi trường bình thường: Gain 1 (±1.3Ga, 0x20)
- Gần động cơ/magnet: Gain 4 hoặc 7 (±4.0Ga hoặc ±8.1Ga)

================================================================================
                            MODE REGISTER (0x02)
================================================================================

CÁC BIT:
+-------+-------+-------+-------+-------+-------+-------+-------+
| Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
+-------+-------+-------+-------+-------+-------+-------+-------+
|   1   |   0   |   0   |   0   |   0   |   0   |  MD1  |  MD0  |
+-------+-------+-------+-------+-------+-------+-------+-------+

MD[1:0] - OPERATING MODE:
- 00: Continuous measurement mode
- 01: Single measurement mode
- 10: Idle mode (default)
- 11: Idle mode

MODE DESCRIPTIONS:
- Continuous (0x00): Tự động đo liên tục
- Single (0x01): Đo một lần rồi về idle
- Idle (0x02/0x03): Không đo, tiết kiệm năng lượng

KHUYẾN NGHỊ:
- Real-time applications: Continuous mode (0x00)
- Battery applications: Single mode (0x01)

================================================================================
                            STATUS REGISTER (0x09)
================================================================================

CÁC BIT:
+-------+-------+-------+-------+-------+-------+-------+-------+
| Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
+-------+-------+-------+-------+-------+-------+-------+-------+
|   0   |   0   |   0   |   0   |   0   |   0   |  LOCK | RDY   |
+-------+-------+-------+-------+-------+-------+-------+-------+

RDY (Bit 0) - READY BIT:
- 0: No new data
- 1: New data ready to read

LOCK (Bit 1) - DATA OUTPUT REGISTER LOCK:
- 0: Normal operation
- 1: Registers locked (during write operation)

CÁCH SỬ DỤNG:
```

// Check data ready before reading
byte status = readRegister(0x1E, 0x09);
if (status \& 0x01) {
// Data ready - safe to read
readMagnetometerData();
}

```

================================================================================
                             ID REGISTERS
================================================================================

ID REGISTER VALUES:
- Register A (0x0A): 0x48 (ASCII 'H')
- Register B (0x0B): 0x34 (ASCII '4') 
- Register C (0x0C): 0x33 (ASCII '3')
- Combined ID: "H43"

DEVICE IDENTIFICATION:
```

char deviceID = {0};
deviceID = readRegister(0x1E, 0x0A); // 'H'
deviceID = readRegister(0x1E, 0x0B); // '4'
deviceID = readRegister(0x1E, 0x0C); // '3'

if (strcmp(deviceID, "H43") == 0) {
// Confirmed HMC5883L
}

```

================================================================================
                            DATA REGISTERS
================================================================================

DATA REGISTER ORDER:
- 0x03: X-axis MSB
- 0x04: X-axis LSB  
- 0x05: Z-axis MSB
- 0x06: Z-axis LSB
- 0x07: Y-axis MSB
- 0x08: Y-axis LSB

LƯU Ý QUAN TRỌNG: 
Thứ tự là X, Z, Y (KHÔNG PHẢI X, Y, Z như thường nghĩ!)

CÁCH ĐỌC DỮ LIỆU:
```

// Method 1: Read individual registers
int16_t x = (readRegister(0x1E, 0x03) << 8) | readRegister(0x1E, 0x04);
int16_t z = (readRegister(0x1E, 0x05) << 8) | readRegister(0x1E, 0x06);
int16_t y = (readRegister(0x1E, 0x07) << 8) | readRegister(0x1E, 0x08);

// Method 2: Burst read (recommended)
Wire.beginTransmission(0x1E);
Wire.write(0x03);  // Start from X MSB
Wire.endTransmission();
Wire.requestFrom(0x1E, 6);  // Read 6 bytes

int16_t x = (Wire.read() << 8) | Wire.read();  // X
int16_t z = (Wire.read() << 8) | Wire.read();  // Z (not Y!)
int16_t y = (Wire.read() << 8) | Wire.read();  // Y

```

DATA RANGE:
- Raw values: -4096 to +4095 (12-bit signed)
- Overflow: -4096 indicates sensor overflow
- Zero field: Typically ±10 counts

================================================================================
                         I2C COMMUNICATION EXAMPLES
================================================================================

BASIC INITIALIZATION:
```

// 1. Wake up and configure
writeRegister(0x1E, 0x00, 0x70);  // Config A: 8-avg, 15Hz, normal
writeRegister(0x1E, 0x01, 0xA0);  // Config B: Gain 5 (±1.3Ga)
writeRegister(0x1E, 0x02, 0x00);  // Mode: Continuous measurement
delay(100);

```

SINGLE MEASUREMENT:
```

// 1. Set single measurement mode
writeRegister(0x1E, 0x02, 0x01);

// 2. Wait for conversion
delay(10);

// 3. Check status
byte status;
do {
status = readRegister(0x1E, 0x09);
} while (!(status \& 0x01));

// 4. Read data
readMagnetometerData();

```

CONTINUOUS MEASUREMENT:
```

// 1. Set continuous mode once
writeRegister(0x1E, 0x02, 0x00);

// 2. In main loop, just read data
while(1) {
if (readRegister(0x1E, 0x09) \& 0x01) {
readMagnetometerData();
}
delay(100);
}

```

================================================================================
                            CALIBRATION
================================================================================

HARD IRON CALIBRATION:
```

// Collect min/max values while rotating sensor 360°
int16_t mag_min = {32767, 32767, 32767};
int16_t mag_max = {-32768, -32768, -32768};

// During calibration
if (raw_x < mag_min) mag_min = raw_x;
if (raw_x > mag_max) mag_max = raw_x;
// Same for Y and Z

// Apply calibration
float calibrated_x = raw_x - (mag_min + mag_max) / 2;
float calibrated_y = raw_y - (mag_min + mag_max) / 2;

```

HEADING CALCULATION:
```

// Calculate heading from calibrated X and Y
float heading = atan2(calibrated_y, calibrated_x) * 180.0 / PI;

// Normalize to 0-360°
if (heading < 0) {
heading += 360;
}

// Apply magnetic declination (optional)
heading += MAGNETIC_DECLINATION;  // Your local declination
if (heading >= 360) heading -= 360;

```

================================================================================
                          TROUBLESHOOTING
================================================================================

COMMON PROBLEMS:

1. DEVICE NOT RESPONDING (0x1E):
   - Check I2C connections (SDA, SCL, VCC, GND)
   - Verify pull-up resistors (4.7kΩ recommended)
   - Check if behind MPU6050 (need I2C bypass)
   - Verify power supply (2.16V - 3.6V)

2. WRONG ID READING:
   - May be QMC5883L instead of HMC5883L
   - QMC5883L has address 0x0D and ID 0xFF
   - Check module marking/datasheet

3. ALL ZEROS OR OVERFLOW:
   - Check sensor orientation
   - Verify gain setting (may need higher gain)
   - Check for magnetic interference
   - Ensure sensor is not saturated

4. INCONSISTENT READINGS:
   - Need calibration (hard iron compensation)
   - Check for nearby magnetic sources
   - Verify measurement rate vs read rate
   - Check status register before reading

I2C BUS ISSUES:
- Use 400kHz max I2C speed
- Add 100ms delay after configuration
- Check for bus collisions with other devices
- Implement proper error handling

================================================================================
                         INTEGRATION WITH GY-87
================================================================================

GY-87 MODULE CONNECTION:
- HMC5883L connects through MPU6050's auxiliary I2C
- Must enable MPU6050 I2C bypass: Register 0x37 = 0x02
- Alternative: Use MPU6050 master mode (more complex)

ENABLE BYPASS CODE:
```

// Wake up MPU6050
writeRegister(0x68, 0x6B, 0x00);

// Enable I2C bypass
writeRegister(0x68, 0x37, 0x02);
delay(100);

// Now HMC5883L accessible at 0x1E

```

COMPLETE GY-87 SENSOR FUSION:
```

// Read all sensors
readMPU6050();     // Accel + Gyro from 0x68
readHMC5883L();    // Magnetometer from 0x1E (via bypass)
readBMP180();      // Pressure from 0x77

// Calculate orientation
float roll = atan2(ay, az);
float pitch = atan2(-ax, sqrt(ay*ay + az*az));
float yaw = atan2(mag_y, mag_x);  // From calibrated magnetometer

```

================================================================================
                              SPECIFICATIONS
================================================================================

ELECTRICAL:
- Supply Voltage: 2.16V to 3.6V
- Supply Current: 100μA (measuring), 2.5μA (standby)
- Logic Levels: VDD-referenced
- I2C Speed: Up to 400kHz

MAGNETIC:
- Resolution: 1-2 milli-gauss
- Range: ±0.88 to ±8.1 Gauss (selectable)
- Linearity: ±0.1% (±0.88 to ±8.1 Gauss)
- Cross-axis sensitivity: ±2%

MECHANICAL:
- Package: LGA-16 (3x3x0.9mm)
- Operating temp: -30°C to +85°C
- Shock: 10,000g
- Vibration: 20g (20Hz to 2000Hz)

================================================================================
                                SUMMARY
================================================================================

KEY POINTS:
✓ I2C Address: 0x1E (7-bit) / 0x3C-0x3D (8-bit)
✓ Device ID: "H43" at registers 0x0A-0x0C
✓ Data order: X, Z, Y (not X, Y, Z!)
✓ Check status register (0x09) before reading
✓ Use gain setting appropriate for environment
✓ Calibration essential for accurate heading
✓ On GY-87: Enable MPU6050 bypass first

RECOMMENDED SETTINGS:
- Config A (0x00): 0x70 (8-average, 15Hz, normal)
- Config B (0x01): 0xA0 (±1.3 Gauss range)
- Mode (0x02): 0x00 (continuous measurement)

================================================================================
                            END OF DOCUMENT
================================================================================

Revision: 1.0
Date: September 18, 2025
Author: I2C Documentation for HMC5883L Digital Compass
Compatible with: Arduino, STM32, ESP32, and other microcontrollers

For latest updates and code examples, refer to manufacturer datasheet
and community libraries.
```

File này chứa toàn bộ thông tin cần thiết về I2C của HMC5883L, từ địa chỉ, registers, cấu hình đến troubleshooting và integration với GY-87 module.

