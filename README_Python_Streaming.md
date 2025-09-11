# MPU6050 Python Streaming Scripts

Hai script Python để streaming dữ liệu từ MPU6050 qua UART và hiển thị đồ thị real-time.

## 📋 Yêu cầu

- Python 3.7+
- STM32F103C8T6 với MPU6050 đã được flash code
- Kết nối UART (PA2-TX, PA3-RX) với máy tính
- COM port (mặc định COM19)

## 🚀 Cài đặt

1. Cài đặt các thư viện cần thiết:
```bash
pip install -r requirements.txt
```

2. Kết nối phần cứng:
   - STM32 PA2 (TX) → USB-UART RX
   - STM32 PA3 (RX) → USB-UART TX
   - STM32 GND → USB-UART GND
   - STM32 3.3V → USB-UART VCC

3. Flash code STM32 và kết nối UART với baud rate 115200

## 📊 Scripts có sẵn

### 1. `mpu6050_streaming.py` - Script đầy đủ với đồ thị

**Tính năng:**
- ✅ Đồ thị real-time 4 subplot
- ✅ Accelerometer (X, Y, Z)
- ✅ Gyroscope (X, Y, Z) 
- ✅ Temperature
- ✅ Euler Angles (Roll, Pitch, Yaw)
- ✅ Auto-scaling và grid
- ✅ Multi-threading

**Cách chạy:**
```bash
python mpu6050_streaming.py
```

**Giao diện:**
```
┌─────────────────┬─────────────────┐
│ Accelerometer   │ Gyroscope       │
│ (g)             │ (deg/s)         │
├─────────────────┼─────────────────┤
│ Temperature     │ Euler Angles    │
│ (°C)            │ (deg)           │
└─────────────────┴─────────────────┘
```

### 2. `mpu6050_simple.py` - Script console đơn giản

**Tính năng:**
- ✅ Hiển thị dữ liệu trong console
- ✅ Format đẹp với emoji
- ✅ Đếm số lần đọc
- ✅ Dễ debug

**Cách chạy:**
```bash
python mpu6050_simple.py
# Hoặc chỉ định COM port
python mpu6050_simple.py COM3
```

**Output mẫu:**
```
============================================================
Timestamp: 14:30:25
============================================================
📊 Accelerometer (g):
   X:    0.123  Y:   -0.456  Z:    0.789
🔄 Gyroscope (deg/s):
   X:   12.340  Y:  -56.780  Z:   90.120
🌡️  Temperature:  25.67 °C
📐 Euler Angles (deg):
   Roll:   15.23  Pitch:   -8.45  Yaw:  123.67
```

## ⚙️ Cấu hình

### Thay đổi COM port:
```python
# Trong file Python
streamer = MPU6050Streamer(port='COM3', baudrate=115200)
```

### Thay đổi baud rate:
```python
# Nếu STM32 sử dụng baud rate khác
streamer = MPU6050Streamer(port='COM19', baudrate=9600)
```

### Thay đổi số điểm hiển thị:
```python
# Tăng/giảm số điểm trên đồ thị
streamer = MPU6050Streamer(port='COM19', max_points=500)
```

## 🔧 Troubleshooting

### Lỗi "COM port not found":
1. Kiểm tra Device Manager (Windows) hoặc `ls /dev/tty*` (Linux)
2. Cài đặt driver USB-UART
3. Thay đổi COM port trong script

### Lỗi "No data received":
1. Kiểm tra kết nối dây
2. Kiểm tra baud rate (115200)
3. Kiểm tra STM32 đã được flash code chưa
4. Thử reset STM32

### Đồ thị không cập nhật:
1. Kiểm tra dữ liệu có đúng format không
2. Thử chạy `mpu6050_simple.py` trước để debug
3. Kiểm tra regex parsing trong code

### Performance issues:
1. Giảm `max_points` trong script
2. Tăng `interval` trong animation
3. Đóng các ứng dụng khác

## 📈 Dữ liệu hiển thị

### Accelerometer:
- **Đơn vị**: g (gravity)
- **Range**: ±2g, ±4g, ±8g, ±16g (tùy cấu hình MPU6050)
- **Độ phân giải**: 16384 LSB/g (±2g)

### Gyroscope:
- **Đơn vị**: deg/s (degrees per second)
- **Range**: ±250, ±500, ±1000, ±2000 deg/s
- **Độ phân giải**: 131 LSB/deg/s (±250 deg/s)

### Temperature:
- **Đơn vị**: °C
- **Range**: -40°C đến +85°C
- **Độ phân giải**: 340 LSB/°C

### Euler Angles:
- **Roll**: Góc quay quanh trục X
- **Pitch**: Góc quay quanh trục Y  
- **Yaw**: Góc quay quanh trục Z
- **Đơn vị**: degrees

## 🎯 Sử dụng nâng cao

### Lưu dữ liệu vào file:
```python
# Thêm vào script
import csv
with open('mpu6050_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ', 'Temp', 'Roll', 'Pitch', 'Yaw'])
    # ... ghi dữ liệu
```

### Filter dữ liệu:
```python
# Thêm low-pass filter
from scipy import signal
filtered_data = signal.savgol_filter(raw_data, window_length=5, polyorder=2)
```

### Export đồ thị:
```python
# Lưu đồ thị
plt.savefig('mpu6050_plot.png', dpi=300, bbox_inches='tight')
```

## 📞 Hỗ trợ

Nếu gặp vấn đề:
1. Kiểm tra kết nối phần cứng
2. Chạy `mpu6050_simple.py` để debug
3. Kiểm tra format dữ liệu từ STM32
4. Thử thay đổi COM port và baud rate
