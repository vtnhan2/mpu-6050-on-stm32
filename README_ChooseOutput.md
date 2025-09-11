# MPU6050 Choose Output - Hướng dẫn sử dụng

Script cho phép chọn loại dữ liệu hiển thị trên terminal từ MPU6050.

## 🚀 Cách sử dụng

### **Chạy script:**
```bash
python mpu6050_chooseOutput.py
# Hoặc chỉ định COM port
python mpu6050_chooseOutput.py COM19
```

### **Menu cấu hình:**
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

## ⚙️ Các tùy chọn

### **1. Accelerometer** - Hiển thị dữ liệu gia tốc
- **Bật**: Hiển thị X, Y, Z (m/s²)
- **Tắt**: Ẩn dữ liệu gia tốc

### **2. Gyroscope** - Hiển thị dữ liệu gyroscope
- **Bật**: Hiển thị X, Y, Z (rad/s)
- **Tắt**: Ẩn dữ liệu gyroscope

### **3. Temperature** - Hiển thị nhiệt độ
- **Bật**: Hiển thị nhiệt độ (°C)
- **Tắt**: Ẩn nhiệt độ

### **4. Angles** - Hiển thị góc quay
- **Bật**: Hiển thị Roll, Pitch, Yaw (deg)
- **Tắt**: Ẩn góc quay

### **5. Raw Data** - Hiển thị dữ liệu thô
- **Bật**: Hiển thị dòng dữ liệu gốc từ STM32
- **Tắt**: Chỉ hiển thị dữ liệu đã parse

### **6. Timestamp** - Hiển thị thời gian
- **Bật**: Hiển thị thời gian hiện tại
- **Tắt**: Ẩn thời gian

### **7. Separator** - Hiển thị dấu phân cách
- **Bật**: Hiển thị dấu "=" giữa các lần đọc
- **Tắt**: Không hiển thị dấu phân cách

### **8. Clear Screen** - Xóa màn hình
- **Bật**: Xóa màn hình mỗi lần cập nhật
- **Tắt**: Không xóa màn hình (cuộn xuống)

### **9. Compact Mode** - Chế độ compact
- **Bật**: Hiển thị tất cả trên 1 dòng
- **Tắt**: Hiển thị chi tiết từng loại

### **10. Emoji Mode** - Chế độ emoji
- **Bật**: Hiển thị emoji (📊, 🔄, 🌡️, 📐)
- **Tắt**: Không hiển thị emoji

## 📊 Ví dụ output

### **Chế độ chi tiết (Compact Mode = OFF):**
```
============================================================
⏰ 14:30:25
============================================================
📊 Accelerometer (m/s²):
   X:    1.207  Y:   -4.475  Z:    7.744
🔄 Gyroscope (rad/s):
   X:    0.215  Y:   -0.991  Z:    1.573
🌡️  Temperature:  25.67 °C
📐 Euler Angles (deg):
   Roll:   15.23  Pitch:   -8.45  Yaw:  123.67
```

### **Chế độ compact (Compact Mode = ON):**
```
⏰ 14:30:25 | A:  1.21, -4.48,  7.74 | G: 0.215, -0.991,  1.573 | T: 25.7°C | R:  15.2, P:  -8.4, Y: 123.7
```

### **Chế độ raw data (Raw Data = ON):**
```
⏰ 14:30:25
📡 Raw: Accelerometer: X=1.207, Y=-4.475, Z=7.744 m/s²
📡 Raw: Gyroscope: X=0.215, Y=-0.991, Z=1.573 rad/s
📡 Raw: Temperature: 25.67 C
📡 Raw: Angles: Roll=15.23, Pitch=-8.45, Yaw=123.67 deg
```

## 🎯 Các chế độ sử dụng phổ biến

### **1. Chế độ debug:**
- Raw Data: ON
- Clear Screen: OFF
- Compact Mode: OFF

### **2. Chế độ monitoring:**
- Clear Screen: ON
- Compact Mode: ON
- Emoji Mode: OFF

### **3. Chế độ chỉ gia tốc:**
- Accelerometer: ON
- Gyroscope: OFF
- Temperature: OFF
- Angles: OFF

### **4. Chế độ chỉ góc quay:**
- Accelerometer: OFF
- Gyroscope: OFF
- Temperature: OFF
- Angles: ON

## 🔧 Troubleshooting

### **Lỗi "COM port not found":**
1. Kiểm tra COM port trong Device Manager
2. Thử COM port khác
3. Kiểm tra kết nối dây

### **Không có dữ liệu:**
1. Kiểm tra STM32 đã flash code chưa
2. Kiểm tra baud rate (115200)
3. Thử reset STM32

### **Menu không hiển thị:**
1. Đảm bảo terminal hỗ trợ Unicode
2. Thử chạy trong Command Prompt thay vì PowerShell

## 📝 Ghi chú

- **Phím tắt**: Nhấn số để toggle, 's' để start, 'q' để quit
- **Dừng streaming**: Ctrl+C
- **Cấu hình**: Có thể thay đổi bất cứ lúc nào trước khi start
- **Lưu cấu hình**: Script không lưu cấu hình, cần cấu hình lại mỗi lần chạy
