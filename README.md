# ROS2 IMU Hardware Interface

This package provides a ROS2 interface for the STM32-based IMU system with MPU6050 accelerometer/gyroscope and HMC5883L magnetometer.

## Features

- **Hardware Interface**: Communicates with STM32 via UART using custom binary protocol
- **ROS2 Integration**: Publishes `sensor_msgs/Imu` messages
- **Frame Protocol**: Supports `AA_data__checksum__FF` format (40 bytes total)
- **Configurable**: Serial port, baud rate, frame ID, and publish rate parameters

## Frame Protocol

The STM32 sends data in the following format:
```
START(1) + DATA(36) + CHECKSUM(2) + END(1) = 40 bytes
```

- **START**: `0xAA`
- **DATA**: 36 bytes containing 9 floats (little-endian):
  - Accelerometer: `accel_x, accel_y, accel_z` (m/s²)
  - Gyroscope: `gyro_x, gyro_y, gyro_z` (rad/s)  
  - Magnetometer: `mag_x, mag_y, mag_z` (µT)
- **CHECKSUM**: 16-bit little-endian sum over DATA only
- **END**: `0xFF`

## Installation

1. Build the package:
```bash
cd your_ros2_workspace
colcon build --packages-select imu
```

2. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Basic Launch
```bash
ros2 launch imu imu.launch.py
```

### With Custom Parameters
```bash
ros2 launch imu imu.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  frame_id:=imu_link \
  publish_rate:=100.0
```

### Manual Node Execution
```bash
ros2 run imu imu_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p frame_id:=imu_link \
  -p publish_rate:=100.0
```

## Topics

- **`/imu/data_raw`** (`sensor_msgs/Imu`): Raw IMU data
  - `linear_acceleration`: Accelerometer data (m/s²)
  - `angular_velocity`: Gyroscope data (rad/s)
  - `orientation`: Quaternion (placeholder, set to identity)

## Parameters

- **`serial_port`** (string, default: `/dev/ttyUSB0`): Serial port device
- **`baud_rate`** (int, default: `115200`): Serial communication baud rate
- **`frame_id`** (string, default: `imu_link`): Frame ID for IMU data
- **`publish_rate`** (double, default: `100.0`): IMU data publish rate in Hz

## Testing

1. **Check if node is running**:
```bash
ros2 node list
```

2. **Monitor IMU data**:
```bash
ros2 topic echo /imu/data_raw
```

3. **Check topic info**:
```bash
ros2 topic info /imu/data_raw
```

4. **View topic rate**:
```bash
ros2 topic hz /imu/data_raw
```

## Hardware Requirements

- STM32F103C8T6 with MPU6050 + HMC5883L (GY87 module)
- UART connection to ROS2 system
- Proper wiring and power supply

## Troubleshooting

### No Data Received
- Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches STM32 configuration
- Check wiring connections

### Invalid Frame Data
- Monitor raw serial data: `cat /dev/ttyUSB0 | hexdump -C`
- Verify frame format matches `AA_data__checksum__FF`
- Check for data corruption or timing issues

### ROS2 Issues
- Ensure workspace is sourced: `source install/setup.bash`
- Check node logs: `ros2 run imu imu_node`
- Verify topic publishing: `ros2 topic list`

## Development Notes

- The current implementation includes placeholder serial communication
- Real serial implementation would use libraries like `boost::asio` or `serial`
- Orientation calculation from sensor fusion is not implemented
- Covariance matrices are set to unknown (-1.0)

## License

MIT License
