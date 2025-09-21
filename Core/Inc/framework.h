#ifndef INC_FRAMEWORK_H_
#define INC_FRAMEWORK_H_

#include <stdint.h>

// Data Frame Protocol Constants
#define FRAME_START_BYTE    0xAA
#define FRAME_END_BYTE      0x55
#define FRAME_DATA_SIZE     36      // 12 + 12 + 12 bytes
#define FRAME_TOTAL_SIZE    40      // 1 + 36 + 2 + 1 bytes

// Sensor Data Structure (36 bytes total)
typedef struct {
    // Accelerometer data (12 bytes) - m/s²
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Gyroscope data (12 bytes) - rad/s
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // Magnetometer data (12 bytes) - Tesla
    float mag_x;
    float mag_y;
    float mag_z;
} SensorData_t;

// Complete Data Frame Structure (40 bytes total)
typedef struct {
    uint8_t start_byte;         // 1 byte - 0xAA
    SensorData_t sensor_data;   // 36 bytes
    uint16_t checksum;          // 2 bytes
    uint8_t end_byte;           // 1 byte - 0x55
} DataFrame_t;

// Communication Framework Functions
uint16_t Framework_Calculate_Checksum(const SensorData_t* data);
uint8_t Framework_Create_DataFrame(DataFrame_t* frame, const SensorData_t* sensor_data);
uint8_t Framework_Validate_DataFrame(const DataFrame_t* frame);
void Framework_Send_DataFrame_UART(const DataFrame_t* frame);
void Framework_Send_DataFrame_Binary(const DataFrame_t* frame);
void Framework_Print_SensorData_UART(const SensorData_t* data);
void Framework_Print_DataFrame_Info(const DataFrame_t* frame);

// Data Conversion Utilities
void Framework_Float_To_Bytes(float value, uint8_t* bytes);
float Framework_Bytes_To_Float(const uint8_t* bytes);
void Framework_SensorData_To_Bytes(const SensorData_t* data, uint8_t* bytes);
void Framework_Bytes_To_SensorData(const uint8_t* bytes, SensorData_t* data);

// Frame Statistics and Monitoring
typedef struct {
    uint32_t frames_sent;
    uint32_t frames_received;
    uint32_t checksum_errors;
    uint32_t format_errors;
    uint32_t last_frame_time;
    float current_frame_rate;
} FrameStatistics_t;

extern FrameStatistics_t frame_stats;

void Framework_Reset_Statistics(void);
void Framework_Update_Statistics(uint8_t frame_valid);
void Framework_Print_Statistics(void);

// Global data storage for framework
extern SensorData_t current_sensor_data;
extern DataFrame_t current_data_frame;

#endif /* INC_FRAMEWORK_H_ */
