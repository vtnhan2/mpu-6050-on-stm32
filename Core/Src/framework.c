#include "framework.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Global data storage
SensorData_t current_sensor_data = {0};
DataFrame_t current_data_frame = {0};
FrameStatistics_t frame_stats = {0};

// Data Frame Processing Functions
uint16_t Framework_Calculate_Checksum(const SensorData_t* data)
{
    if (data == NULL) return 0;
    
    uint16_t checksum = 0;
    const uint8_t* bytes = (const uint8_t*)data;
    
    for (int i = 0; i < sizeof(SensorData_t); i++) {
        checksum += bytes[i];
    }
    
    return checksum;
}

uint8_t Framework_Create_DataFrame(DataFrame_t* frame, const SensorData_t* sensor_data)
{
    if (frame == NULL || sensor_data == NULL) return 0;
    
    // Set frame structure
    frame->start_byte = FRAME_START_BYTE;
    memcpy(&frame->sensor_data, sensor_data, sizeof(SensorData_t));
    frame->checksum = Framework_Calculate_Checksum(sensor_data);
    frame->end_byte = FRAME_END_BYTE;
    
    return 1;
}

uint8_t Framework_Validate_DataFrame(const DataFrame_t* frame)
{
    if (frame == NULL) return 0;
    
    // Check start and end bytes
    if (frame->start_byte != FRAME_START_BYTE) return 0;
    if (frame->end_byte != FRAME_END_BYTE) return 0;
    
    // Validate checksum
    uint16_t calculated_checksum = Framework_Calculate_Checksum(&frame->sensor_data);
    if (frame->checksum != calculated_checksum) return 0;
    
    return 1;
}

void Framework_Send_DataFrame_UART(const DataFrame_t* frame)
{
    if (frame == NULL) return;
    
    // Send the entire frame as binary data
    HAL_UART_Transmit(&huart2, (uint8_t*)frame, sizeof(DataFrame_t), 1000);
    
    // Update statistics
    frame_stats.frames_sent++;
    frame_stats.last_frame_time = HAL_GetTick();
}

void Framework_Send_DataFrame_Binary(const DataFrame_t* frame)
{
    if (frame == NULL) return;
    
    // Alternative binary transmission method
    // Can be customized for different communication protocols
    Framework_Send_DataFrame_UART(frame);
}

void Framework_Print_SensorData_UART(const SensorData_t* data)
{
    if (data == NULL) return;
    
    char buffer[256];
    
    snprintf(buffer, sizeof(buffer), 
        "Accel: X=%.3f Y=%.3f Z=%.3f m/s²\r\n"
        "Gyro:  X=%.3f Y=%.3f Z=%.3f rad/s\r\n"
        "Mag:   X=%.6f Y=%.6f Z=%.6f Tesla\r\n\r\n",
        data->accel_x, data->accel_y, data->accel_z,
        data->gyro_x, data->gyro_y, data->gyro_z,
        data->mag_x, data->mag_y, data->mag_z
    );
    
    UART_SendString(buffer);
}

void Framework_Print_DataFrame_Info(const DataFrame_t* frame)
{
    if (frame == NULL) return;
    
    char buffer[128];
    
    snprintf(buffer, sizeof(buffer),
        "DataFrame Info:\r\n"
        "  Start: 0x%02X\r\n"
        "  Checksum: 0x%04X\r\n"
        "  End: 0x%02X\r\n"
        "  Size: %d bytes\r\n"
        "  Valid: %s\r\n\r\n",
        frame->start_byte,
        frame->checksum,
        frame->end_byte,
        (int)sizeof(DataFrame_t),
        Framework_Validate_DataFrame(frame) ? "YES" : "NO"
    );
    
    UART_SendString(buffer);
}

// Data Conversion Utilities
void Framework_Float_To_Bytes(float value, uint8_t* bytes)
{
    if (bytes == NULL) return;
    
    union {
        float f;
        uint8_t b[4];
    } converter;
    
    converter.f = value;
    memcpy(bytes, converter.b, 4);
}

float Framework_Bytes_To_Float(const uint8_t* bytes)
{
    if (bytes == NULL) return 0.0f;
    
    union {
        float f;
        uint8_t b[4];
    } converter;
    
    memcpy(converter.b, bytes, 4);
    return converter.f;
}

void Framework_SensorData_To_Bytes(const SensorData_t* data, uint8_t* bytes)
{
    if (data == NULL || bytes == NULL) return;
    
    int offset = 0;
    
    // Accelerometer data (12 bytes)
    Framework_Float_To_Bytes(data->accel_x, &bytes[offset]); offset += 4;
    Framework_Float_To_Bytes(data->accel_y, &bytes[offset]); offset += 4;
    Framework_Float_To_Bytes(data->accel_z, &bytes[offset]); offset += 4;
    
    // Gyroscope data (12 bytes)
    Framework_Float_To_Bytes(data->gyro_x, &bytes[offset]); offset += 4;
    Framework_Float_To_Bytes(data->gyro_y, &bytes[offset]); offset += 4;
    Framework_Float_To_Bytes(data->gyro_z, &bytes[offset]); offset += 4;
    
    // Magnetometer data (12 bytes)
    Framework_Float_To_Bytes(data->mag_x, &bytes[offset]); offset += 4;
    Framework_Float_To_Bytes(data->mag_y, &bytes[offset]); offset += 4;
    Framework_Float_To_Bytes(data->mag_z, &bytes[offset]); offset += 4;
}

void Framework_Bytes_To_SensorData(const uint8_t* bytes, SensorData_t* data)
{
    if (bytes == NULL || data == NULL) return;
    
    int offset = 0;
    
    // Accelerometer data (12 bytes)
    data->accel_x = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    data->accel_y = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    data->accel_z = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    
    // Gyroscope data (12 bytes)
    data->gyro_x = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    data->gyro_y = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    data->gyro_z = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    
    // Magnetometer data (12 bytes)
    data->mag_x = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    data->mag_y = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
    data->mag_z = Framework_Bytes_To_Float(&bytes[offset]); offset += 4;
}

// Frame Statistics and Monitoring
void Framework_Reset_Statistics(void)
{
    memset(&frame_stats, 0, sizeof(FrameStatistics_t));
}

void Framework_Update_Statistics(uint8_t frame_valid)
{
    static uint32_t last_update_time = 0;
    static uint32_t frames_in_period = 0;
    
    uint32_t current_time = HAL_GetTick();
    
    if (frame_valid) {
        frame_stats.frames_received++;
    } else {
        frame_stats.checksum_errors++;
    }
    
    frames_in_period++;
    
    // Calculate frame rate every second
    if (current_time - last_update_time >= 1000) {
        frame_stats.current_frame_rate = (float)frames_in_period * 1000.0f / (current_time - last_update_time);
        frames_in_period = 0;
        last_update_time = current_time;
    }
}

void Framework_Print_Statistics(void)
{
    char buffer[256];
    
    snprintf(buffer, sizeof(buffer),
        "=== Frame Statistics ===\r\n"
        "Frames Sent: %lu\r\n"
        "Frames Received: %lu\r\n"
        "Checksum Errors: %lu\r\n"
        "Format Errors: %lu\r\n"
        "Current Frame Rate: %.1f Hz\r\n"
        "Last Frame Time: %lu ms\r\n"
        "========================\r\n\r\n",
        frame_stats.frames_sent,
        frame_stats.frames_received,
        frame_stats.checksum_errors,
        frame_stats.format_errors,
        frame_stats.current_frame_rate,
        frame_stats.last_frame_time
    );
    
    UART_SendString(buffer);
}

// High-level framework functions
uint8_t Framework_Process_And_Send(const SensorData_t* sensor_data)
{
    if (sensor_data == NULL) return 0;
    
    // Create frame
    if (!Framework_Create_DataFrame(&current_data_frame, sensor_data)) {
        return 0;
    }
    
    // Validate frame
    if (!Framework_Validate_DataFrame(&current_data_frame)) {
        frame_stats.format_errors++;
        return 0;
    }
    
    // Send frame
    Framework_Send_DataFrame_UART(&current_data_frame);
    
    // Update statistics
    Framework_Update_Statistics(1);
    
    return 1;
}
