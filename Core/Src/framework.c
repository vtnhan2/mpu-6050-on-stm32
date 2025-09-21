/**
 * @file       framework.c
 * @copyright  Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2025-09-19
 * @author     Nhan Vo
 *             
 * @brief      Data transmission framework implementation for GY87 IMU sensor
 *             Provides structured data protocol for UART communication
 *             
 * @note       This framework handles data packaging and transmission
 * @example    main.c
 *             Framework usage in main application
 * @see        https://github.com/vtnhan2/mpu-6050-on-stm32
 */

/* Includes ----------------------------------------------------------- */
#include "framework.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
#define FRAMEWORK_MAX_RETRIES    (3)     /*!< Maximum transmission retries */
#define FRAMEWORK_RETRY_DELAY_MS (10)    /*!< Retry delay in milliseconds */

/* Private enumerate/structure ---------------------------------------- */
/**
 * @brief Framework internal state enumeration
 */
typedef enum 
{
  FRAMEWORK_STATE_IDLE,      /**< Framework idle state */
  FRAMEWORK_STATE_TX,        /**< Framework transmission state */
  FRAMEWORK_STATE_RX,        /**< Framework reception state */
  FRAMEWORK_STATE_ERROR      /**< Framework error state */
}
framework_state_t;

/* Private macros ----------------------------------------------------- */
/**
 * @brief  Check if frame buffer is valid
 *
 * @param[in]     buffer      Frame buffer to check
 * @param[in]     size        Buffer size
 *
 * @return  
 *  - 0: Invalid buffer
 *  - 1: Valid buffer
 */
#define FRAMEWORK_VALIDATE_BUFFER(buffer, size)  \
  ((buffer) != NULL && (size) >= FRAME_TOTAL_SIZE)

/**
 * @brief  Calculate simple checksum for data integrity
 *
 * @param[in]     data        Data buffer
 * @param[in]     length      Data length
 *
 * @return  Calculated checksum
 */
#define FRAMEWORK_SIMPLE_CHECKSUM(data, length)  \
  FRAME_CALCULATE_CHECKSUM(data, length)

/* Private variables -------------------------------------------------- */
static framework_state_t s_framework_state = FRAMEWORK_STATE_IDLE;
static uint32_t s_frames_sent = 0;
static uint32_t s_frames_received = 0;
static uint32_t s_errors_count = 0;
static uint8_t s_frame_buffer[FRAME_TOTAL_SIZE];

/* Private function prototypes ---------------------------------------- */
/**
 * @brief  Send frame buffer via UART
 *
 * @param[in]     buffer      Frame buffer to send
 * @param[in]     size        Buffer size
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
static uint8_t framework_send_frame(const uint8_t *buffer, size_t size);

/**
 * @brief  Receive frame buffer via UART
 *
 * @param[out]    buffer      Frame buffer to receive
 * @param[in]     size        Buffer size
 * @param[in]     timeout_ms  Reception timeout
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 *  - 2: Timeout
 */
static uint8_t framework_receive_frame(uint8_t *buffer, size_t size, uint32_t timeout_ms);

// Removed unused function framework_find_start_byte

/* Function definitions ----------------------------------------------- */
void framework_init(void)
{
  s_framework_state = FRAMEWORK_STATE_IDLE;
  s_frames_sent = 0;
  s_frames_received = 0;
  s_errors_count = 0;
  
  // Clear frame buffer
  memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
}

uint8_t framework_pack_sensor_data(const sensor_frame_t *sensor_data, 
                                   uint8_t *frame_buffer, 
                                   size_t buffer_size)
{
  if (!FRAMEWORK_VALIDATE_BUFFER(frame_buffer, buffer_size) || sensor_data == NULL)
  {
    return 1;
}

  // Clear frame buffer
  memset(frame_buffer, 0, buffer_size);
  
  // Frame structure: [START][DATA][CHECKSUM][END]
  uint8_t offset = 0;
  
  // Start byte
  frame_buffer[offset++] = FRAME_START_BYTE;
  
  // Pack accelerometer data
  PACK_FLOAT(sensor_data->accel_x, frame_buffer, offset);
  offset += 4;
  PACK_FLOAT(sensor_data->accel_y, frame_buffer, offset);
  offset += 4;
  PACK_FLOAT(sensor_data->accel_z, frame_buffer, offset);
  offset += 4;
  
  // Pack gyroscope data
  PACK_FLOAT(sensor_data->gyro_x, frame_buffer, offset);
  offset += 4;
  PACK_FLOAT(sensor_data->gyro_y, frame_buffer, offset);
  offset += 4;
  PACK_FLOAT(sensor_data->gyro_z, frame_buffer, offset);
  offset += 4;
  
  // Pack magnetometer data
  PACK_FLOAT(sensor_data->mag_x, frame_buffer, offset);
  offset += 4;
  PACK_FLOAT(sensor_data->mag_y, frame_buffer, offset);
  offset += 4;
  PACK_FLOAT(sensor_data->mag_z, frame_buffer, offset);
  offset += 4;
  
  // Calculate and add checksum
  uint8_t checksum = FRAMEWORK_SIMPLE_CHECKSUM(frame_buffer + 1, FRAME_DATA_SIZE);
  frame_buffer[offset++] = checksum;
  
  // End byte
  frame_buffer[offset++] = FRAME_END_BYTE;
  
  return 0;
}

uint8_t framework_unpack_sensor_data(const uint8_t *frame_buffer, 
                                     size_t buffer_size, 
                                     sensor_frame_t *sensor_data)
{
  if (!FRAMEWORK_VALIDATE_BUFFER(frame_buffer, buffer_size) || sensor_data == NULL)
  {
    return 1;
}

  // Validate frame structure
  if (framework_validate_frame(frame_buffer, buffer_size))
  {
    return 1;
}

  uint8_t offset = 1;  // Skip start byte
  
  // Unpack accelerometer data
  sensor_data->accel_x = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  sensor_data->accel_y = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  sensor_data->accel_z = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  
  // Unpack gyroscope data
  sensor_data->gyro_x = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  sensor_data->gyro_y = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  sensor_data->gyro_z = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  
  // Unpack magnetometer data
  sensor_data->mag_x = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  sensor_data->mag_y = UNPACK_FLOAT(frame_buffer, offset);
  offset += 4;
  sensor_data->mag_z = UNPACK_FLOAT(frame_buffer, offset);
  
  return 0;
}

uint8_t framework_transmit_sensor_data(const sensor_frame_t *sensor_data)
{
  if (sensor_data == NULL)
  {
    s_errors_count++;
    return 1;
  }
  
  s_framework_state = FRAMEWORK_STATE_TX;
  
  // Pack sensor data into frame
  if (framework_pack_sensor_data(sensor_data, s_frame_buffer, sizeof(s_frame_buffer)))
  {
    s_framework_state = FRAMEWORK_STATE_ERROR;
    s_errors_count++;
    return 1;
  }
  
  // Send frame via UART
  if (framework_send_frame(s_frame_buffer, FRAME_TOTAL_SIZE))
  {
    s_framework_state = FRAMEWORK_STATE_ERROR;
    s_errors_count++;
    return 1;
  }
  
  s_frames_sent++;
  s_framework_state = FRAMEWORK_STATE_IDLE;
  
  return 0;
}

uint8_t framework_receive_sensor_data(sensor_frame_t *sensor_data, uint32_t timeout_ms)
{
  if (sensor_data == NULL)
  {
    s_errors_count++;
    return 1;
  }
  
  s_framework_state = FRAMEWORK_STATE_RX;
  
  // Receive frame via UART
  if (framework_receive_frame(s_frame_buffer, FRAME_TOTAL_SIZE, timeout_ms))
  {
    s_framework_state = FRAMEWORK_STATE_ERROR;
    s_errors_count++;
    return 1;
  }
  
  // Unpack sensor data from frame
  if (framework_unpack_sensor_data(s_frame_buffer, FRAME_TOTAL_SIZE, sensor_data))
  {
    s_framework_state = FRAMEWORK_STATE_ERROR;
    s_errors_count++;
    return 1;
  }
  
  s_frames_received++;
  s_framework_state = FRAMEWORK_STATE_IDLE;
  
  return 0;
}

uint8_t framework_validate_frame(const uint8_t *frame_buffer, size_t buffer_size)
{
  if (!FRAMEWORK_VALIDATE_BUFFER(frame_buffer, buffer_size))
  {
    return 1;
  }
  
  // Check start byte
  if (frame_buffer[0] != FRAME_START_BYTE)
  {
    return 1;
  }
  
  // Check end byte
  if (frame_buffer[FRAME_TOTAL_SIZE - 1] != FRAME_END_BYTE)
  {
    return 1;
  }
  
  // Check checksum
  uint8_t calculated_checksum = FRAMEWORK_SIMPLE_CHECKSUM(frame_buffer + 1, FRAME_DATA_SIZE);
  uint8_t received_checksum = frame_buffer[FRAME_TOTAL_SIZE - 2];
  
  if (calculated_checksum != received_checksum)
  {
    return 1;
  }
  
  return 0;
}

void framework_get_statistics(uint32_t *frames_sent, 
                              uint32_t *frames_received, 
                              uint32_t *errors_count)
{
  if (frames_sent != NULL)
  {
    *frames_sent = s_frames_sent;
  }
  
  if (frames_received != NULL)
  {
    *frames_received = s_frames_received;
  }
  
  if (errors_count != NULL)
  {
    *errors_count = s_errors_count;
  }
}

void framework_reset_statistics(void)
{
  s_frames_sent = 0;
  s_frames_received = 0;
  s_errors_count = 0;
}

/* Private definitions ----------------------------------------------- */
static uint8_t framework_send_frame(const uint8_t *buffer, size_t size)
{
  if (buffer == NULL || size == 0)
  {
    return 1;
  }
  
  // Send frame via UART
  for (size_t i = 0; i < size; i++)
  {
    UART_SendChar(buffer[i]);
  }
  
        return 0;
    }
    
static uint8_t framework_receive_frame(uint8_t *buffer, size_t size, uint32_t timeout_ms)
{
  if (buffer == NULL || size == 0)
  {
    return 1;
}

  uint32_t start_time = HAL_GetTick();
  size_t received_bytes = 0;
  
  // Receive frame with timeout
  while (received_bytes < size)
  {
    if ((HAL_GetTick() - start_time) > timeout_ms)
    {
      return 2;  // Timeout
    }
    
    // Check if data is available (this would need UART implementation)
    // For now, simulate reception
    if (received_bytes < size)
    {
      buffer[received_bytes] = 0;  // Placeholder
      received_bytes++;
    }
  }
  
  return 0;
}

// Removed unused function implementation

/* End of file -------------------------------------------------------- */