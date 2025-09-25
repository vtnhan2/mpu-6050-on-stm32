/**
 * @file       framework.h
 * @copyright  Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2025-09-19
 * @author     Nhan Vo
 *             
 * @brief      Data transmission framework for GY87 IMU sensor
 *             Provides structured data protocol for UART communication
 *             
 * @note       This framework handles data packaging and transmission
 * @example    framework.c
 *             Framework implementation and usage examples
 * @see        https://github.com/vtnhan2/mpu-6050-on-stm32
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __FRAMEWORK_H
#define __FRAMEWORK_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <stddef.h>

/* Public defines ----------------------------------------------------- */
/* Data Frame Protocol Constants */
#define FRAME_START_BYTE     (0xAA) /*!< Frame start delimiter */
#define FRAME_END_BYTE       (0x00) /*!< Frame end delimiter */
#define FRAME_DATA_SIZE      (36)   /*!< Payload size: accel+gyro (24) + padding (12) */
#define FRAME_CHECKSUM_SIZE  (2)    /*!< Checksum size in bytes */
#define FRAME_HEADER_SIZE    (1)    /*!< Frame header size in bytes */
#define FRAME_FOOTER_SIZE    (FRAME_CHECKSUM_SIZE + 1) /*!< Footer: checksum + end byte */
#define FRAME_TOTAL_SIZE     (FRAME_HEADER_SIZE + FRAME_DATA_SIZE + FRAME_FOOTER_SIZE) /*!< Total frame size in bytes */

/* Sensor Data Offsets */
#define ACCEL_DATA_OFFSET    (0)    /*!< Accelerometer data offset */
#define GYRO_DATA_OFFSET     (12)   /*!< Gyroscope data offset */
#define MAG_DATA_OFFSET      (24)   /*!< Magnetometer data offset */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Sensor data structure for transmission
 */
typedef struct 
{
  float accel_x;  /**< Accelerometer X-axis data in m/s² */
  float accel_y;  /**< Accelerometer Y-axis data in m/s² */
  float accel_z;  /**< Accelerometer Z-axis data in m/s² */
  float gyro_x;   /**< Gyroscope X-axis data in rad/s */
  float gyro_y;   /**< Gyroscope Y-axis data in rad/s */
  float gyro_z;   /**< Gyroscope Z-axis data in rad/s */
  float mag_x;    /**< Magnetometer X-axis data in microTesla */
  float mag_y;    /**< Magnetometer Y-axis data in microTesla */
  float mag_z;    /**< Magnetometer Z-axis data in microTesla */
}
sensor_frame_t;

/**
 * @brief Frame transmission status enumeration
 */
typedef enum 
{
  FRAME_STATUS_OK,      /**< Frame transmission successful */
  FRAME_STATUS_ERROR,   /**< Frame transmission failed */
  FRAME_STATUS_TIMEOUT  /**< Frame transmission timeout */
}
frame_status_t;

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Calculate frame checksum
 *
 * @param[in]     data      Data buffer
 * @param[in]     length    Data length
 *
 * @return  Calculated checksum
 */
#define FRAME_CALCULATE_CHECKSUM(data, length)  \
  ({ \
    uint16_t checksum = 0; \
    for (int i = 0; i < (length); i++) { \
      checksum = (uint16_t)(checksum + (data)[i]); \
    } \
    checksum; \
  })

/**
 * @brief  Pack float data into byte array
 *
 * @param[in]     value     Float value to pack
 * @param[out]    buffer    Output byte buffer
 * @param[in]     offset    Buffer offset
 *
 * @attention  This macro assumes little-endian byte order
 */
#define PACK_FLOAT(value, buffer, offset) \
  do { \
    union { float f; uint8_t b[4]; } u; \
    u.f = (value); \
    (buffer)[(offset)] = u.b[0]; \
    (buffer)[(offset) + 1] = u.b[1]; \
    (buffer)[(offset) + 2] = u.b[2]; \
    (buffer)[(offset) + 3] = u.b[3]; \
  } while (0)

/**
 * @brief  Unpack byte array into float data
 *
 * @param[in]     buffer    Input byte buffer
 * @param[in]     offset    Buffer offset
 *
 * @return  Unpacked float value
 *
 * @attention  This macro assumes little-endian byte order
 */
#define UNPACK_FLOAT(buffer, offset) \
  ({ \
    union { float f; uint8_t b[4]; } u; \
    u.b[0] = (buffer)[(offset)]; \
    u.b[1] = (buffer)[(offset) + 1]; \
    u.b[2] = (buffer)[(offset) + 2]; \
    u.b[3] = (buffer)[(offset) + 3]; \
    u.f; \
  })

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Initialize data transmission framework
 *
 * @attention  Call this function before using any framework functions
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
void framework_init(void);

/**
 * @brief  Pack sensor data into transmission frame
 *
 * @param[in]     sensor_data  Sensor data structure
 * @param[out]    frame_buffer Output frame buffer
 * @param[in]     buffer_size  Frame buffer size
 *
 * @attention  Frame buffer must be at least FRAME_TOTAL_SIZE bytes
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
uint8_t framework_pack_sensor_data(const sensor_frame_t *sensor_data, 
                                   uint8_t *frame_buffer, 
                                   size_t buffer_size);

/**
 * @brief  Unpack sensor data from transmission frame
 *
 * @param[in]     frame_buffer Input frame buffer
 * @param[in]     buffer_size  Frame buffer size
 * @param[out]    sensor_data  Output sensor data structure
 *
 * @attention  Frame buffer must contain valid frame data
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
uint8_t framework_unpack_sensor_data(const uint8_t *frame_buffer, 
                                     size_t buffer_size, 
                                     sensor_frame_t *sensor_data);

/**
 * @brief  Transmit sensor data frame via UART
 *
 * @param[in]     sensor_data  Sensor data structure
 *
 * @attention  This function blocks until transmission is complete
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
uint8_t framework_transmit_sensor_data(const sensor_frame_t *sensor_data);

/**
 * @brief  Receive sensor data frame via UART
 *
 * @param[out]    sensor_data  Output sensor data structure
 * @param[in]     timeout_ms   Reception timeout in milliseconds
 *
 * @attention  This function blocks until frame is received or timeout
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 *  - 2: Timeout
 */
uint8_t framework_receive_sensor_data(sensor_frame_t *sensor_data, uint32_t timeout_ms);

/**
 * @brief  Validate received frame
 *
 * @param[in]     frame_buffer Input frame buffer
 * @param[in]     buffer_size  Frame buffer size
 *
 * @attention  This function checks frame structure and checksum
 *
 * @return  
 *  - 0: Valid frame
 *  - 1: Invalid frame
 */
uint8_t framework_validate_frame(const uint8_t *frame_buffer, size_t buffer_size);

/**
 * @brief  Get framework statistics
 *
 * @param[out]    frames_sent     Number of frames sent
 * @param[out]    frames_received Number of frames received
 * @param[out]    errors_count    Number of transmission errors
 *
 * @attention  Statistics are reset when framework is initialized
 */
void framework_get_statistics(uint32_t *frames_sent, 
                              uint32_t *frames_received, 
                              uint32_t *errors_count);

/**
 * @brief  Reset framework statistics
 *
 * @attention  This function resets all framework counters
 */
void framework_reset_statistics(void);

#endif // __FRAMEWORK_H

/* End of file -------------------------------------------------------- */