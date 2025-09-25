/**
 * @file       gy87_mpu6050.h
 * @copyright  Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2025-09-19
 * @author     Nhan Vo
 *             
 * @brief      GY87 10DOF IMU sensor driver for STM32F103C8T6
 *             Includes MPU6050 (6-axis), HMC5883L (3-axis magnetometer), 
 *             and BMP180 (barometric pressure sensor) support
 *             
 * @note       This driver provides full sensor integration with I2C communication
 * @example    main.c
 *             Basic IMU data reading and display
 * @example    framework_usage_examples.c
 *             Advanced usage examples and testing
 * @see        https://github.com/vtnhan2/mpu-6050-on-stm32
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __GY87_MPU6050_H
#define __GY87_MPU6050_H

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
/* I2C Scanner Configuration */
#define I2C_SCAN_START_ADDR  (0x08) /*!< I2C scanner start address */
#define I2C_SCAN_END_ADDR    (0x77) /*!< I2C scanner end address */
#define I2C_SCAN_TIMEOUT     (100)  /*!< I2C scanner timeout in ms */

/* Device I2C Addresses */
#define MPU6050_ADDRESS      (0x68) /*!< MPU6050 I2C address (AD0=GND) */
#define HMC5883L_ADDRESS     (0x1E) /*!< HMC5883L magnetometer I2C address */
#define BMP180_ADDRESS       (0x77) /*!< BMP180 pressure sensor I2C address */

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief Sensor data structure for IMU readings
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
  float temperature; /**< Temperature data in °C */
}
imu_data_t;

/**
 * @brief Sensor status enumeration
 */
typedef enum 
{
  SENSOR_STATUS_OK,     /**< Sensor operation successful */
  SENSOR_STATUS_ERROR,  /**< Sensor operation failed */
  SENSOR_STATUS_TIMEOUT /**< Sensor operation timeout */
}
sensor_status_t;

/* Public macros ------------------------------------------------------ */
/**
 * @brief  Convert degrees to radians
 *
 * @param[in]     deg  Angle in degrees
 *
 * @return  Angle in radians
 */
#define DEG_TO_RAD(deg)  ((deg) * 0.017453292519943295f)

/**
 * @brief  Convert radians to degrees
 *
 * @param[in]     rad  Angle in radians
 *
 * @return  Angle in degrees
 */
#define RAD_TO_DEG(rad)  ((rad) * 57.29577951308232f)

/* Public variables --------------------------------------------------- */
extern float g_roll;  /**< Current roll angle in radians */
extern float g_pitch; /**< Current pitch angle in radians */
extern float g_yaw;   /**< Current yaw angle in radians */

/* Public function prototypes ----------------------------------------- */
/**
 * @brief  Initialize I2C scanner and detect connected devices
 *
 * @attention  Call this function before using any sensor functions
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
void gy87_i2c_scanner(void);

/**
 * @brief  Check if I2C device is ready for communication
 *
 * @param[in]     address  I2C device address to check
 *
 * @return  
 *  - 0: Device not ready
 *  - 1: Device ready
 */
uint8_t gy87_i2c_is_device_ready(uint8_t address);

/**
 * @brief  Debug I2C communication status
 *
 * @attention  This function prints debug information to UART
 */
void gy87_debug_i2c_status(void);

/**
 * @brief  Initialize MPU6050 6-axis IMU sensor
 *
 * @attention  Call this function before reading MPU6050 data
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
void gy87_mpu6050_init(void);

/**
 * @brief  Initialize HMC5883L 3-axis magnetometer
 *
 * @attention  Call this function before reading magnetometer data
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
void gy87_hmc5883l_init(void);

/**
 * @brief  Initialize BMP180 barometric pressure sensor
 *
 * @attention  Call this function before reading pressure data
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
void gy87_bmp180_init(void);

/**
 * @brief  Initialize all GY87 sensors
 *
 * @attention  This function initializes MPU6050, HMC5883L, and BMP180
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
void gy87_init_all_sensors(void);

/**
 * @brief  Read data from MPU6050 sensor
 *
 * @attention  Data is stored in internal buffers after successful read
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
uint8_t gy87_mpu6050_read_data(void);

/**
 * @brief  Read data from HMC5883L magnetometer
 *
 * @attention  Data is stored in internal buffers after successful read
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
uint8_t gy87_hmc5883l_read_data(void);

/**
 * @brief  Read data from all sensors and return combined data
 *
 * @param[out]    accel_x  Accelerometer X-axis data in m/s²
 * @param[out]    accel_y  Accelerometer Y-axis data in m/s²
 * @param[out]    accel_z  Accelerometer Z-axis data in m/s²
 * @param[out]    gyro_x   Gyroscope X-axis data in rad/s
 * @param[out]    gyro_y   Gyroscope Y-axis data in rad/s
 * @param[out]    gyro_z   Gyroscope Z-axis data in rad/s
 * @param[out]    mag_x    Magnetometer X-axis data in microTesla
 * @param[out]    mag_y    Magnetometer Y-axis data in microTesla
 * @param[out]    mag_z    Magnetometer Z-axis data in microTesla
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
uint8_t gy87_read_all_sensors(float *accel_x, float *accel_y, float *accel_z,
                              float *gyro_x, float *gyro_y, float *gyro_z,
                              float *mag_x, float *mag_y, float *mag_z);

/**
 * @brief  Get accelerometer X-axis data
 *
 * @return  Accelerometer X-axis data in m/s²
 */
float gy87_mpu6050_get_ax(void);

/**
 * @brief  Get accelerometer Y-axis data
 *
 * @return  Accelerometer Y-axis data in m/s²
 */
float gy87_mpu6050_get_ay(void);

/**
 * @brief  Get accelerometer Z-axis data
 *
 * @return  Accelerometer Z-axis data in m/s²
 */
float gy87_mpu6050_get_az(void);

/**
 * @brief  Get gyroscope X-axis data
 *
 * @return  Gyroscope X-axis data in rad/s
 */
float gy87_mpu6050_get_gx(void);

/**
 * @brief  Get gyroscope Y-axis data
 *
 * @return  Gyroscope Y-axis data in rad/s
 */
float gy87_mpu6050_get_gy(void);

/**
 * @brief  Get gyroscope Z-axis data
 *
 * @return  Gyroscope Z-axis data in rad/s
 */
float gy87_mpu6050_get_gz(void);

/**
 * @brief  Get MPU6050 internal temperature
 *
 * @return  Temperature data in °C
 */
float gy87_mpu6050_get_temperature(void);

/**
 * @brief  Get magnetometer X-axis data
 *
 * @return  Magnetometer X-axis data in microTesla
 */
float gy87_hmc5883l_get_mx(void);

/**
 * @brief  Get magnetometer Y-axis data
 *
 * @return  Magnetometer Y-axis data in microTesla
 */
float gy87_hmc5883l_get_my(void);

/**
 * @brief  Get magnetometer Z-axis data
 *
 * @return  Magnetometer Z-axis data in microTesla
 */
float gy87_hmc5883l_get_mz(void);

/**
 * @brief  Calculate roll, pitch, yaw angles from sensor data
 *
 * @param[out]    roll   Roll angle in radians
 * @param[out]    pitch  Pitch angle in radians
 * @param[out]    yaw    Yaw angle in radians
 *
 * @attention  This function uses accelerometer and magnetometer data
 */
void gy87_mpu6050_calculate_angles(float *roll, float *pitch, float *yaw);

/**
 * @brief  Apply complementary filter for orientation estimation
 *
 * @param[inout]  roll   Roll angle in radians
 * @param[inout]  pitch  Pitch angle in radians
 * @param[inout]  yaw    Yaw angle in radians
 * @param[in]     dt     Time step in seconds
 *
 * @attention  This function fuses accelerometer and gyroscope data
 */
void gy87_mpu6050_complementary_filter(float *roll, float *pitch, float *yaw, float dt);

/**
 * @brief  Display MPU6050 individual sensor data
 *
 * @param[in]     period_ms  Display period in milliseconds
 *
 * @attention  This function prints data to UART
 */
void gy87_display_mpu6050_individual(uint32_t period_ms);

/**
 * @brief  Display accelerometer, gyroscope, and magnetometer data
 *
 * @param[in]     ax  Accelerometer X-axis data in m/s²
 * @param[in]     ay  Accelerometer Y-axis data in m/s²
 * @param[in]     az  Accelerometer Z-axis data in m/s²
 * @param[in]     gx  Gyroscope X-axis data in rad/s
 * @param[in]     gy  Gyroscope Y-axis data in rad/s
 * @param[in]     gz  Gyroscope Z-axis data in rad/s
 * @param[in]     mx  Magnetometer X-axis data in microTesla
 * @param[in]     my  Magnetometer Y-axis data in microTesla
 * @param[in]     mz  Magnetometer Z-axis data in microTesla
 * @param[in]     period_ms  Display period in milliseconds
 *
 * @attention  This function prints formatted data to UART
 */
void gy87_display_agm(float ax, float ay, float az, 
                      float gx, float gy, float gz, 
                      float mx, float my, float mz, 
                      uint32_t period_ms);

/**
 * @brief  Display formatted sensor data
 *
 * @param[in]     ax  Accelerometer X-axis data in m/s²
 * @param[in]     ay  Accelerometer Y-axis data in m/s²
 * @param[in]     az  Accelerometer Z-axis data in m/s²
 * @param[in]     gx  Gyroscope X-axis data in rad/s
 * @param[in]     gy  Gyroscope Y-axis data in rad/s
 * @param[in]     gz  Gyroscope Z-axis data in rad/s
 * @param[in]     mx  Magnetometer X-axis data in microTesla
 * @param[in]     my  Magnetometer Y-axis data in microTesla
 * @param[in]     mz  Magnetometer Z-axis data in microTesla
 * @param[in]     period_ms  Display period in milliseconds
 *
 * @attention  This function prints formatted data to UART
 */
void gy87_display_formatted_data(float ax, float ay, float az, 
                                 float gx, float gy, float gz, 
                                 float mx, float my, float mz, 
                                 uint32_t period_ms);

/**
 * @brief  Display all sensors data with AGM format
 *
 * @param[in]     period_ms  Display period in milliseconds
 *
 * @attention  This function reads and displays all sensor data
 */
void gy87_display_all_sensors_agm(uint32_t period_ms);

/**
 * @brief  Display MPU6050 only data with AGM format
 *
 * @param[in]     period_ms  Display period in milliseconds
 *
 * @attention  This function reads and displays only MPU6050 data
 */
void gy87_display_mpu6050_only_agm(uint32_t period_ms);

/**
 * @brief  Test HMC5883L magnetometer functionality
 *
 * @attention  This function performs magnetometer self-test
 */
void gy87_test_hmc5883l_only(void);

/**
 * @brief  Debug HMC5883L: presence, ID registers, config, status, sample data
 *
 * @attention  Prints diagnostic info over UART
 */
void gy87_hmc5883l_debug(void);

/**
 * @brief  Compatibility wrapper to display all sensors (AGM) over UART
 *
 * @param[in]     period_ms  Display period in milliseconds
 *
 * @attention  Calls gy87_display_all_sensors_agm(period_ms)
 */
void GY87_Display_All_Sensors_AGM(uint32_t period_ms);

/**
 * @brief  Log error message to UART
 *
 * @param[in]     function   Function name where error occurred
 * @param[in]     operation  Operation that failed
 * @param[in]     status     Error status code
 *
 * @attention  This function prints error information to UART
 */
void gy87_log_error(const char *function, const char *operation, int status);

/**
 * @brief  Log info message to UART
 *
 * @param[in]     message  Information message to log
 *
 * @attention  This function prints information to UART
 */
void gy87_log_info(const char *message);

#endif // __GY87_MPU6050_H

/* End of file -------------------------------------------------------- */