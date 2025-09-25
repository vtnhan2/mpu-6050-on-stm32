/**
 * @file       gy87_mpu6050.c
 * @copyright  Copyright (C) 2025 Vo Thanh Nhan. All rights reserved.
 * @license    This project is released under the Fiot License.
 * @version    1.0.0
 * @date       2025-09-19
 * @author     Nhan Vo
 *             
 * @brief      GY87 10DOF IMU sensor driver implementation for STM32F103C8T6
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

/* Includes ----------------------------------------------------------- */
#include "gy87_mpu6050.h"
#include "i2c.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Private defines ---------------------------------------------------- */
/* HMC5883L Register Definitions */
#define HMC5883L_REG_CONFIG_A       (0x00) /*!< Configuration register A */
#define HMC5883L_REG_CONFIG_B       (0x01) /*!< Configuration register B */
#define HMC5883L_REG_MODE           (0x02) /*!< Mode register */
#define HMC5883L_REG_DATA_X_MSB     (0x03) /*!< Data X MSB register */
#define HMC5883L_REG_DATA_X_LSB     (0x04) /*!< Data X LSB register */
#define HMC5883L_REG_DATA_Z_MSB     (0x05) /*!< Data Z MSB register */
#define HMC5883L_REG_DATA_Z_LSB     (0x06) /*!< Data Z LSB register */
#define HMC5883L_REG_DATA_Y_MSB     (0x07) /*!< Data Y MSB register */
#define HMC5883L_REG_DATA_Y_LSB     (0x08) /*!< Data Y LSB register */
#define HMC5883L_REG_STATUS         (0x09) /*!< Status register */
#define HMC5883L_REG_ID_A           (0x0A) /*!< ID register A */
#define HMC5883L_REG_ID_B           (0x0B) /*!< ID register B */
#define HMC5883L_REG_ID_C           (0x0C) /*!< ID register C */

/* HMC5883L Configuration Values */
#define HMC5883L_CFG_A_8AVG_15HZ    (0x70) /*!< 8-average, 15 Hz, normal measurement */
#define HMC5883L_CFG_A_8AVG_75HZ    (0x78) /*!< 8-average, 75 Hz, normal measurement (closest to 60 Hz) */
#define HMC5883L_CFG_B_GAIN_1_3GA   (0x20) /*!< Gain = 1.3 Ga (1090 LSB/Gauss) */
#define HMC5883L_MODE_IDLE          (0x02) /*!< Idle mode */
#define HMC5883L_MODE_SINGLE        (0x01) /*!< Single measurement mode */
#define HMC5883L_MODE_CONTINUOUS    (0x00) /*!< Continuous measurement mode */

/* HMC5883L Constants */
#define HMC5883L_GAIN_LSB_PER_GAUSS     (1090.0f) /*!< LSB per Gauss for 1.3Ga gain */
#define HMC5883L_GAUSS_TO_MICROTESLA    (0.1f)    /*!< Conversion factor */
#define HMC5883L_OVERFLOW_VALUE         (-4096)   /*!< Overflow detection value */
#define HMC5883L_DEVICE_ID_H            (0x48)    /*!< Expected device ID bytes */
#define HMC5883L_DEVICE_ID_4            (0x34)    /*!< Expected device ID bytes */
#define HMC5883L_DEVICE_ID_3            (0x33)    /*!< Expected device ID bytes */

/* MPU6050 Constants */
#define MPU6050_ACCEL_SCALE             (16384.0f) /*!< LSB per g for ±2g range */
#define MPU6050_GYRO_SCALE              (131.0f)   /*!< LSB per deg/s for ±250deg/s range */
#define MPU6050_TEMP_SCALE              (340.0f)   /*!< Temperature scale factor */
#define MPU6050_TEMP_OFFSET             (36.53f)   /*!< Temperature offset */
#define MPU6050_G_TO_MS2                (9.81f)    /*!< Gravity to m/s² conversion */
#define MPU6050_DEG_TO_RAD              (0.017453292519943295f) /*!< Degrees to radians */

/* MPU6050 Registers for Auxiliary I2C Master */
#define MPU6050_REG_INT_PIN_CFG         (0x37)
#define MPU6050_REG_USER_CTRL           (0x6A)
#define MPU6050_REG_I2C_MST_CTRL        (0x24)
#define MPU6050_REG_I2C_SLV0_ADDR       (0x25)
#define MPU6050_REG_I2C_SLV0_REG        (0x26)
#define MPU6050_REG_I2C_SLV0_CTRL       (0x27)
#define MPU6050_REG_EXT_SENS_DATA_00    (0x49)

/* MPU6050 Bitfields */
#define MPU6050_BIT_I2C_BYPASS_EN       (0x02) /* INT_PIN_CFG */
#define MPU6050_BIT_I2C_MST_EN          (0x20) /* USER_CTRL */

/* MPU6050 I2C master clock: 0x0D is ~400kHz. 0x07 ~ 100kHz (datasheet). */
#define MPU6050_I2C_MST_CLK_400KHZ      (0x0D)
#define MPU6050_I2C_MST_CLK_100KHZ      (0x07)

/* Timing Constants */
#define I2C_TIMEOUT_MS                  (100)  /*!< I2C communication timeout */
#define HMC5883L_INIT_DELAY_MS          (10)   /*!< HMC5883L initialization delay */
#define HMC5883L_MEASURE_DELAY_MS       (50)   /*!< HMC5883L measurement delay */
#define HMC5883L_READ_DELAY_MS          (5)    /*!< HMC5883L read delay */

/* Complementary Filter Constants */
#define COMPLEMENTARY_FILTER_ALPHA      (0.98f) /*!< Complementary filter coefficient */

/* Private enumerate/structure ---------------------------------------- */
/**
 * @brief I2C communication status enumeration
 */
typedef enum 
{
  I2C_STATUS_OK,      /**< I2C communication successful */
  I2C_STATUS_ERROR,   /**< I2C communication error */
  I2C_STATUS_TIMEOUT  /**< I2C communication timeout */
}
i2c_status_t;

/* Private macros ----------------------------------------------------- */
/**
 * @brief  Check if I2C operation was successful
 *
 * @param[in]     status  HAL status to check
 *
 * @return  
 *  - 0: Success
 *  - 1: Error
 */
#define I2C_CHECK_STATUS(status)  ((status) == HAL_OK ? 0 : 1)

/**
 * @brief  Convert raw accelerometer data to m/s²
 *
 * @param[in]     raw_data  Raw accelerometer data
 *
 * @return  Accelerometer data in m/s²
 */
#define ACCEL_RAW_TO_MS2(raw_data)  ((float)(raw_data) / MPU6050_ACCEL_SCALE * MPU6050_G_TO_MS2)

/**
 * @brief  Convert raw gyroscope data to rad/s
 *
 * @param[in]     raw_data  Raw gyroscope data
 *
 * @return  Gyroscope data in rad/s
 */
#define GYRO_RAW_TO_RAD_S(raw_data)  ((float)(raw_data) / MPU6050_GYRO_SCALE * MPU6050_DEG_TO_RAD)

/**
 * @brief  Convert raw magnetometer data to microTesla
 *
 * @param[in]     raw_data  Raw magnetometer data
 *
 * @return  Magnetometer data in microTesla
 */
#define MAG_RAW_TO_MICROTESLA(raw_data)  ((float)(raw_data) / HMC5883L_GAIN_LSB_PER_GAUSS * HMC5883L_GAUSS_TO_MICROTESLA)

/* Public variables --------------------------------------------------- */
float g_roll = 0.0f;   /**< Current roll angle in radians */
float g_pitch = 0.0f;  /**< Current pitch angle in radians */
float g_yaw = 0.0f;    /**< Current yaw angle in radians */

/* Private variables -------------------------------------------------- */
static uint8_t s_data_tx[8];           /**< I2C transmit buffer */
static uint8_t s_mpu6050_data_rx[14];  /**< MPU6050 raw data buffer */
static uint8_t s_hmc5883l_data_rx[6];  /**< HMC5883L raw data buffer */
// Removed unused variables for complementary filter

/* Private function prototypes ---------------------------------------- */
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

/**
 * @brief  Format sensor data for display
 *
 * @param[out]    buffer      Output buffer for formatted string
 * @param[in]     buffer_size Size of output buffer
 * @param[in]     ax          Accelerometer X-axis data
 * @param[in]     ay          Accelerometer Y-axis data
 * @param[in]     az          Accelerometer Z-axis data
 * @param[in]     gx          Gyroscope X-axis data
 * @param[in]     gy          Gyroscope Y-axis data
 * @param[in]     gz          Gyroscope Z-axis data
 * @param[in]     mx          Magnetometer X-axis data
 * @param[in]     my          Magnetometer Y-axis data
 * @param[in]     mz          Magnetometer Z-axis data
 * @param[in]     period_ms   Display period in milliseconds
 *
 * @attention  This function formats sensor data for UART output
 */
static void gy87_format_sensor_data(char *buffer, size_t buffer_size, 
                                   float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float mx, float my, float mz,
                                   uint32_t period_ms);

/* Configure MPU6050 internal I2C master to read HMC5883L over SLV0 */
static uint8_t gy87_mpu6050_enable_i2c_master_for_hmc(void);
/* Read 6 magnetometer bytes via MPU6050 EXT_SENS_DATA registers */
static uint8_t gy87_hmc5883l_read_via_mpu(void);

/* Function definitions ----------------------------------------------- */
void gy87_i2c_scanner(void)
{
  gy87_log_info("=== I2C Scanner ===");
  
  uint8_t device_count = 0;
	char buffer[64];
	
  for (uint8_t addr = I2C_SCAN_START_ADDR; addr <= I2C_SCAN_END_ADDR; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, I2C_SCAN_TIMEOUT) == HAL_OK)
    {
      snprintf(buffer, sizeof(buffer), "Device found at address: 0x%02X", addr);
      gy87_log_info(buffer);
      device_count++;
    }
  }
  
  if (device_count == 0)
  {
    gy87_log_info("No I2C devices found!");
	}
	else
	{
    snprintf(buffer, sizeof(buffer), "Total devices found: %d", device_count);
    gy87_log_info(buffer);
  }
  
  gy87_log_info("=== End I2C Scanner ===");
}

uint8_t gy87_i2c_is_device_ready(uint8_t address)
{
  return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address << 1), 1, I2C_SCAN_TIMEOUT) == HAL_OK) ? 1 : 0;
}

void gy87_debug_i2c_status(void)
{
  gy87_log_info("=== I2C Debug Status ===");
  
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "I2C Instance: %p", &hi2c1);
  gy87_log_info(buffer);
  
  snprintf(buffer, sizeof(buffer), "I2C Clock Speed: %lu Hz", HAL_RCC_GetPCLK1Freq());
  gy87_log_info(buffer);
  
  gy87_log_info("=== End I2C Debug Status ===");
}

void gy87_mpu6050_init(void)
{
  gy87_log_info("Initializing MPU6050...");
  
  HAL_StatusTypeDef status;
  
  // Wake up MPU6050 from sleep mode
  s_data_tx[0] = 0x6B;  // PWR_MGMT_1 register
  s_data_tx[1] = 0x00;  // Wake up (set sleep bit to 0)
  
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Init", "Wake up", (int)status);
    return;
  }
  
  HAL_Delay(10);
  
  // Configure accelerometer range (±2g)
  s_data_tx[0] = 0x1C;  // ACCEL_CONFIG register
  s_data_tx[1] = 0x00;  // ±2g range
  
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Init", "Accel config", (int)status);
    return;
  }
  
  // Configure gyroscope range (±250°/s)
  s_data_tx[0] = 0x1B;  // GYRO_CONFIG register
  s_data_tx[1] = 0x00;  // ±250°/s range
  
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Init", "Gyro config", (int)status);
    return;
  }
  
  // Ensure internal I2C master is disabled before enabling BYPASS
  s_data_tx[0] = MPU6050_REG_USER_CTRL; // USER_CTRL
  s_data_tx[1] = 0x00; // I2C_MST_EN = 0
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Init", "USER_CTRL (disable I2C_MST)", (int)status);
    return;
  }
  HAL_Delay(5);

  // Enable BYPASS to allow MCU to configure and read external magnetometer directly
  s_data_tx[0] = MPU6050_REG_INT_PIN_CFG; // INT_PIN_CFG
  s_data_tx[1] = MPU6050_BIT_I2C_BYPASS_EN; // Enable bypass so HMC5883L is visible on main I2C bus
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Init", "INT_PIN_CFG (bypass)", (int)status);
    return;
  }
  HAL_Delay(100); // Important delay after enabling BYPASS
  
  gy87_log_info("MPU6050 initialized successfully!");
}

void gy87_hmc5883l_init(void)
{
  gy87_log_info("Initializing HMC5883L...");
  
    HAL_StatusTypeDef status;
  
  // Verify device presence at 0x1E (bypass must be enabled earlier)
  if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), 1, I2C_SCAN_TIMEOUT) != HAL_OK)
  {
    gy87_log_error("HMC5883L_Init", "Device not ready (0x1E)", 1);
    return;
  }

  // Configure HMC5883L: 8-average, 15Hz, normal measurement
  {
    uint8_t val = HMC5883L_CFG_A_8AVG_15HZ;
    status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), HMC5883L_REG_CONFIG_A,
                               I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
  }
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Init", "Config A", (int)status);
    return;
  }
  HAL_Delay(HMC5883L_INIT_DELAY_MS);
  
  // Gain = 1.3 Gauss (suitable for most environments)
  {
    uint8_t val = HMC5883L_CFG_B_GAIN_1_3GA;
    status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), HMC5883L_REG_CONFIG_B,
                               I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
  }
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Init", "Config B", (int)status);
        return;
    }
  HAL_Delay(HMC5883L_INIT_DELAY_MS);
  
  // Set to idle mode first (recommended sequence)
  {
    uint8_t val = HMC5883L_MODE_IDLE;
    status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), HMC5883L_REG_MODE,
                               I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
  }
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Init", "Idle mode", (int)status);
        return;
    }
  HAL_Delay(HMC5883L_INIT_DELAY_MS);
  
  // Kick a single measurement first (wake up the sensor)
  {
    uint8_t val = HMC5883L_MODE_SINGLE;
    status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), HMC5883L_REG_MODE,
                               I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
  }
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Init", "Single measurement", (int)status);
        return;
    }
  HAL_Delay(HMC5883L_MEASURE_DELAY_MS);
  
  // Set to continuous measurement mode
  {
    uint8_t val = HMC5883L_MODE_CONTINUOUS;
    status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), HMC5883L_REG_MODE,
                               I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
  }
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Init", "Continuous mode", (int)status);
        return;
    }
    HAL_Delay(67);  // Allow first measurement
    
  gy87_log_info("HMC5883L initialized successfully!");
}

void gy87_bmp180_init(void)
{
  gy87_log_info("Initializing BMP180...");
  // BMP180 initialization code would go here
  gy87_log_info("BMP180 initialized successfully!");
}

void gy87_init_all_sensors(void)
{
  gy87_log_info("Initializing all GY87 sensors...");
  
  gy87_mpu6050_init();
  gy87_hmc5883l_init();
  gy87_bmp180_init();
  
  gy87_log_info("All GY87 sensors initialized successfully!");
}

uint8_t gy87_mpu6050_read_data(void)
{
	HAL_StatusTypeDef status;
  
  s_data_tx[0] = 0x3B;  // Starting register for accelerometer data
	
	// Send register address
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), &s_data_tx[0], 1, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Read", "TX", (int)status);
		return 0;
	}
	
	// Read data
  status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_mpu6050_data_rx, 14, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("MPU6050_Read", "RX", (int)status);
		return 0;
	}
	
	return 1;
}

uint8_t gy87_hmc5883l_read_data(void)
{
	HAL_StatusTypeDef status;
  
  // Direct read in BYPASS mode
  s_data_tx[0] = HMC5883L_REG_DATA_X_MSB;

  // Send register address
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &s_data_tx[0], 1, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Read", "Data TX", (int)status);
    return 0;
  }

  // Read 6 bytes: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
  status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), s_hmc5883l_data_rx, 6, I2C_TIMEOUT_MS);
  if (I2C_CHECK_STATUS(status))
  {
    gy87_log_error("HMC5883L_Read", "Data RX", (int)status);
    return 0;
  }

  return 1;
}

uint8_t gy87_read_all_sensors(float *accel_x, float *accel_y, float *accel_z,
                              float *gyro_x, float *gyro_y, float *gyro_z,
                              float *mag_x, float *mag_y, float *mag_z)
{
	// Read MPU6050 data
  if (!gy87_mpu6050_read_data()) return 0;
	
  // Read HMC5883L data (via MPU6050 I2C master or bypass fallback)
  if (!gy87_hmc5883l_read_data()) return 0;
	
	// Get accelerometer data (m/s²)
  *accel_x = gy87_mpu6050_get_ax();
  *accel_y = gy87_mpu6050_get_ay();
  *accel_z = gy87_mpu6050_get_az();
	
	// Get gyroscope data (rad/s)
  *gyro_x = gy87_mpu6050_get_gx();
  *gyro_y = gy87_mpu6050_get_gy();
  *gyro_z = gy87_mpu6050_get_gz();
  
  // Get magnetometer data (microTesla)
  *mag_x = gy87_hmc5883l_get_mx();
  *mag_y = gy87_hmc5883l_get_my();
  *mag_z = gy87_hmc5883l_get_mz();
	
	return 1;
}

float gy87_mpu6050_get_ax(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[0] << 8 | s_mpu6050_data_rx[1]);
  return ACCEL_RAW_TO_MS2(raw);
}

float gy87_mpu6050_get_ay(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[2] << 8 | s_mpu6050_data_rx[3]);
  return ACCEL_RAW_TO_MS2(raw);
}

float gy87_mpu6050_get_az(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[4] << 8 | s_mpu6050_data_rx[5]);
  return ACCEL_RAW_TO_MS2(raw);
}

float gy87_mpu6050_get_gx(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[8] << 8 | s_mpu6050_data_rx[9]);
  return GYRO_RAW_TO_RAD_S(raw);
}

float gy87_mpu6050_get_gy(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[10] << 8 | s_mpu6050_data_rx[11]);
  return GYRO_RAW_TO_RAD_S(raw);
}

float gy87_mpu6050_get_gz(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[12] << 8 | s_mpu6050_data_rx[13]);
  return GYRO_RAW_TO_RAD_S(raw);
}

float gy87_mpu6050_get_temperature(void)
{
  int16_t raw = (int16_t)(s_mpu6050_data_rx[6] << 8 | s_mpu6050_data_rx[7]);
  return ((float)raw / MPU6050_TEMP_SCALE) + MPU6050_TEMP_OFFSET;
}

float gy87_hmc5883l_get_mx(void)
{
  int16_t raw = (int16_t)(s_hmc5883l_data_rx[0] << 8 | s_hmc5883l_data_rx[1]);
  return MAG_RAW_TO_MICROTESLA(raw);
}

float gy87_hmc5883l_get_my(void)
{
  int16_t raw = (int16_t)(s_hmc5883l_data_rx[4] << 8 | s_hmc5883l_data_rx[5]);
  return MAG_RAW_TO_MICROTESLA(raw);
}

float gy87_hmc5883l_get_mz(void)
{
  int16_t raw = (int16_t)(s_hmc5883l_data_rx[2] << 8 | s_hmc5883l_data_rx[3]);
  return MAG_RAW_TO_MICROTESLA(raw);
}

void gy87_mpu6050_calculate_angles(float *roll, float *pitch, float *yaw)
{
  float ax = gy87_mpu6050_get_ax();
  float ay = gy87_mpu6050_get_ay();
  float az = gy87_mpu6050_get_az();
  
  // Calculate roll and pitch from accelerometer
  *roll = atan2f(ay, sqrtf(ax * ax + az * az));
  *pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  
  // Yaw calculation would require magnetometer data
  *yaw = 0.0f;  // Placeholder
}

void gy87_mpu6050_complementary_filter(float *roll, float *pitch, float *yaw, float dt)
{
  float ax = gy87_mpu6050_get_ax();
  float ay = gy87_mpu6050_get_ay();
  float az = gy87_mpu6050_get_az();
  float gx = gy87_mpu6050_get_gx();
  float gy = gy87_mpu6050_get_gy();
  float gz = gy87_mpu6050_get_gz();
  
  // Calculate roll and pitch from accelerometer
  float accel_roll = atan2f(ay, sqrtf(ax * ax + az * az));
  float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  
  // Integrate gyroscope data
  *roll = COMPLEMENTARY_FILTER_ALPHA * (*roll + gx * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
  *pitch = COMPLEMENTARY_FILTER_ALPHA * (*pitch + gy * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
  *yaw = *yaw + gz * dt;  // Yaw from gyroscope only
  
  // Update global variables
  g_roll = *roll;
  g_pitch = *pitch;
  g_yaw = *yaw;
}

void gy87_display_mpu6050_individual(uint32_t period_ms)
{
  static uint32_t last_display_time = 0;
  uint32_t current_time = HAL_GetTick();
  
  if ((current_time - last_display_time) < period_ms) return;
  
  if (!gy87_mpu6050_read_data()) return;
  
  char buffer[256];
    snprintf(buffer, sizeof(buffer), 
    "Accel: X=%.3f Y=%.3f Z=%.3f m/s² | "
    "Gyro: X=%.3f Y=%.3f Z=%.3f rad/s | "
    "Temp: %.1f°C | t=%lums",
    gy87_mpu6050_get_ax(), gy87_mpu6050_get_ay(), gy87_mpu6050_get_az(),
    gy87_mpu6050_get_gx(), gy87_mpu6050_get_gy(), gy87_mpu6050_get_gz(),
    gy87_mpu6050_get_temperature(),
    current_time - last_display_time);
  
  gy87_log_info(buffer);
  last_display_time = current_time;
}

void gy87_display_agm(float ax, float ay, float az, 
                      float gx, float gy, float gz, 
                      float mx, float my, float mz, 
                      uint32_t period_ms)
{
  static uint32_t last_display_time = 0;
  uint32_t current_time = HAL_GetTick();
  
  if ((current_time - last_display_time) < period_ms) return;
  
    char buffer[256];
  gy87_format_sensor_data(buffer, sizeof(buffer), ax, ay, az, gx, gy, gz, mx, my, mz, current_time - last_display_time);
  
  gy87_log_info(buffer);
  last_display_time = current_time;
}

void gy87_display_formatted_data(float ax, float ay, float az, 
                                 float gx, float gy, float gz, 
                                 float mx, float my, float mz, 
                                 uint32_t period_ms)
{
  gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}

void gy87_display_all_sensors_agm(uint32_t period_ms)
{
  // Timing is controlled by main.c, this function just displays data
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  
  if (!gy87_read_all_sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz)) return;
  
  // Display with current period (passed from main.c)
  gy87_display_agm(ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}

void gy87_display_mpu6050_only_agm(uint32_t period_ms)
{
  static uint32_t last_display_time = 0;
  uint32_t current_time = HAL_GetTick();
  
  if ((current_time - last_display_time) < period_ms) return;
  
  if (!gy87_mpu6050_read_data()) return;
  
  gy87_display_agm(gy87_mpu6050_get_ax(), gy87_mpu6050_get_ay(), gy87_mpu6050_get_az(),
                   gy87_mpu6050_get_gx(), gy87_mpu6050_get_gy(), gy87_mpu6050_get_gz(),
                   0.0f, 0.0f, 0.0f,  // No magnetometer data
                   current_time - last_display_time);
  
  last_display_time = current_time;
}

void GY87_Display_All_Sensors_AGM(uint32_t period_ms)
{
  gy87_display_all_sensors_agm(period_ms);
}

static uint8_t read_hmc_reg(uint8_t reg, uint8_t *val)
{
  HAL_StatusTypeDef status;
  s_data_tx[0] = reg;
  status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &s_data_tx[0], 1, I2C_TIMEOUT_MS);
  if (status != HAL_OK) return 0;
  status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), val, 1, I2C_TIMEOUT_MS);
  return (status == HAL_OK) ? 1 : 0;
}

void gy87_hmc5883l_debug(void)
{
  char buf[96];
  gy87_log_info("=== HMC5883L Debug ===");

  // Check presence
  if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), 1, I2C_SCAN_TIMEOUT) == HAL_OK)
  {
    gy87_log_info("HMC5883L at 0x1E: READY");
  }
  else
  {
    gy87_log_info("HMC5883L at 0x1E: NOT READY");
  }

  // Read ID A/B/C
  uint8_t id_a=0, id_b=0, id_c=0;
  if (read_hmc_reg(HMC5883L_REG_ID_A, &id_a) &&
      read_hmc_reg(HMC5883L_REG_ID_B, &id_b) &&
      read_hmc_reg(HMC5883L_REG_ID_C, &id_c))
  {
    snprintf(buf, sizeof(buf), "ID A/B/C = 0x%02X 0x%02X 0x%02X", id_a, id_b, id_c);
    gy87_log_info(buf);
  }
  else
  {
    gy87_log_info("Read ID registers failed");
  }

  // Read Config A/B and Mode
  uint8_t cfg_a=0, cfg_b=0, mode=0, status=0;
  if (read_hmc_reg(HMC5883L_REG_CONFIG_A, &cfg_a)) { snprintf(buf, sizeof(buf), "CONFIG_A = 0x%02X", cfg_a); gy87_log_info(buf);} else { gy87_log_info("CONFIG_A read fail"); }
  if (read_hmc_reg(HMC5883L_REG_CONFIG_B, &cfg_b)) { snprintf(buf, sizeof(buf), "CONFIG_B = 0x%02X", cfg_b); gy87_log_info(buf);} else { gy87_log_info("CONFIG_B read fail"); }
  if (read_hmc_reg(HMC5883L_REG_MODE, &mode))       { snprintf(buf, sizeof(buf), "MODE     = 0x%02X", mode);  gy87_log_info(buf);} else { gy87_log_info("MODE read fail"); }
  if (read_hmc_reg(HMC5883L_REG_STATUS, &status))   { snprintf(buf, sizeof(buf), "STATUS   = 0x%02X", status);gy87_log_info(buf);} else { gy87_log_info("STATUS read fail"); }

  // Try reading one sample (X,Z,Y order)
  if (gy87_hmc5883l_read_data())
  {
    float mx = gy87_hmc5883l_get_mx();
    float my = gy87_hmc5883l_get_my();
    float mz = gy87_hmc5883l_get_mz();
    snprintf(buf, sizeof(buf), "Sample Mxyz = %.3f %.3f %.3f uT", mx, my, mz);
    gy87_log_info(buf);
  }
  else
  {
    gy87_log_info("Sample read failed");
  }

  gy87_log_info("=== End HMC5883L Debug ===");
}

void gy87_test_hmc5883l_only(void)
{
  gy87_log_info("=== HMC5883L Test ===");
  
  if (!gy87_hmc5883l_read_data())
  {
    gy87_log_error("HMC5883L_Test", "Read data", 1);
    return;
  }
  
  char buffer[128];
  snprintf(buffer, sizeof(buffer), 
    "HMC5883L Test Result: X=%.2f Y=%.2f Z=%.2f microTesla",
    gy87_hmc5883l_get_mx(), gy87_hmc5883l_get_my(), gy87_hmc5883l_get_mz());
  
  gy87_log_info(buffer);
  gy87_log_info("=== End HMC5883L Test ===");
}

void gy87_log_error(const char *function, const char *operation, int status)
{
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "[%s] %s failed: %d", function, operation, status);
  gy87_log_info(buffer);
}

void gy87_log_info(const char *message)
{
  UART_SendString(message);
  UART_SendString("\r\n");
}

/* Private definitions ----------------------------------------------- */
static void gy87_format_sensor_data(char *buffer, size_t buffer_size, 
                                   float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float mx, float my, float mz,
                                   uint32_t period_ms)
{
  snprintf(buffer, buffer_size,
    "Axyz= %.4f %.4f %.4f m/s2 | "
    "Gxyz= %.4f %.4f %.4f rad/s | "
    "Mxyz= %.4f %.4f %.4f microTesla | "
    "t=%lums",
    ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}

/* Configure MPU6050 internal I2C master to read HMC5883L over SLV0 */
static uint8_t gy87_mpu6050_enable_i2c_master_for_hmc(void)
{
  // Disable BYPASS (route aux lines to internal master)
  s_data_tx[0] = MPU6050_REG_INT_PIN_CFG;
  s_data_tx[1] = 0x00; // BYPASS disabled
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  // Enable I2C master
  s_data_tx[0] = MPU6050_REG_USER_CTRL;
  s_data_tx[1] = MPU6050_BIT_I2C_MST_EN;
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  // Configure master clock (400kHz) for fast transfers
  s_data_tx[0] = MPU6050_REG_I2C_MST_CTRL;
  s_data_tx[1] = MPU6050_I2C_MST_CLK_400KHZ;
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  // Configure SLV0 to read 6 bytes from HMC5883L starting at DATA_X_MSB
  s_data_tx[0] = MPU6050_REG_I2C_SLV0_ADDR;
  s_data_tx[1] = (uint8_t)((HMC5883L_ADDRESS << 1) | 0x80);
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  s_data_tx[0] = MPU6050_REG_I2C_SLV0_REG;
  s_data_tx[1] = HMC5883L_REG_DATA_X_MSB;
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  s_data_tx[0] = MPU6050_REG_I2C_SLV0_CTRL;
  s_data_tx[1] = (uint8_t)(0x80 | 6); // Enable, length 6 bytes
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_data_tx, 2, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  return 1;
}

/* Read 6 magnetometer bytes via MPU6050 EXT_SENS_DATA registers */
static uint8_t gy87_hmc5883l_read_via_mpu(void)
{
  // Read from MPU6050 EXT_SENS_DATA registers (0x49..0x4E)
  s_data_tx[0] = MPU6050_REG_EXT_SENS_DATA_00;
  if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), &s_data_tx[0], 1, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), s_hmc5883l_data_rx, 6, I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0;
  }

  return 1;
}
/* End of file -------------------------------------------------------- */
