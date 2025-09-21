#include "i2c.h"
#include "gy87_mpu6050.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// I2C Communication Buffers
uint8_t data_tx[8];
uint8_t mpu6050_data_rx[14];    // MPU6050 raw data
uint8_t hmc5883l_data_rx[6];    // HMC5883L raw data

// Global variables for angle calculation
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float prev_roll = 0.0f, prev_pitch = 0.0f, prev_yaw = 0.0f;


// HMC5883L Register Definitions
#define HMC5883L_REG_CONFIG_A       0x00
#define HMC5883L_REG_CONFIG_B       0x01
#define HMC5883L_REG_MODE           0x02
#define HMC5883L_REG_DATA_X_MSB     0x03
#define HMC5883L_REG_DATA_X_LSB     0x04
#define HMC5883L_REG_DATA_Z_MSB     0x05
#define HMC5883L_REG_DATA_Z_LSB     0x06
#define HMC5883L_REG_DATA_Y_MSB     0x07
#define HMC5883L_REG_DATA_Y_LSB     0x08
#define HMC5883L_REG_STATUS         0x09
#define HMC5883L_REG_ID_A           0x0A
#define HMC5883L_REG_ID_B           0x0B
#define HMC5883L_REG_ID_C           0x0C

// HMC5883L Configuration Values
#define HMC5883L_CFG_A_8AVG_15HZ    0x70  // 8-average, 15 Hz, normal measurement
#define HMC5883L_CFG_B_GAIN_1_3GA   0x20  // Gain = 1.3 Ga (1090 LSB/Gauss)
#define HMC5883L_MODE_IDLE          0x02  // Idle mode
#define HMC5883L_MODE_SINGLE        0x01  // Single measurement mode
#define HMC5883L_MODE_CONTINUOUS    0x00  // Continuous measurement mode

// HMC5883L Constants
#define HMC5883L_GAIN_LSB_PER_GAUSS 1090.0f  // LSB per Gauss for 1.3Ga gain
#define HMC5883L_GAUSS_TO_MICROTESLA 0.1f    // Conversion factor
#define HMC5883L_OVERFLOW_VALUE     -4096    // Overflow detection value
#define HMC5883L_DEVICE_ID_H        0x48     // Expected device ID bytes
#define HMC5883L_DEVICE_ID_4        0x34
#define HMC5883L_DEVICE_ID_3        0x33

// MPU6050 Constants
#define MPU6050_ACCEL_SCALE         16384.0f  // LSB per g for ±2g range
#define MPU6050_GYRO_SCALE          131.0f    // LSB per deg/s for ±250deg/s range
#define MPU6050_TEMP_SCALE          340.0f    // Temperature scale factor
#define MPU6050_TEMP_OFFSET         36.53f    // Temperature offset
#define MPU6050_G_TO_MS2            9.81f     // Gravity to m/s² conversion
#define MPU6050_DEG_TO_RAD          (M_PI / 180.0f)  // Degrees to radians

// Timing Constants
#define I2C_TIMEOUT_MS              100
#define HMC5883L_INIT_DELAY_MS      10
#define HMC5883L_MEASURE_DELAY_MS   50
#define HMC5883L_READ_DELAY_MS      5

// Helper Functions
void GY87_Log_Error(const char* function, const char* operation, int status)
{
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "[%s] %s failed: %d", function, operation, status);
    GY87_Log_Info(buffer);
}

void GY87_Log_Info(const char* message)
{
    UART_SendString(message);
    UART_SendString("\r\n");
}

// Buffer management helper
static void GY87_Format_Sensor_Data(char* buffer, size_t buffer_size, 
                                   float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float mx, float my, float mz,
                                   uint32_t period_ms)
{
    snprintf(buffer, buffer_size, 
        "Axyz= %.3f %.3f %.3f m/s² | Gxyz= %.3f %.3f %.3f rad/s | Mxyz= %.4f %.4f %.4f microTesla | t=%lums",
        ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
}

// I2C Scanner Functions
void GY87_I2C_Scanner(void)
{
	UART_SendString("\r\n=== I2C Scanner Started ===\r\n");
	UART_SendString("Scanning I2C bus for devices...\r\n");
	
	uint8_t devices_found = 0;
	char buffer[64];
	
	for(uint8_t address = I2C_SCAN_START_ADDR; address <= I2C_SCAN_END_ADDR; address++)
	{
		if(GY87_I2C_IsDeviceReady(address))
		{
			snprintf(buffer, sizeof(buffer), "Device found at address: 0x%02X (%d)\r\n", address, address);
			UART_SendString(buffer);
			devices_found++;
		}
	}
	
	if(devices_found == 0)
	{
		UART_SendString("No I2C devices found!\r\n");
	}
	else
	{
		snprintf(buffer, sizeof(buffer), "Total devices found: %d\r\n", devices_found);
		UART_SendString(buffer);
	}
	
	UART_SendString("=== I2C Scanner Completed ===\r\n\r\n");
}

uint8_t GY87_I2C_IsDeviceReady(uint8_t address)
{
	HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, I2C_SCAN_TIMEOUT);
	return (result == HAL_OK) ? 1 : 0;
}

void GY87_Debug_I2C_Status(void)
{
	UART_SendString("\r\n=== I2C Debug Status ===\r\n");
	
	char buffer[128];
	
	// Check MPU6050
	uint8_t mpu_ready = GY87_I2C_IsDeviceReady(MPU6050_ADDRESS);
	snprintf(buffer, sizeof(buffer), "MPU6050 (0x%02X): %s\r\n", 
		MPU6050_ADDRESS, mpu_ready ? "READY" : "NOT READY");
	UART_SendString(buffer);
	
    // Check HMC5883L with detailed status
    uint8_t hmc_ready = GY87_I2C_IsDeviceReady(HMC5883L_ADDRESS);
    snprintf(buffer, sizeof(buffer), "HMC5883L (0x%02X): %s", 
        HMC5883L_ADDRESS, hmc_ready ? "READY" : "NOT READY");
    UART_SendString(buffer);
    
    if (hmc_ready) {
        // Read status register for more details
        HAL_StatusTypeDef status;
        uint8_t status_reg;
        data_tx[0] = HMC5883L_REG_STATUS;
        status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 1, I2C_TIMEOUT_MS);
        if (status == HAL_OK) {
            status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &status_reg, 1, I2C_TIMEOUT_MS);
            if (status == HAL_OK) {
                snprintf(buffer, sizeof(buffer), " [RDY:%d LOCK:%d]\r\n", 
                    (status_reg & 0x01) ? 1 : 0, (status_reg & 0x02) ? 1 : 0);
                UART_SendString(buffer);
            } else {
                UART_SendString(" [Status read failed]\r\n");
            }
        } else {
            UART_SendString(" [Status read failed]\r\n");
        }
    } else {
        UART_SendString("\r\n");
    }
	
	// Check BMP180
	uint8_t bmp_ready = GY87_I2C_IsDeviceReady(BMP180_ADDRESS);
	snprintf(buffer, sizeof(buffer), "BMP180 (0x%02X): %s\r\n", 
		BMP180_ADDRESS, bmp_ready ? "READY" : "NOT READY");
	UART_SendString(buffer);
	
	UART_SendString("========================\r\n\r\n");
}

// Sensor Initialization Functions
void GY87_MPU6050_Init(void)
{
	// Wake up MPU6050 (disable sleep mode)
	data_tx[0] = 0x6B;
	data_tx[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
	HAL_Delay(HMC5883L_INIT_DELAY_MS);

	// Disable MPU6050 I2C master to release aux bus
	data_tx[0] = 0x6A;  // USER_CTRL
	data_tx[1] = 0x00;  // I2C_MST_EN=0
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
	HAL_Delay(5);

	// Enable I2C bypass for HMC5883L access (CRITICAL for GY-87 module)
	data_tx[0] = 0x37;  // INT_PIN_CFG register
	data_tx[1] = 0x02;  // BYPASS_EN (bit 1)
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
	HAL_Delay(100);  // Important delay after bypass enable

	// Set sample rate divider (100Hz: SMPLRT_DIV = 7)
	data_tx[0] = 0x19;
	data_tx[1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
	
	HAL_Delay(HMC5883L_INIT_DELAY_MS);
}

void GY87_HMC5883L_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t device_id[3];
    char buffer[64];
    
    GY87_Log_Info("Initializing HMC5883L...");
    
    // Verify HMC5883L device ID (should be "H43")
    data_tx[0] = HMC5883L_REG_ID_A;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 1, I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), device_id, 3, I2C_TIMEOUT_MS);
        if (status == HAL_OK && device_id[0] == HMC5883L_DEVICE_ID_H && 
            device_id[1] == HMC5883L_DEVICE_ID_4 && device_id[2] == HMC5883L_DEVICE_ID_3) {
            GY87_Log_Info("HMC5883L device ID verified: H43");
        } else {
            snprintf(buffer, sizeof(buffer), "HMC5883L device ID mismatch! Got: %02X %02X %02X", 
                    device_id[0], device_id[1], device_id[2]);
            GY87_Log_Info(buffer);
            GY87_Log_Info("Continuing with initialization...");
        }
    } else {
        GY87_Log_Info("HMC5883L device ID read failed, continuing...");
    }
    
    // Configure HMC5883L: 8-average, 15Hz, normal measurement
    data_tx[0] = HMC5883L_REG_CONFIG_A;
    data_tx[1] = HMC5883L_CFG_A_8AVG_15HZ;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Init", "Config A", (int)status);
        return;
    }
    HAL_Delay(HMC5883L_INIT_DELAY_MS);

    // Gain = 1.3 Gauss (suitable for most environments)
    data_tx[0] = HMC5883L_REG_CONFIG_B;
    data_tx[1] = HMC5883L_CFG_B_GAIN_1_3GA;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Init", "Config B", (int)status);
        return;
    }
    HAL_Delay(HMC5883L_INIT_DELAY_MS);

    // Set to idle mode first (recommended sequence)
    data_tx[0] = HMC5883L_REG_MODE;
    data_tx[1] = HMC5883L_MODE_IDLE;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Init", "Idle mode", (int)status);
        return;
    }
    HAL_Delay(HMC5883L_INIT_DELAY_MS);

    // Kick a single measurement first (wake up the sensor)
    data_tx[0] = HMC5883L_REG_MODE;
    data_tx[1] = HMC5883L_MODE_SINGLE;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Init", "Single measurement", (int)status);
        return;
    }
    HAL_Delay(HMC5883L_MEASURE_DELAY_MS);

    // Set to continuous measurement mode
    data_tx[0] = HMC5883L_REG_MODE;
    data_tx[1] = HMC5883L_MODE_CONTINUOUS;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Init", "Continuous mode", (int)status);
        return;
    }
    HAL_Delay(100);  // Important delay after configuration
    
    GY87_Log_Info("HMC5883L initialized successfully!");
    GY87_Log_Info("Configuration: 8-avg, 15Hz, ±1.3Ga range, continuous mode");
}

void GY87_BMP180_Init(void)
{
	// BMP180 doesn't require special initialization
	// It's ready to use after power-on
	HAL_Delay(HMC5883L_INIT_DELAY_MS);
}

void GY87_Init_All_Sensors(void)
{
	GY87_MPU6050_Init();
    GY87_HMC5883L_Init();
	GY87_BMP180_Init();
	
	GY87_Log_Info("All GY87 sensors initialized successfully!");
}

// Data Reading Functions
uint8_t GY87_MPU6050_Read_Data(void)
{
	HAL_StatusTypeDef status;
	data_tx[0] = 0x3B;  // Starting register for accelerometer data
	
	// Send register address
	status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), &data_tx[0], 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		GY87_Log_Error("MPU6050_Read", "TX", (int)status);
		return 0;
	}
	
	// Read data
	status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), mpu6050_data_rx, 14, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
		GY87_Log_Error("MPU6050_Read", "RX", (int)status);
		return 0;
	}
	
	return 1;
}

uint8_t GY87_HMC5883L_Read_Data(void)
{
	HAL_StatusTypeDef status;
	static uint32_t last_read_time = 0;
	uint32_t current_time = HAL_GetTick();
	
    // Rate limiting disabled for continuous fresh data
    last_read_time = current_time;
    
    // Small delay to ensure fresh data in continuous mode
    HAL_Delay(HMC5883L_READ_DELAY_MS);
	
	// Read magnetometer data (X, Z, Y order as per HMC5883L spec)
    data_tx[0] = HMC5883L_REG_DATA_X_MSB;
	
	// Send register address
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &data_tx[0], 1, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Read", "Data TX", (int)status);
		return 0;
	}
	
    // Read 6 bytes: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
    status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), hmc5883l_data_rx, 6, I2C_TIMEOUT_MS);
	if (status != HAL_OK) {
        GY87_Log_Error("HMC5883L_Read", "Data RX", (int)status);
		return 0;
	}

    // Check for overflow (all bits set to 1)
    int16_t raw_x = (int16_t)(hmc5883l_data_rx[0]<<8 | hmc5883l_data_rx[1]);
    int16_t raw_y = (int16_t)(hmc5883l_data_rx[4]<<8 | hmc5883l_data_rx[5]);
    int16_t raw_z = (int16_t)(hmc5883l_data_rx[2]<<8 | hmc5883l_data_rx[3]);
    
    if (raw_x == HMC5883L_OVERFLOW_VALUE || raw_y == HMC5883L_OVERFLOW_VALUE || raw_z == HMC5883L_OVERFLOW_VALUE) {
        GY87_Log_Info("HMC5883L sensor overflow detected!");
        return 0;
    }

    // Debug output for first few readings
    static uint32_t debug_count = 0;
    if (debug_count < 3) {
        char debug_buffer[128];
        snprintf(debug_buffer, sizeof(debug_buffer), 
            "HMC5883L Raw: %02X %02X %02X %02X %02X %02X (X=%d Y=%d Z=%d)",
            hmc5883l_data_rx[0], hmc5883l_data_rx[1], hmc5883l_data_rx[2],
            hmc5883l_data_rx[3], hmc5883l_data_rx[4], hmc5883l_data_rx[5],
            raw_x, raw_y, raw_z);
        GY87_Log_Info(debug_buffer);
        debug_count++;
    }
    
    // Check for all-zero data (configuration issue)
    if (hmc5883l_data_rx[0]==0 && hmc5883l_data_rx[1]==0 &&
        hmc5883l_data_rx[2]==0 && hmc5883l_data_rx[3]==0 &&
        hmc5883l_data_rx[4]==0 && hmc5883l_data_rx[5]==0) {
        GY87_Log_Info("HMC5883L raw bytes all zero (check bypass/gain/mode)");
        return 0;
    }
	
	return 1;
}

uint8_t GY87_Read_All_Sensors(float* accel_x, float* accel_y, float* accel_z,
                              float* gyro_x, float* gyro_y, float* gyro_z,
                              float* mag_x, float* mag_y, float* mag_z)
{
	// Validate input parameters
	if (accel_x == NULL || accel_y == NULL || accel_z == NULL ||
	    gyro_x == NULL || gyro_y == NULL || gyro_z == NULL ||
	    mag_x == NULL || mag_y == NULL || mag_z == NULL) {
		return 0;
	}
	
	// Read MPU6050 data
	if (!GY87_MPU6050_Read_Data()) return 0;
	
    // Read HMC5883L data
    if (!GY87_HMC5883L_Read_Data()) return 0;
	
	// Get accelerometer data (m/s²)
	*accel_x = GY87_MPU6050_Get_Ax();
	*accel_y = GY87_MPU6050_Get_Ay();
	*accel_z = GY87_MPU6050_Get_Az();
	
	// Get gyroscope data (rad/s)
	*gyro_x = GY87_MPU6050_Get_Gx();
	*gyro_y = GY87_MPU6050_Get_Gy();
	*gyro_z = GY87_MPU6050_Get_Gz();
	
    // Get magnetometer data (microTesla)
    *mag_x = GY87_HMC5883L_Get_Mx();
    *mag_y = GY87_HMC5883L_Get_My();
    *mag_z = GY87_HMC5883L_Get_Mz();
	
	return 1;
}

// Individual Sensor Data Access
float GY87_MPU6050_Get_Ax(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[0]<<8 | mpu6050_data_rx[1]);
	return (float)raw / MPU6050_ACCEL_SCALE * MPU6050_G_TO_MS2;
}

float GY87_MPU6050_Get_Ay(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[2]<<8 | mpu6050_data_rx[3]);
	return (float)raw / MPU6050_ACCEL_SCALE * MPU6050_G_TO_MS2;
}

float GY87_MPU6050_Get_Az(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[4]<<8 | mpu6050_data_rx[5]);
	return (float)raw / MPU6050_ACCEL_SCALE * MPU6050_G_TO_MS2;
}

float GY87_MPU6050_Get_Gx(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[8]<<8 | mpu6050_data_rx[9]);
	return (float)raw / MPU6050_GYRO_SCALE * MPU6050_DEG_TO_RAD;
}

float GY87_MPU6050_Get_Gy(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[10]<<8 | mpu6050_data_rx[11]);
	return (float)raw / MPU6050_GYRO_SCALE * MPU6050_DEG_TO_RAD;
}

float GY87_MPU6050_Get_Gz(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[12]<<8 | mpu6050_data_rx[13]);
	return (float)raw / MPU6050_GYRO_SCALE * MPU6050_DEG_TO_RAD;
}

float GY87_MPU6050_Get_Temperature(void)
{
	int16_t raw = (int16_t)(mpu6050_data_rx[6]<<8 | mpu6050_data_rx[7]);
	return (float)raw / MPU6050_TEMP_SCALE + MPU6050_TEMP_OFFSET;
}

// Magnetometer Data Access
float GY87_HMC5883L_Get_Mx(void)
{
    int16_t raw_x = (int16_t)(hmc5883l_data_rx[0]<<8 | hmc5883l_data_rx[1]);
    float gauss = (float)raw_x / HMC5883L_GAIN_LSB_PER_GAUSS;
    return gauss * HMC5883L_GAUSS_TO_MICROTESLA;
}

float GY87_HMC5883L_Get_My(void)
{
    int16_t raw_y = (int16_t)(hmc5883l_data_rx[4]<<8 | hmc5883l_data_rx[5]);
    float gauss = (float)raw_y / HMC5883L_GAIN_LSB_PER_GAUSS;
    return gauss * HMC5883L_GAUSS_TO_MICROTESLA;
}

float GY87_HMC5883L_Get_Mz(void)
{
    int16_t raw_z = (int16_t)(hmc5883l_data_rx[2]<<8 | hmc5883l_data_rx[3]);
    float gauss = (float)raw_z / HMC5883L_GAIN_LSB_PER_GAUSS;
    return gauss * HMC5883L_GAUSS_TO_MICROTESLA;
}

void GY87_MPU6050_CalculateAngles(float* roll, float* pitch, float* yaw)
{
    float ax = GY87_MPU6050_Get_Ax();
    float ay = GY87_MPU6050_Get_Ay();
    float az = GY87_MPU6050_Get_Az();
    
    // Calculate roll and pitch from accelerometer (convert to degrees for display)
    float rad_to_deg = 180.0f / M_PI;
    *roll = atan2(ay, sqrt(ax*ax + az*az)) * rad_to_deg;
    *pitch = atan2(-ax, sqrt(ay*ay + az*az)) * rad_to_deg;
    
    // Yaw requires gyroscope integration or magnetometer
    *yaw = 0.0f;
}

void GY87_MPU6050_ComplementaryFilter(float* roll, float* pitch, float* yaw, float dt)
{
    float ax = GY87_MPU6050_Get_Ax();
    float ay = GY87_MPU6050_Get_Ay();
    float az = GY87_MPU6050_Get_Az();
    float gx = GY87_MPU6050_Get_Gx(); // Now in rad/s
    float gy = GY87_MPU6050_Get_Gy(); // Now in rad/s
    float gz = GY87_MPU6050_Get_Gz(); // Now in rad/s
    
    // Calculate angles from accelerometer (convert to degrees for display)
    float acc_roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / M_PI;
    float acc_pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
    
    // Complementary filter (98% gyro, 2% accelerometer)
    float alpha = 0.98f;
    float rad_to_deg = 180.0f / M_PI;
    
    *roll = alpha * (prev_roll + gx * dt * rad_to_deg) + (1.0f - alpha) * acc_roll;
    *pitch = alpha * (prev_pitch + gy * dt * rad_to_deg) + (1.0f - alpha) * acc_pitch;
    *yaw = prev_yaw + gz * dt * rad_to_deg;
    
    // Update previous values
    prev_roll = *roll;
    prev_pitch = *pitch;
    prev_yaw = *yaw;
}

// UART Display Functions
void GY87_Display_MPU6050_Individual(uint32_t period_ms)
{
    if (!GY87_MPU6050_Read_Data()) {
        GY87_Log_Info("Error reading MPU6050 data!");
        return;
    }
    
    float ax = GY87_MPU6050_Get_Ax();
    float ay = GY87_MPU6050_Get_Ay();
    float az = GY87_MPU6050_Get_Az();
    float temp = GY87_MPU6050_Get_Temperature();
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
        "MPU6050: Ax=%.3f Ay=%.3f Az=%.3f m/s² | Temp=%.1f°C | t=%lums",
        ax, ay, az, temp, period_ms);
    GY87_Log_Info(buffer);
}

void GY87_Display_AGM(float ax, float ay, float az, 
                      float gx, float gy, float gz, 
                      float mx, float my, float mz, 
                      uint32_t period_ms)
{
    char buffer[256];
    GY87_Format_Sensor_Data(buffer, sizeof(buffer), ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    GY87_Log_Info(buffer);
}

void GY87_Display_Formatted_Data(float ax, float ay, float az, 
                                 float gx, float gy, float gz, 
                                 float mx, float my, float mz, 
                                 uint32_t period_ms)
{
    char buffer[512];
    
    // Display header every 50 readings (about 0.5 seconds at 100Hz)
    static uint32_t display_count = 0;
    if (display_count % 50 == 0) {
        GY87_Log_Info("\r\n=== GY87 Sensor Data ===");
        GY87_Log_Info("Accelerometer (m/s²) | Gyroscope (rad/s) | Magnetometer (microTesla)");
        GY87_Log_Info("X        Y        Z   | X        Y        Z   | X        Y        Z");
        GY87_Log_Info("----------------------------------------");
    }
    display_count++;
    
    // Display data in table format
    snprintf(buffer, sizeof(buffer), 
        "%7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f | %7.2f %7.2f %7.2f | %lums",
        ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    GY87_Log_Info(buffer);
}

void GY87_Display_All_Sensors_AGM(uint32_t period_ms)
{
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    // Try to read MPU6050 first
    if (GY87_MPU6050_Read_Data()) {
        // Get MPU6050 data
        ax = GY87_MPU6050_Get_Ax();
        ay = GY87_MPU6050_Get_Ay();
        az = GY87_MPU6050_Get_Az();
        gx = GY87_MPU6050_Get_Gx();
        gy = GY87_MPU6050_Get_Gy();
        gz = GY87_MPU6050_Get_Gz();
        
        // Try to read HMC5883L (magnetometer)
        if (GY87_HMC5883L_Read_Data()) {
            // Get magnetometer data if available
            mx = GY87_HMC5883L_Get_Mx();
            my = GY87_HMC5883L_Get_My();
            mz = GY87_HMC5883L_Get_Mz();
            
            // Debug: Show actual magnetometer values
            static uint32_t mag_debug_count = 0;
            if (mag_debug_count % 20 == 0) { // Every 20 readings
                char debug_buffer[128];
                snprintf(debug_buffer, sizeof(debug_buffer), 
                    "Mag Debug: X=%.2f Y=%.2f Z=%.2f microTesla", mx, my, mz);
                GY87_Log_Info(debug_buffer);
            }
            mag_debug_count++;
        } else {
            // HMC5883L not available - set magnetometer to zero
            mx = 0.0f;
            my = 0.0f;
            mz = 0.0f;
            
            // Only show warning occasionally to reduce spam
            static uint32_t warning_count = 0;
            if (warning_count % 100 == 0) {
                GY87_Log_Info("Warning: HMC5883L read failed, Mag=0");
            }
            warning_count++;
        }
        
        // Display AGM format with available data
        GY87_Display_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    } else {
        GY87_Log_Info("Error reading MPU6050 data!");
    }
}

void GY87_Display_MPU6050_Only_AGM(uint32_t period_ms)
{
    // Read only MPU6050 and display in AGM format with Mag=0
    if (GY87_MPU6050_Read_Data()) {
        float ax = GY87_MPU6050_Get_Ax();
        float ay = GY87_MPU6050_Get_Ay();
        float az = GY87_MPU6050_Get_Az();
        float gx = GY87_MPU6050_Get_Gx();
        float gy = GY87_MPU6050_Get_Gy();
        float gz = GY87_MPU6050_Get_Gz();
        
        // Set magnetometer to zero (not available)
        float mx = 0.0f, my = 0.0f, mz = 0.0f;
        
        GY87_Display_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    } else {
        GY87_Log_Info("Error reading MPU6050 data!");
    }
}

void GY87_Test_HMC5883L_Only(void)
{
    GY87_Log_Info("\r\n=== HMC5883L Test ===");
    
    // Test reading HMC5883L data with simplified approach
    HAL_StatusTypeDef status;
    char buffer[128];
    
    // Wait a bit for sensor to be ready
    HAL_Delay(100);
    
    // Try to read data directly without checking RDY bit
    data_tx[0] = HMC5883L_REG_DATA_X_MSB;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 1, I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), hmc5883l_data_rx, 6, I2C_TIMEOUT_MS);
        if (status == HAL_OK) {
            // Print raw data
            snprintf(buffer, sizeof(buffer), 
                "HMC5883L Raw Data: %02X %02X %02X %02X %02X %02X",
                hmc5883l_data_rx[0], hmc5883l_data_rx[1], hmc5883l_data_rx[2],
                hmc5883l_data_rx[3], hmc5883l_data_rx[4], hmc5883l_data_rx[5]);
            GY87_Log_Info(buffer);
            
            // Calculate and display values
            int16_t raw_x = (int16_t)(hmc5883l_data_rx[0]<<8 | hmc5883l_data_rx[1]);
            int16_t raw_y = (int16_t)(hmc5883l_data_rx[4]<<8 | hmc5883l_data_rx[5]);
            int16_t raw_z = (int16_t)(hmc5883l_data_rx[2]<<8 | hmc5883l_data_rx[3]);
            
            snprintf(buffer, sizeof(buffer), 
                "HMC5883L Raw Values: X=%d Y=%d Z=%d", raw_x, raw_y, raw_z);
            GY87_Log_Info(buffer);
            
            // Convert to microTesla
            float mx = (float)raw_x / HMC5883L_GAIN_LSB_PER_GAUSS * HMC5883L_GAUSS_TO_MICROTESLA;
            float my = (float)raw_y / HMC5883L_GAIN_LSB_PER_GAUSS * HMC5883L_GAUSS_TO_MICROTESLA;
            float mz = (float)raw_z / HMC5883L_GAIN_LSB_PER_GAUSS * HMC5883L_GAUSS_TO_MICROTESLA;
            
            snprintf(buffer, sizeof(buffer), 
                "HMC5883L Test Result: X=%.2f Y=%.2f Z=%.2f microTesla", mx, my, mz);
            GY87_Log_Info(buffer);
        } else {
            GY87_Log_Error("HMC5883L_Test", "RX", (int)status);
        }
    } else {
        GY87_Log_Error("HMC5883L_Test", "TX", (int)status);
    }
    
    GY87_Log_Info("=== End HMC5883L Test ===\r\n");
}

