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
// Config A: 8-average, 15 Hz default, normal measurement
#define HMC5883L_CFG_A_8AVG_15HZ    0x70
// Config B: Gain = 1.3 Ga (1090 LSB/Gauss)
#define HMC5883L_CFG_B_GAIN_1_3GA   0x20
// Mode: Continuous-Measurement Mode
#define HMC5883L_MODE_CONTINUOUS    0x00

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
	// Test if device is ready at given address
	// HAL_I2C_IsDeviceReady returns HAL_OK if device responds
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
        status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 1, 100);
        if (status == HAL_OK) {
            status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &status_reg, 1, 100);
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
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, 100);
	HAL_Delay(10);

	// Disable MPU6050 I2C master to release aux bus
	data_tx[0] = 0x6A;  // USER_CTRL
	data_tx[1] = 0x00;  // I2C_MST_EN=0
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, 100);
	HAL_Delay(5);

	// Enable I2C bypass for HMC5883L access (CRITICAL for GY-87 module)
	data_tx[0] = 0x37;  // INT_PIN_CFG register
	data_tx[1] = 0x02;  // BYPASS_EN (bit 1)
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, 100);
	HAL_Delay(100);  // Important delay after bypass enable

	// Set sample rate divider (100Hz: SMPLRT_DIV = 7)
	data_tx[0] = 0x19;
	data_tx[1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), data_tx, 2, 100);
	
	HAL_Delay(10);
}

void GY87_HMC5883L_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t device_id[3];
    char buffer[64];
    
    UART_SendString("Initializing HMC5883L...\r\n");
    
    // Verify HMC5883L device ID (should be "H43")
    data_tx[0] = HMC5883L_REG_ID_A;
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 1, 100);
    if (status == HAL_OK) {
        status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), device_id, 3, 100);
        if (status == HAL_OK && device_id[0] == 0x48 && device_id[1] == 0x34 && device_id[2] == 0x33) {
            UART_SendString("HMC5883L device ID verified: H43\r\n");
        } else {
            snprintf(buffer, sizeof(buffer), "HMC5883L device ID mismatch! Got: %02X %02X %02X\r\n", 
                    device_id[0], device_id[1], device_id[2]);
            UART_SendString(buffer);
            UART_SendString("Continuing with initialization...\r\n");
        }
    } else {
        UART_SendString("HMC5883L device ID read failed, continuing...\r\n");
    }
    
    // Configure HMC5883L: 8-average, 15Hz, normal measurement
    data_tx[0] = HMC5883L_REG_CONFIG_A;
    data_tx[1] = HMC5883L_CFG_A_8AVG_15HZ;  // 8 samples average, 15Hz, normal measurement
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, 100);
    if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Config A failed! Status: %d\r\n", status);
        UART_SendString(buffer);
        return;
    }
    HAL_Delay(10);

    // Gain = 1.3 Gauss (suitable for most environments)
    data_tx[0] = HMC5883L_REG_CONFIG_B;
    data_tx[1] = HMC5883L_CFG_B_GAIN_1_3GA;  // ±1.3 Gauss range
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, 100);
    if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Config B failed! Status: %d\r\n", status);
        UART_SendString(buffer);
        return;
    }
    HAL_Delay(10);

    // Set to idle mode first (recommended sequence)
    data_tx[0] = HMC5883L_REG_MODE;
    data_tx[1] = 0x02; // Idle mode
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, 100);
    if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Idle mode failed! Status: %d\r\n", status);
        UART_SendString(buffer);
        return;
    }
    HAL_Delay(10);

    // Kick a single measurement first (wake up the sensor)
    data_tx[0] = HMC5883L_REG_MODE;
    data_tx[1] = 0x01; // Single measurement
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, 100);
    if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Single measurement failed! Status: %d\r\n", status);
        UART_SendString(buffer);
        return;
    }
    HAL_Delay(50); // Wait for measurement to complete

    // Set to continuous measurement mode
    data_tx[0] = HMC5883L_REG_MODE;
    data_tx[1] = HMC5883L_MODE_CONTINUOUS; // Continuous measurement
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), data_tx, 2, 100);
    if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Continuous mode failed! Status: %d\r\n", status);
        UART_SendString(buffer);
        return;
    }
    HAL_Delay(100);  // Important delay after configuration
    
    UART_SendString("HMC5883L initialized successfully!\r\n");
    UART_SendString("Configuration: 8-avg, 15Hz, ±1.3Ga range, continuous mode\r\n");
}

void GY87_BMP180_Init(void)
{
	// BMP180 doesn't require special initialization
	// It's ready to use after power-on
	HAL_Delay(10);
}

void GY87_Init_All_Sensors(void)
{
	GY87_MPU6050_Init();
    GY87_HMC5883L_Init();
	GY87_BMP180_Init();
	
	UART_SendString("All GY87 sensors initialized successfully!\r\n");
}

// Data Reading Functions
uint8_t GY87_MPU6050_Read_Data(void)
{
	HAL_StatusTypeDef status;
	data_tx[0] = 0x3B;  // Starting register for accelerometer data
	
	// Send register address
	status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), &data_tx[0], 1, 100);
	if (status != HAL_OK) {
		char buffer[64];
		snprintf(buffer, sizeof(buffer), "MPU6050 TX Error: %d\r\n", status);
		UART_SendString(buffer);
		return 0;
	}
	
	// Read data
	status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(MPU6050_ADDRESS << 1), mpu6050_data_rx, 14, 100);
	if (status != HAL_OK) {
		char buffer[64];
		snprintf(buffer, sizeof(buffer), "MPU6050 RX Error: %d\r\n", status);
		UART_SendString(buffer);
		return 0;
	}
	
	return 1;
}

uint8_t GY87_HMC5883L_Read_Data(void)
{
	HAL_StatusTypeDef status;
	uint8_t status_reg;
	char buffer[64];
	
    // Wait for data ready (RDY bit) with timeout
    uint32_t start = HAL_GetTick();
    uint8_t data_ready = 0;
    
    do {
        data_tx[0] = HMC5883L_REG_STATUS;
        status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &data_tx[0], 1, 100);
        if (status != HAL_OK) {
            snprintf(buffer, sizeof(buffer), "HMC5883L Status TX Error: %d\r\n", status);
            UART_SendString(buffer);
            return 0;
        }
        
        status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &status_reg, 1, 100);
        if (status != HAL_OK) {
            snprintf(buffer, sizeof(buffer), "HMC5883L Status RX Error: %d\r\n", status);
            UART_SendString(buffer);
            return 0;
        }
        
        if (status_reg & 0x01) { // RDY bit set
            data_ready = 1;
            break;
        }
        
        // Check for lock bit (indicates data registers are being updated)
        if (status_reg & 0x02) {
            HAL_Delay(1); // Wait a bit if locked
        }
        
    } while ((HAL_GetTick() - start) < 20); // up to ~20ms timeout
    
    if (!data_ready) {
        UART_SendString("HMC5883L data not ready (timeout)\r\n");
        return 0;
    }
	
	// Read magnetometer data (X, Z, Y order as per HMC5883L spec)
    data_tx[0] = HMC5883L_REG_DATA_X_MSB;  // Starting register for magnetometer data (X MSB)
	
	// Send register address
    status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), &data_tx[0], 1, 100);
	if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Data TX Error: %d\r\n", status);
		UART_SendString(buffer);
		return 0;
	}
	
    // Read data (6 bytes: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB)
    status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(HMC5883L_ADDRESS << 1), hmc5883l_data_rx, 6, 100);
	if (status != HAL_OK) {
        snprintf(buffer, sizeof(buffer), "HMC5883L Data RX Error: %d\r\n", status);
		UART_SendString(buffer);
		return 0;
	}

    // Check for overflow (all bits set to 1)
    int16_t raw_x = (int16_t)(hmc5883l_data_rx[0]<<8 | hmc5883l_data_rx[1]);
    int16_t raw_y = (int16_t)(hmc5883l_data_rx[4]<<8 | hmc5883l_data_rx[5]);
    int16_t raw_z = (int16_t)(hmc5883l_data_rx[2]<<8 | hmc5883l_data_rx[3]);
    
    if (raw_x == -4096 || raw_y == -4096 || raw_z == -4096) {
        UART_SendString("HMC5883L sensor overflow detected!\r\n");
        return 0;
    }

    // Debug: Print raw data for troubleshooting
    char debug_buffer[128];
    snprintf(debug_buffer, sizeof(debug_buffer), 
        "HMC5883L Raw: %02X %02X %02X %02X %02X %02X\r\n",
        hmc5883l_data_rx[0], hmc5883l_data_rx[1], hmc5883l_data_rx[2],
        hmc5883l_data_rx[3], hmc5883l_data_rx[4], hmc5883l_data_rx[5]);
    UART_SendString(debug_buffer);
    
    // If all bytes are zero, print a hint for debugging
    if (hmc5883l_data_rx[0]==0 && hmc5883l_data_rx[1]==0 &&
        hmc5883l_data_rx[2]==0 && hmc5883l_data_rx[3]==0 &&
        hmc5883l_data_rx[4]==0 && hmc5883l_data_rx[5]==0) {
        UART_SendString("HMC5883L raw bytes all zero (check bypass/gain/mode)\r\n");
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
	
    // Get magnetometer data (Tesla)
    *mag_x = GY87_HMC5883L_Get_Mx();
    *mag_y = GY87_HMC5883L_Get_My();
    *mag_z = GY87_HMC5883L_Get_Mz();
	
	return 1;
}

// Individual Sensor Data Access (Legacy Support)
float GY87_MPU6050_Get_Ax(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[0]<<8 | mpu6050_data_rx[1]))/(float)16384) * 9.81f; // Convert g to m/s²
}

float GY87_MPU6050_Get_Ay(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[2]<<8 | mpu6050_data_rx[3]))/(float)16384) * 9.81f; // Convert g to m/s²
}

float GY87_MPU6050_Get_Az(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[4]<<8 | mpu6050_data_rx[5]))/(float)16384) * 9.81f; // Convert g to m/s²
}

float GY87_MPU6050_Get_Gx(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[8]<<8 | mpu6050_data_rx[9]))/(float)131) * (M_PI / 180.0f); // Convert deg/s to rad/s
}

float GY87_MPU6050_Get_Gy(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[10]<<8 | mpu6050_data_rx[11]))/(float)131) * (M_PI / 180.0f); // Convert deg/s to rad/s
}

float GY87_MPU6050_Get_Gz(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[12]<<8 | mpu6050_data_rx[13]))/(float)131) * (M_PI / 180.0f); // Convert deg/s to rad/s
}

float GY87_MPU6050_Get_Temperature(void)
{
	return (float)(((int16_t)(mpu6050_data_rx[6]<<8 | mpu6050_data_rx[7]))/(float)340 + (float)36.53);
}

// Magnetometer Data Access
float GY87_HMC5883L_Get_Mx(void)
{
    // HMC5883L output order: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
    int16_t raw_x = (int16_t)(hmc5883l_data_rx[0]<<8 | hmc5883l_data_rx[1]);
    // Gain 1.3 Gauss => 1090 LSB/Gauss (theo datasheet HMC5883L)
    float gauss = (float)raw_x / 1090.0f;
    return gauss * 0.0001f; // Convert Gauss to Tesla (1 Gauss = 0.0001 Tesla)
}

float GY87_HMC5883L_Get_My(void)
{
    int16_t raw_y = (int16_t)(hmc5883l_data_rx[4]<<8 | hmc5883l_data_rx[5]);
    float gauss = (float)raw_y / 1090.0f;
    return gauss * 0.0001f; // Convert Gauss to Tesla
}

float GY87_HMC5883L_Get_Mz(void)
{
    int16_t raw_z = (int16_t)(hmc5883l_data_rx[2]<<8 | hmc5883l_data_rx[3]);
    float gauss = (float)raw_z / 1090.0f;
    return gauss * 0.0001f; // Convert Gauss to Tesla
}

void GY87_MPU6050_CalculateAngles(float* roll, float* pitch, float* yaw)
{
    float ax = GY87_MPU6050_Get_Ax();
    float ay = GY87_MPU6050_Get_Ay();
    float az = GY87_MPU6050_Get_Az();
    
    // Calculate roll and pitch from accelerometer (convert to degrees for display)
    *roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / M_PI;
    *pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
    
    // Yaw cannot be calculated from accelerometer alone
    // It requires gyroscope integration or magnetometer
    *yaw = 0.0f; // Placeholder
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
    
    // Complementary filter (alpha = 0.98 for gyro, 0.02 for acc)
    // Convert gyro from rad/s to deg/s for integration
    float alpha = 0.98f;
    
    *roll = alpha * (prev_roll + gx * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_roll;
    *pitch = alpha * (prev_pitch + gy * dt * 180.0f / M_PI) + (1.0f - alpha) * acc_pitch;
    *yaw = prev_yaw + gz * dt * 180.0f / M_PI; // Yaw from gyro only, convert to degrees
    
    // Update previous values
    prev_roll = *roll;
    prev_pitch = *pitch;
    prev_yaw = *yaw;
}

// UART Display Functions
void GY87_Display_MPU6050_Individual(uint32_t period_ms)
{
    if (!GY87_MPU6050_Read_Data()) {
        UART_SendString("Error reading MPU6050 data!\r\n");
        return;
    }
    
    float ax = GY87_MPU6050_Get_Ax();
    float ay = GY87_MPU6050_Get_Ay();
    float az = GY87_MPU6050_Get_Az();
    float temp = GY87_MPU6050_Get_Temperature();
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
        "MPU6050: Ax=%.3f Ay=%.3f Az=%.3f m/s² | Temp=%.1f°C | t=%lums\r\n",
        ax, ay, az, temp, period_ms);
    UART_SendString(buffer);
}

void GY87_Display_AGM(float ax, float ay, float az, 
                      float gx, float gy, float gz, 
                      float mx, float my, float mz, 
                      uint32_t period_ms)
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
        "Axyz= %.3f %.3f %.3f m/s² | Gxyz= %.3f %.3f %.3f rad/s | Mxyz= %.3f %.3f %.3f Tesla | t=%lums\r\n",
        ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    UART_SendString(buffer);
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
        UART_SendString("\r\n=== GY87 Sensor Data ===\r\n");
        UART_SendString("Accelerometer (m/s²) | Gyroscope (rad/s) | Magnetometer (Tesla)\r\n");
        UART_SendString("X        Y        Z   | X        Y        Z   | X        Y        Z\r\n");
        UART_SendString("----------------------------------------\r\n");
    }
    display_count++;
    
    // Display data in table format
    snprintf(buffer, sizeof(buffer), 
        "%7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f | %7.6f %7.6f %7.6f | %lums\r\n",
        ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    UART_SendString(buffer);
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
            
            // Debug: Print calculated values
            char debug_buffer[128];
            snprintf(debug_buffer, sizeof(debug_buffer), 
                "HMC5883L Calculated: X=%.6f Y=%.6f Z=%.6f Tesla\r\n", mx, my, mz);
            UART_SendString(debug_buffer);
        } else {
            // HMC5883L not available - set magnetometer to zero
            mx = 0.0f;
            my = 0.0f;
            mz = 0.0f;
            
            UART_SendString("Warning: HMC5883L read failed, Mag=0\r\n");
        }
        
        // Display AGM format with available data
        GY87_Display_AGM(ax, ay, az, gx, gy, gz, mx, my, mz, period_ms);
    } else {
        UART_SendString("Error reading MPU6050 data!\r\n");
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
        UART_SendString("Error reading MPU6050 data!\r\n");
    }
}

