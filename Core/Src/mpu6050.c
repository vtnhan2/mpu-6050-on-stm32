#include "i2c.h"
#include "mpu6050.h"
#include <math.h>

uint8_t data_tx[2];
uint8_t data_rx[15];

// Global variables for angle calculation
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float prev_roll = 0.0f, prev_pitch = 0.0f, prev_yaw = 0.0f;

void MPU6050_Init(void)
{
	data_tx[0] = 0x6B;
	data_tx[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_ADDRESS, data_tx, 2, 100);

	data_tx[0] = 0x19;
	data_tx[1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_ADDRESS, data_tx, 2, 100);
}

uint8_t MPU6050_Read_Data(void)
{
	HAL_StatusTypeDef status;
	data_tx[0] = 0x3B;
	data_tx[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MPU6050_ADDRESS, &data_tx[0], 1, 100);
	status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MPU6050_ADDRESS, data_rx, 14, 100);
	return (status == HAL_OK) ? 1 : 0;
}

float MPU6050_Get_Ax(void)
{
	return (float)(((int16_t)(data_rx[0]<<8 | data_rx[1]))/(float)16384) * 9.81f; // Convert g to m/s²
}

float MPU6050_Get_Ay(void)
{
	return (float)(((int16_t)(data_rx[2]<<8 | data_rx[3]))/(float)16384) * 9.81f; // Convert g to m/s²
}

float MPU6050_Get_Az(void)
{
	return (float)(((int16_t)(data_rx[4]<<8 | data_rx[5]))/(float)16384) * 9.81f; // Convert g to m/s²
}

float MPU6050_Get_Gx(void)
{
	return (float)(((int16_t)(data_rx[10]<<8 | data_rx[11]))/(float)131) * (M_PI / 180.0f); // Convert deg/s to rad/s
}

float MPU6050_Get_Gy(void)
{
	return (float)(((int16_t)(data_rx[12]<<8 | data_rx[13]))/(float)131) * (M_PI / 180.0f); // Convert deg/s to rad/s
}

float MPU6050_Get_Gz(void)
{
	return (float)(((int16_t)(data_rx[8]<<8 | data_rx[9]))/(float)131) * (M_PI / 180.0f); // Convert deg/s to rad/s
}

float MPU6050_Get_Temperature(void)
{
	return (float)(((int16_t)(data_rx[6]<<8 | data_rx[7]))/(float)340 + (float)36.53);
}

void MPU6050_CalculateAngles(float* roll, float* pitch, float* yaw)
{
    float ax = MPU6050_Get_Ax();
    float ay = MPU6050_Get_Ay();
    float az = MPU6050_Get_Az();
    
    // Calculate roll and pitch from accelerometer (convert to degrees for display)
    *roll = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / M_PI;
    *pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
    
    // Yaw cannot be calculated from accelerometer alone
    // It requires gyroscope integration or magnetometer
    *yaw = 0.0f; // Placeholder
}

void MPU6050_ComplementaryFilter(float* roll, float* pitch, float* yaw, float dt)
{
    float ax = MPU6050_Get_Ax();
    float ay = MPU6050_Get_Ay();
    float az = MPU6050_Get_Az();
    float gx = MPU6050_Get_Gx(); // Now in rad/s
    float gy = MPU6050_Get_Gy(); // Now in rad/s
    float gz = MPU6050_Get_Gz(); // Now in rad/s
    
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
