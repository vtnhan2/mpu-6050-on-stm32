#ifndef INC_GY87_MPU6050_H_
#define INC_GY87_MPU6050_H_

#include <stdint.h>

// I2C Scanner definitions
#define I2C_SCAN_START_ADDR 0x08
#define I2C_SCAN_END_ADDR   0x77
#define I2C_SCAN_TIMEOUT    100

// Device I2C Addresses
#define MPU6050_ADDRESS     0x68    // MPU6050 (AD0=GND), 0x69 (AD0=VCC)
#define HMC5883L_ADDRESS    0x1E    // HMC5883L Magnetometer
#define BMP180_ADDRESS      0x77    // BMP180 Pressure Sensor


// I2C Scanner function
void GY87_I2C_Scanner(void);
uint8_t GY87_I2C_IsDeviceReady(uint8_t address);
void GY87_Debug_I2C_Status(void);

// Sensor Initialization Functions
void GY87_MPU6050_Init(void);
void GY87_HMC5883L_Init(void);
void GY87_BMP180_Init(void);
void GY87_Init_All_Sensors(void);

// Data Reading Functions
uint8_t GY87_MPU6050_Read_Data(void);
uint8_t GY87_HMC5883L_Read_Data(void);
uint8_t GY87_Read_All_Sensors(float* accel_x, float* accel_y, float* accel_z,
                              float* gyro_x, float* gyro_y, float* gyro_z,
                              float* mag_x, float* mag_y, float* mag_z);

// Individual Sensor Data Access (Legacy Support)
float GY87_MPU6050_Get_Ax(void);
float GY87_MPU6050_Get_Ay(void);
float GY87_MPU6050_Get_Az(void);
float GY87_MPU6050_Get_Gx(void);
float GY87_MPU6050_Get_Gy(void);
float GY87_MPU6050_Get_Gz(void);
float GY87_MPU6050_Get_Temperature(void);

// Magnetometer Data Access
float GY87_HMC5883L_Get_Mx(void);
float GY87_HMC5883L_Get_My(void);
float GY87_HMC5883L_Get_Mz(void);


// Angle calculation functions
void GY87_MPU6050_CalculateAngles(float* roll, float* pitch, float* yaw);
void GY87_MPU6050_ComplementaryFilter(float* roll, float* pitch, float* yaw, float dt);

// UART Display Functions
void GY87_Display_MPU6050_Individual(uint32_t period_ms);
void GY87_Display_AGM(float ax, float ay, float az, 
                      float gx, float gy, float gz, 
                      float mx, float my, float mz, 
                      uint32_t period_ms);
void GY87_Display_Formatted_Data(float ax, float ay, float az, 
                                 float gx, float gy, float gz, 
                                 float mx, float my, float mz, 
                                 uint32_t period_ms);
void GY87_Display_All_Sensors_AGM(uint32_t period_ms);
void GY87_Display_MPU6050_Only_AGM(uint32_t period_ms);

// Global variables for angle calculation
extern float roll, pitch, yaw;
extern float prev_roll, prev_pitch, prev_yaw;


#endif
