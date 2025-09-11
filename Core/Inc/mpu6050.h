#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>

#define MPU6050_ADDRESS 0xD0

// Function prototypes
void MPU6050_Init(void);
uint8_t MPU6050_Read_Data(void);
float MPU6050_Get_Ax(void);
float MPU6050_Get_Ay(void);
float MPU6050_Get_Az(void);
float MPU6050_Get_Gx(void);
float MPU6050_Get_Gy(void);
float MPU6050_Get_Gz(void);
float MPU6050_Get_Temperature(void);

// Angle calculation functions
void MPU6050_CalculateAngles(float* roll, float* pitch, float* yaw);
void MPU6050_ComplementaryFilter(float* roll, float* pitch, float* yaw, float dt);

// Global variables for angle calculation
extern float roll, pitch, yaw;
extern float prev_roll, prev_pitch, prev_yaw;

#endif
