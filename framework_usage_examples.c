/**
 * @file framework_usage_examples.c
 * @brief Examples demonstrating the new framework architecture
 * @author Nhan Vo
 * @date 2025
 */

#include "gy87_mpu6050.h"
#include "framework.h"
#include "usart.h"

/**
 * @brief Example 1: Basic sensor reading with new architecture
 */
void Example1_SensorReading_NewArchitecture(void)
{
    UART_SendString("=== Example 1: Basic Sensor Reading ===\r\n");
    
    // Variables for sensor data
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    
    // Read all sensors using the clean interface
    if (GY87_Read_All_Sensors(&accel_x, &accel_y, &accel_z,
                              &gyro_x, &gyro_y, &gyro_z,
                              &mag_x, &mag_y, &mag_z))
    {
        // Print individual sensor values
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
            "Accelerometer: X=%.3f Y=%.3f Z=%.3f m/s²\r\n",
            accel_x, accel_y, accel_z);
        UART_SendString(buffer);
        
        snprintf(buffer, sizeof(buffer),
            "Gyroscope: X=%.3f Y=%.3f Z=%.3f rad/s\r\n",
            gyro_x, gyro_y, gyro_z);
        UART_SendString(buffer);
        
        snprintf(buffer, sizeof(buffer),
            "Magnetometer: X=%.6f Y=%.6f Z=%.6f Tesla\r\n",
            mag_x, mag_y, mag_z);
        UART_SendString(buffer);
    }
    else
    {
        UART_SendString("Failed to read sensors!\r\n");
    }
    
    UART_SendString("\r\n");
}

/**
 * @brief Example 2: Framework-based data processing
 */
void Example2_FrameworkProcessing(void)
{
    UART_SendString("=== Example 2: Framework Processing ===\r\n");
    
    // Create sensor data structure
    SensorData_t sensor_data;
    
    // Read sensors into individual variables first
    if (GY87_Read_All_Sensors(&sensor_data.accel_x, &sensor_data.accel_y, &sensor_data.accel_z,
                              &sensor_data.gyro_x, &sensor_data.gyro_y, &sensor_data.gyro_z,
                              &sensor_data.mag_x, &sensor_data.mag_y, &sensor_data.mag_z))
    {
        // Use framework for high-level processing
        if (Framework_Process_And_Send(&sensor_data))
        {
            UART_SendString("Data frame processed and sent successfully!\r\n");
            
            // Optional: Print human-readable data
            Framework_Print_SensorData_UART(&sensor_data);
        }
        else
        {
            UART_SendString("Failed to process data frame!\r\n");
        }
    }
    else
    {
        UART_SendString("Failed to read sensors!\r\n");
    }
    
    UART_SendString("\r\n");
}

/**
 * @brief Example 3: Manual data frame creation and validation
 */
void Example3_ManualFrameProcessing(void)
{
    UART_SendString("=== Example 3: Manual Frame Processing ===\r\n");
    
    SensorData_t sensor_data;
    DataFrame_t data_frame;
    
    // Read sensors
    if (GY87_Read_All_Sensors(&sensor_data.accel_x, &sensor_data.accel_y, &sensor_data.accel_z,
                              &sensor_data.gyro_x, &sensor_data.gyro_y, &sensor_data.gyro_z,
                              &sensor_data.mag_x, &sensor_data.mag_y, &sensor_data.mag_z))
    {
        // Manual frame creation
        if (Framework_Create_DataFrame(&data_frame, &sensor_data))
        {
            UART_SendString("Data frame created successfully!\r\n");
            
            // Print frame information
            Framework_Print_DataFrame_Info(&data_frame);
            
            // Validate frame
            if (Framework_Validate_DataFrame(&data_frame))
            {
                UART_SendString("Frame validation: PASSED\r\n");
                
                // Send frame
                Framework_Send_DataFrame_UART(&data_frame);
                UART_SendString("Frame sent via UART\r\n");
            }
            else
            {
                UART_SendString("Frame validation: FAILED\r\n");
            }
        }
        else
        {
            UART_SendString("Failed to create data frame!\r\n");
        }
    }
    else
    {
        UART_SendString("Failed to read sensors!\r\n");
    }
    
    UART_SendString("\r\n");
}

/**
 * @brief Example 4: Individual sensor access (legacy compatibility)
 */
void Example4_IndividualSensorAccess(void)
{
    UART_SendString("=== Example 4: Individual Sensor Access ===\r\n");
    
    // Read MPU6050 data
    if (GY87_MPU6050_Read_Data())
    {
        float ax = GY87_MPU6050_Get_Ax();
        float ay = GY87_MPU6050_Get_Ay();
        float az = GY87_MPU6050_Get_Az();
        float temp = GY87_MPU6050_Get_Temperature();
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
            "MPU6050 Individual Access:\r\n"
            "  Accel: X=%.3f Y=%.3f Z=%.3f m/s²\r\n"
            "  Temperature: %.1f°C\r\n",
            ax, ay, az, temp);
        UART_SendString(buffer);
    }
    
    // Read HMC5883L data
    if (GY87_HMC5883L_Read_Data())
    {
        float mx = GY87_HMC5883L_Get_Mx();
        float my = GY87_HMC5883L_Get_My();
        float mz = GY87_HMC5883L_Get_Mz();
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
            "HMC5883L Individual Access:\r\n"
            "  Magnetometer: X=%.6f Y=%.6f Z=%.6f Tesla\r\n",
            mx, my, mz);
        UART_SendString(buffer);
    }
    
    UART_SendString("\r\n");
}

/**
 * @brief Example 5: Statistics and monitoring
 */
void Example5_StatisticsMonitoring(void)
{
    UART_SendString("=== Example 5: Statistics Monitoring ===\r\n");
    
    // Reset statistics
    Framework_Reset_Statistics();
    UART_SendString("Statistics reset.\r\n");
    
    // Simulate some data processing
    SensorData_t sensor_data;
    for (int i = 0; i < 10; i++)
    {
        if (GY87_Read_All_Sensors(&sensor_data.accel_x, &sensor_data.accel_y, &sensor_data.accel_z,
                                  &sensor_data.gyro_x, &sensor_data.gyro_y, &sensor_data.gyro_z,
                                  &sensor_data.mag_x, &sensor_data.mag_y, &sensor_data.mag_z))
        {
            Framework_Process_And_Send(&sensor_data);
        }
        HAL_Delay(10); // 100Hz simulation
    }
    
    // Print statistics
    Framework_Print_Statistics();
    
    UART_SendString("\r\n");
}

/**
 * @brief Example 6: 100Hz data streaming loop
 */
void Example6_HighFrequencyStreaming(void)
{
    UART_SendString("=== Example 6: 100Hz Streaming ===\r\n");
    UART_SendString("Starting 100Hz data streaming (press reset to stop)...\r\n");
    
    uint32_t last_time = HAL_GetTick();
    uint32_t frame_count = 0;
    
    // Reset statistics for clean measurement
    Framework_Reset_Statistics();
    
    while (1)
    {
        uint32_t current_time = HAL_GetTick();
        
        // 100Hz timing (10ms period)
        if (current_time - last_time >= 10)
        {
            last_time = current_time;
            
            SensorData_t sensor_data;
            
            // Read all sensors
            if (GY87_Read_All_Sensors(&sensor_data.accel_x, &sensor_data.accel_y, &sensor_data.accel_z,
                                      &sensor_data.gyro_x, &sensor_data.gyro_y, &sensor_data.gyro_z,
                                      &sensor_data.mag_x, &sensor_data.mag_y, &sensor_data.mag_z))
            {
                // Process and send via framework
                Framework_Process_And_Send(&sensor_data);
                frame_count++;
            }
            
            // Print statistics every 5 seconds
            static uint32_t last_stats_time = 0;
            if (current_time - last_stats_time >= 5000)
            {
                last_stats_time = current_time;
                
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "Frames sent: %lu\r\n", frame_count);
                UART_SendString(buffer);
                
                Framework_Print_Statistics();
            }
        }
    }
}

/**
 * @brief Main function to run all examples
 */
void Framework_Examples_Run(void)
{
    UART_SendString("\r\n=== Framework Architecture Examples ===\r\n\r\n");
    
    // Initialize system
    GY87_I2C_Scanner();
    GY87_Init_All_Sensors();
    HAL_Delay(100);
    
    // Run examples
    Example1_SensorReading_NewArchitecture();
    HAL_Delay(1000);
    
    Example2_FrameworkProcessing();
    HAL_Delay(1000);
    
    Example3_ManualFrameProcessing();
    HAL_Delay(1000);
    
    Example4_IndividualSensorAccess();
    HAL_Delay(1000);
    
    Example5_StatisticsMonitoring();
    HAL_Delay(1000);
    
    // Final example runs indefinitely
    Example6_HighFrequencyStreaming();
}
