
#ifndef _STARTINSTASK_H
#define _STARTINSTASK_H


#include "stdint.h"
#include "stdbool.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


typedef struct
{
    float Xk_accelX;
    float pk_accelX;
    float Q_accelX;
    float R_accelX;
    float Kk_accelX;

    float Xk_accelY;
    float Pk_accelY;
    float R_accelY;
    float Q_accelY;
    float Kk_accelY;

    float Xk_accelZ;
    float Pk_accelZ;
    float Q_accelZ;
    float R_accelZ;
    float Kk_accelZ;		

    float Xk_magX;
    float Pk_magX;
    float Q_magX;
    float R_magX;
    float Kk_magX;

    float Xk_magY;
    float Pk_magY;
    float Q_magY;
    float R_magY;
    float Kk_magY;

    float Xk_magZ;
    float Pk_magZ;
    float Q_magZ;
    float R_magZ;
    float Kk_magZ;
}kalman_t;

typedef struct __BMI088_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
    float GyroRawData_Z;
    float GyroRawData_Y;
}BMI088_REAL_DATA_t;

typedef struct
{
	float pit_angle;
	float yaw_angle;
	float yaw_tolangle;
	float rol_angle;

	float pit_gyro;
	float yaw_gyro;
	float rol_gyro;
	
	float last_yawangle;
	int16_t YawRoundCount;

}INS_t;

INS_t Imu;



#endif


