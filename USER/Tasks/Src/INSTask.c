

#include "cmsis_os.h"
#include "INSTask.h"

#include "bmi088.h"
#include "ist8310.h"

#include "tim.h"

#include "pid.h"
#include "kalman.h"

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "myuart.h"
//线性校准标量
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                    \
    {0.0f, 0.0f, 1.0f}                      \
		
#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

static PID_Delta_t imu_temp_pid;

static uint8_t first_temperate;

float eff_x;
float eff_y;
float eff_z;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s


float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
float INS_accel[3] = {0.0f, 0.0f, 0.0f};
float INS_mag[3] = {0.0f, 0.0f, 0.0f};
float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad   

float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};  //校准变量
float gyro_offset[3];
float gyro_cali_offset[3];

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_offset[3];
float accel_cali_offset[3];

float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float mag_offset[3];
float mag_cali_offset[3];

static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

float Eff[80] = { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
                10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
                30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 
                40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
                50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
                60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
                70, 71, 72, 73, 74, 75, 76, 77, 78, 79}; 


volatile float mygetqval[9];	//用于存放传感器转换结果的数组

BMI088_REAL_DATA_t  BMI088_read_data;
bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

kalman_t kalman;
float BMI088_FIFO[6][101] = {0};//[0]-[90]为最近10次数据 [100]为10次数据的平均值	


static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);
static void IMU_getValues(volatile float * values);
static void imu_temp_control(float temp);


/* USER CODE BEGIN Header_StartINSTask */
/**
  * @brief  Function implementing the myINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartINSTask */
void INSTask(void const * argument)
{
  /* USER CODE BEGIN StartINSTask */
		TickType_t systick;
    while(BMI088_init())  
    {
        osDelay(100);
    }
//    while(ist8310_init())
//    {
//        osDelay(100);
//    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    PID_Delta_init(&imu_temp_pid,TEMPERATURE_PID_MAX_OUT,TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD);
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    AHRS_init(INS_quat, INS_accel,INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

  /* Infinite loop */
  for(;;)
  {
			systick = osKernelSysTick();//当前系统时间
			myprintf(Imu.pit_angle*100.f,Imu.rol_angle*100.f,Imu.yaw_angle*100.f);
			IMU_getValues(mygetqval);
			imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
			imu_temp_control(bmi088_real_data.temp);

			accel_fliter_1[0] = accel_fliter_2[0];
			accel_fliter_2[0] = accel_fliter_3[0];
			accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

			accel_fliter_1[1] = accel_fliter_2[1];
			accel_fliter_2[1] = accel_fliter_3[1];
			accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

			accel_fliter_1[2] = accel_fliter_2[2];
			accel_fliter_2[2] = accel_fliter_3[2];
			accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];			

			AHRS_update(INS_quat,timing_time,INS_gyro,accel_fliter_3,INS_mag);
			get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET , INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
			
			Imu.last_yawangle = Imu.yaw_angle;
			Imu.yaw_angle = INS_angle[0] *180.f/3.1415926f;
			if(Imu.yaw_angle - Imu.last_yawangle >180.f)
			{
				Imu.YawRoundCount--;
			}
			else if(Imu.yaw_angle - Imu.last_yawangle <-180.f)
			{
				Imu.YawRoundCount++;
			}
			Imu.yaw_tolangle = Imu.YawRoundCount*360.f + Imu.yaw_angle;
			
			Imu.pit_angle = INS_angle[2] *180.f/3.1415926f;
			Imu.rol_angle = INS_angle[1] *180.f/3.1415926f;
			Imu.yaw_gyro = INS_gyro[2] *180.f/3.1415926f;
			Imu.pit_gyro = INS_gyro[1] *180.f/3.1415926f;
			Imu.rol_gyro = INS_gyro[0] *180.f/3.1415926f;

			osDelayUntil(&systick, 1);//绝对延时
  }
  /* USER CODE END StartINSTask */
}

static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

float Q_=0.001;
float R_=1000000;
static void IMU_getValues(volatile float * values) 
{
    uint8_t i = 0;
    float bmi088_sum0=0;
    float bmi088_sum1=0;
    float bmi088_sum2=0;
    float bmi088_sum3=0;
    float bmi088_sum4=0;
    float bmi088_sum5=0;
    int j;
    int k;
    int l;
    int c;
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);   //循环读取加速计、陀螺仪、温度的值
    for(i=1;i<100;i++)
    {
        BMI088_FIFO[0][i-1]=BMI088_FIFO[0][i];
        BMI088_FIFO[1][i-1]=BMI088_FIFO[1][i];
        BMI088_FIFO[2][i-1]=BMI088_FIFO[2][i];
        BMI088_FIFO[3][i-1]=BMI088_FIFO[3][i];
        BMI088_FIFO[4][i-1]=BMI088_FIFO[4][i];
        BMI088_FIFO[5][i-1]=BMI088_FIFO[5][i];
    }
    BMI088_FIFO[0][99] = bmi088_real_data.accel[0];  
    BMI088_FIFO[1][99] = bmi088_real_data.accel[1];
    BMI088_FIFO[2][99] = bmi088_real_data.accel[2];
    BMI088_FIFO[3][99] = bmi088_real_data.gyro[0];
    BMI088_FIFO[4][99] = bmi088_real_data.gyro[1];
    BMI088_FIFO[5][99] = bmi088_real_data.gyro[2];

    for(i=0;i<100;i++)//求当前数组（加速、陀螺仪的100个值）的和，再取平均值
    {	
        c++;
        bmi088_sum0 += BMI088_FIFO[0][i];	
    }
    BMI088_FIFO[0][100] = bmi088_sum0/100;
    bmi088_sum0 = 0;

    for(i=0;i<100;i++)
    {	
        bmi088_sum1 += BMI088_FIFO[1][i];
    }
    BMI088_FIFO[1][100] = bmi088_sum1/100;
    bmi088_sum1 = 0;

    for(i=0;i<100;i++)
    {	
        bmi088_sum2 += BMI088_FIFO[2][i];
    }
    BMI088_FIFO[2][100] = bmi088_sum2/100;
    bmi088_sum2 = 0;

    for(i=0;i<100;i++)
    {	
        bmi088_sum3 += BMI088_FIFO[3][i];
    }
    BMI088_FIFO[3][100] = bmi088_sum3/100;
    bmi088_sum3 = 0;

    for(i=0;i<100;i++)
    {	
        bmi088_sum4 += BMI088_FIFO[4][i];
    }
    BMI088_FIFO[4][100] = bmi088_sum4/100;
    bmi088_sum4 = 0;

    for(i=0;i<100;i++)
    {	
        bmi088_sum5 += BMI088_FIFO[5][i];
    }
    BMI088_FIFO[5][100] = bmi088_sum5/100;
    bmi088_sum5 = 0; 

    BMI088_read_data.Accel_X = BMI088_FIFO[0][100] ;
    BMI088_read_data.Accel_Y = BMI088_FIFO[1][100] ;
    BMI088_read_data.Accel_Z = BMI088_FIFO[2][100] ;	
    BMI088_read_data.Gyro_X = BMI088_FIFO[3][100]  ;
    BMI088_read_data.Gyro_Y = BMI088_FIFO[4][100]  ;
    BMI088_read_data.Gyro_Z = BMI088_FIFO[5][100]  ;	

    for(j = 0; j <50; j++)
    {
        if(BMI088_read_data.Gyro_X ==  Eff[j])
        {
            eff_x = Eff[j];
            break;
        }
        if(-BMI088_read_data.Gyro_X ==  Eff[j])
        {
            eff_x = -Eff[j];
            break;
        }
    }		
    for(k = 0; k <25; k++)
    {
        if(BMI088_read_data.Gyro_Y ==  Eff[k])
        {
            eff_y = Eff[k];
            break;
        }
        if(-BMI088_read_data.Gyro_Y ==  Eff[k])
        {
            eff_y = -Eff[k];
            break;
        }
    }	
    for(l = 0; l<23; l++)
    {
        if(BMI088_read_data.Gyro_Z ==  Eff[l])
        {
            eff_z = Eff[l];
            break;
        }
        if(-BMI088_read_data.Gyro_Z ==  Eff[l])
        {
            eff_z = -Eff[l];
            break;
        }
    }			

    kalman.Xk_accelX = kalman.Xk_accelX;   //先验估计  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    kalman.Q_accelX = Q_;//0.018
    kalman.R_accelX = R_;//0.542
    kalman.pk_accelX = kalman.pk_accelX + kalman.Q_accelX;   //先验误差 p(k|k-1) = A*p(k-1|k-1)*A'+Q
    kalman.Kk_accelX = kalman.pk_accelX / (kalman.pk_accelX + kalman.R_accelX);   //卡尔曼增益 kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    kalman.Xk_accelX = kalman.Xk_accelX + kalman.Kk_accelX*(BMI088_read_data.Accel_X - kalman.Xk_accelX);  //卡尔曼增益 kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    kalman.pk_accelX = (1 - kalman.Kk_accelX)*kalman.pk_accelX;	  //状态更新 p(k|k) = (I-kg(k)*H)*P(k|k-1)

    kalman.Xk_accelY = kalman.Xk_accelY;
    kalman.Q_accelY = Q_;
    kalman.R_accelY = R_;
    kalman.Pk_accelY = kalman.Pk_accelY + kalman.Q_accelY;
    kalman.Kk_accelY = kalman.Pk_accelY / (kalman.Pk_accelY + kalman.R_accelY);
    kalman.Xk_accelY = kalman.Xk_accelY + kalman.Kk_accelY*(BMI088_read_data.Accel_Y - kalman.Xk_accelY);
    kalman.Pk_accelY = (1 -kalman. Kk_accelY)*kalman.Pk_accelY;

    kalman.Xk_accelZ = kalman.Xk_accelZ;
    kalman.Q_accelZ = Q_;
    kalman.R_accelZ = R_;
    kalman.Pk_accelZ = kalman.Pk_accelZ + kalman.Q_accelZ;
    kalman.Kk_accelZ = kalman.Pk_accelZ / (kalman.Pk_accelZ + kalman.R_accelZ);
    kalman.Xk_accelZ = kalman.Xk_accelZ + kalman.Kk_accelZ*(BMI088_read_data.Accel_Z - kalman.Xk_accelZ);
    kalman.Pk_accelZ = (1 -kalman. Kk_accelZ)*kalman.Pk_accelZ;		

    values[0] = kalman.Xk_accelX;
    values[1] = kalman.Xk_accelY;
    values[2] = kalman.Xk_accelZ;

    BMI088_read_data.Gyro_X=BMI088_read_data.Gyro_X - eff_x;
    values[3]=BMI088_read_data.Gyro_X;
    BMI088_read_data.Gyro_Y=BMI088_read_data.Gyro_Y - eff_y;
    values[4]=BMI088_read_data.Gyro_Y;
    BMI088_read_data.Gyro_Z=BMI088_read_data.Gyro_Z - eff_z;
    values[5]=BMI088_read_data.Gyro_Z;

    INS_accel[0] = values[0];
    INS_accel[1] = values[1];
    INS_accel[2] = values[2];
    INS_gyro[0] =  values[3];
    INS_gyro[1] =  values[4];
    INS_gyro[2] =  values[5];

}

static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        f_PID_Delta_Calc(&imu_temp_pid, temp-45.0f);
        if (imu_temp_pid.Output < 0.0f)
        {
            imu_temp_pid.Output = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.Output;
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > 45.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, MPU6500_TEMP_PWM_MAX - 1);
    }
}

