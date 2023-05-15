//
// Created by YanYuanbin on 22-10-11.
//

#ifndef _PID_H
#define _PID_H

#include "stdint.h"
#include "stdbool.h"
#include "math.h"
#include "stm32f4xx.h"

#define VAL_Limit(x,min,max)  do{ \
																if (x > max) {x = max;} \
																else if (x < min) {x = min;} \
															}while(0U)

#define ABS(x) (((x) > 0) ? (x) : -(x))

//异常情况枚举
typedef enum
{
    PID_ERROR_NONE = 0x00U, //无异常
    Motor_Blocked = 0x01U, //堵转
		Output_NAN = 0x02U,  //nan
}ErrorType_e;

//异常情况结构体
typedef struct
{
    uint16_t ERRORCount;
    ErrorType_e ERRORType;
}PID_ErrorHandler_t;

typedef struct
{
    float kp;
    float Ki;
    float Kd;

		float Deadband;
    float maxIntegral;
    float MaxOut;
}PID_param_t;

//PID结构体
typedef struct _PID_TypeDef
{
		float Target;
		float Measure;
	
    float Err;
    float Last_Err;
		float Integral;

    float Pout;
    float Iout;
    float Dout;
    float Output;
	
		PID_param_t param;
    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
				float *para);
				
		void (*PID_clear)(
				struct _PID_TypeDef *pid);
				
}PID_TypeDef_t;

typedef struct _PID_Delta
{
    float kp;
    float ki;
    float kd;
	
		float Err[3];
    float MaxOut;

		float Pout;
    float Iout;
    float Dout;
    float Output;
}PID_Delta_t;

extern void PID_Init(
    PID_TypeDef_t *pid,
    float *para);

extern void PID_Delta_init(
		struct _PID_Delta *pid,
		float maxOut,
		float kp,
		float ki,
		float kd);

extern float f_PID_Delta_Calc(struct _PID_Delta *pid,float err);
extern float f_PID_Calculate(PID_TypeDef_t *pid, float Target,float Measure);
#endif


