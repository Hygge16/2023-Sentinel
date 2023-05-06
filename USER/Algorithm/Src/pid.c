//
// Created by YanYuanbin on 22-10-11.
//
#include "pid.h"


/**************************PID param Init*********************************/
static void PID_param_init(
    PID_TypeDef_t *pid,
    float *para)
{
	
		pid->PID_clear(pid);
	
    pid->param.Deadband = para[0];
    pid->param.maxIntegral = para[1];
    pid->param.MaxOut = para[2];

    pid->param.kp = para[3];
    pid->param.Ki = para[4];
    pid->param.Kd = para[5];

    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
	
		pid->Pout = 0;
		pid->Iout = 0;
		pid->Dout = 0;
    pid->Output = 0;
}

/**************************PID clear*********************************/
static void PID_clear(PID_TypeDef_t *pid)
{
	pid->Err = 0;
	pid->Last_Err = 0;
	pid->Integral = 0;
		
	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->Output = 0;
}

static void f_PID_ErrorHandle(PID_TypeDef_t *pid)
{
		/*Output NAN Handle*/
		if(isnan(pid->Output) == true)
		{
				pid->ERRORHandler.ERRORType = Output_NAN;
		}
}

/***************************PID calculate**********************************/
float f_PID_Calculate(PID_TypeDef_t *pid, float Target,float Measure)
{
    if (pid == NULL)
    {
				return 0; 
    }
		
		f_PID_ErrorHandle(pid);
		if(pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
		{
			pid->PID_clear(pid);
			return 0;
		}
		
		pid->Target = Target;
		pid->Measure = Measure;
		
		pid->Last_Err = pid->Err;
		
    pid->Err = pid->Target-pid->Measure;
		
		if(ABS(pid->Err) > pid->param.Deadband)
		{
			pid->Integral += pid->Err;
			VAL_Limit(pid->Integral,-pid->param.maxIntegral,pid->param.maxIntegral);
			
			pid->Pout = pid->param.kp * pid->Err;
			pid->Iout = pid->param.Ki * pid->Integral;
			pid->Dout = pid->param.Kd * (pid->Err - pid->Last_Err);
			
			pid->Output = pid->Pout + pid->Iout + pid->Dout;
			VAL_Limit(pid->Output,-pid->param.MaxOut,pid->param.MaxOut);
		}
		
    return pid->Output;
}


void PID_Init(
    PID_TypeDef_t *pid,
    float *para)
{
		pid->PID_clear = PID_clear;
    pid->PID_param_init = PID_param_init;
    pid->PID_param_init(pid, para);
}

float f_PID_Delta_Calc(struct _PID_Delta *pid,float err)
{
	if(pid == NULL)
	{
		return 0;
	}
	
	pid->Err[2] = pid->Err[1];
	pid->Err[1] = pid->Err[0];
	pid->Err[0] = err;
	
	pid->Pout = pid->kp * (pid->Err[0] - pid->Err[1]);
	pid->Iout = pid->ki * pid->Err[0];
	pid->Dout = pid->kd * (pid->Err[0] - 2.f*pid->Err[1] + pid->Err[2]);
	
	pid->Dout = pid->Pout + pid->Iout + pid->Dout;
	VAL_Limit(pid->Dout,-pid->MaxOut,pid->MaxOut);
	
	return pid->Dout;
}

void PID_Delta_init(
	struct _PID_Delta *pid,
	float maxOut,
	float kp,
	float ki,
	float kd)
{
	pid->MaxOut = maxOut;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

