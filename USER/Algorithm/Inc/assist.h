//
// Created by YanYuanbin on 22-10-5.
//

#ifndef GIMBAL_ASSIST_H
#define GIMBAL_ASSIST_H

#include "stm32f4xx.h"

extern float encoder_To_Angle(volatile int16_t const *encoder,float encoder_Max);

#endif //GIMBAL_ASSIST_H
