//
// Created by YanYuanbin on 22-10-5.
//

#include "assist.h"
#include "pid.h"


/**
  * @name   encoder_To_Angle
  * @brief  角度转换
  * @param
  *         encoder:编码器角度        encoder_Max:编码器最大值（量程）
  * @retval result_Angle:周角角度
  * @attention
*/
float encoder_To_Angle(volatile int16_t const *encoder,float encoder_Max)
{
    return (float)*encoder / encoder_Max * 360.0f;
}


