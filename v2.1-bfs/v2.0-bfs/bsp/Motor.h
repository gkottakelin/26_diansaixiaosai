#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ti_msp_dl_config.h"
#include "PWM.h"

void Left_Motor_SetPWM(int16_t PWM);
void Right_Motor_SetPWM(int16_t PWM);
//void Run(int8_t PWM);
//void Stop();

#endif