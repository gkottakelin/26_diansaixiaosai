/*************电机PWM配置*************/
#include "PWM.h"

//电机输出比较
void PWM_SetCompare1(uint16_t Compare) //设置左轮占空比
{
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, Compare,GPIO_PWM_Motor_C0_IDX);
}

void PWM_SetCompare2(uint16_t Compare) //设置右轮占空比
{
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, Compare,GPIO_PWM_Motor_C1_IDX);
}