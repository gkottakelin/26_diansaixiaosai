/***********电机控制代码***********/
#include "Motor.h"

void Left_Motor_SetPWM(int16_t PWM2)
{
	
    if(PWM2 >= 0){
        DL_GPIO_setPins(Motor_Right_Motor1_PORT, Motor_Right_Motor1_PIN);
		DL_GPIO_clearPins(Motor_Right_Motor2_PORT, Motor_Right_Motor2_PIN);
        PWM_SetCompare2(PWM2);
    }
    else{
        DL_GPIO_clearPins(Motor_Right_Motor1_PORT, Motor_Right_Motor1_PIN);
		DL_GPIO_setPins(Motor_Right_Motor2_PORT, Motor_Right_Motor2_PIN);
        PWM_SetCompare2(-PWM2);
    }
}

void Right_Motor_SetPWM(int16_t PWM1)
{
	PWM1=PWM1*-1;
    if(PWM1 >= 0){
        DL_GPIO_setPins(Motor_Left_Motor1_PORT, Motor_Left_Motor1_PIN);
		DL_GPIO_clearPins(Motor_Left_Motor2_PORT, Motor_Left_Motor2_PIN);
        PWM_SetCompare1(PWM1);
    }
    else{
        DL_GPIO_clearPins(Motor_Left_Motor1_PORT, Motor_Left_Motor1_PIN);
		DL_GPIO_setPins(Motor_Left_Motor2_PORT, Motor_Left_Motor2_PIN);
        PWM_SetCompare1(-PWM1);
    }
}