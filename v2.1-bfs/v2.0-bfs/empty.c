/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "math.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "GWHD.h"
#include "bsp_uart.h"
#include "JY60.h"
#include "Encoder.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "bsp_bluetooth.h"
#include "bfs.h"

uint16_t num1 = 0;
uint8_t turns = 0;
int16_t Target1, Target2, Out1, Out2, Current1;
//uint16_t car_speed = 210;
uint16_t count = 0, HD_count = 0;
uint8_t choose = 0;
int back = 0;
int tuo = 0;
char Serial_RxPacket[100];
int enc_delta;
int g_encoder_raw, g_motor_speed;
int speed = 250;
const int max_speed=230;
int distance = 0;
int chao = 0;
int zhong = 0;
int heng = 20;
int shu = 15;
int zhuan = 0;
float chang;
void track(int a, int b)
{
    int c = 0;
    int i = 0;
    int y[8];
    int x[8] = {60, 30, 10, 20, -10, -20, -30, -60};
    for (; i < 8; i++) {
        y[i] = (!(a % 2));
        a = a / 2;
    }
    y[2] = y[2] || y[3];
    y[3] = y[2];
    y[4] = y[5] || y[4];
    y[5] = y[4];
    for (i = 0; i < 8; i++)
        c = c + y[i] * x[i];
//    OLED_ShowNum(4, 1, b - c - 8, 4);
//    OLED_ShowNum(4, 5, b + c, 4);
//	Target1=b - c - 9;
//	Target2=b+c;
		if(c!=0)
			b=b-20;
    Left_Motor_SetPWM((b - c - 9));
    Right_Motor_SetPWM((b + c));
}

int mymax(int x, int y)
{
    return x > y ? x : y;
}

int mymin(int x, int y)
{
    return x < y ? x : y;
}

void run(int16_t x)
{
    Right_Motor_SetPWM(x);
    Left_Motor_SetPWM(x);
}

void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true)
        ;
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

void uart0_send_string(char* str)
{
    while (*str != 0 && str != 0) {
        uart0_send_char(*str++);
    }
}

void trans(void)
{
    uint8_t length = 0;
    Serial_RxPacket[length++] = '@';
    Serial_RxPacket[length++] = turns % 4 + '0';
    Serial_RxPacket[length++] = '\r';
    Serial_RxPacket[length++] = '\n';
    Serial_RxPacket[length++] = 0;
    uart0_send_string(Serial_RxPacket);
}

void transbluetooth(){
	int temp=Current1;
	uint8_t length = 0;
    Serial_RxPacket[length++] = '@';
    if(temp<0)Serial_RxPacket[length++] = '-',temp=-temp;
	Serial_RxPacket[length++] = '0';
	Serial_RxPacket[length++] = '.';
	Serial_RxPacket[length++] =temp/100%10+'0';
	Serial_RxPacket[length++] =temp/10%10+'0';
	Serial_RxPacket[length++] =temp%10+'0';
	Serial_RxPacket[length++] = 'm';
	Serial_RxPacket[length++] = '/';
	Serial_RxPacket[length++] = 's';
    Serial_RxPacket[length++] = '\r';
    Serial_RxPacket[length++] = '\n';
    Serial_RxPacket[length++] = 0;
    Bluetooth_SendString(Serial_RxPacket);
	
	temp=distance;
	length=0;
	Serial_RxPacket[length++] = '@';
	Serial_RxPacket[length++] =temp/100%10+'0';
	Serial_RxPacket[length++] =temp/10%10+'0';
	Serial_RxPacket[length++] =temp%10+'0';
	Serial_RxPacket[length++] = 'm';
	Serial_RxPacket[length++] = 'm';
    Serial_RxPacket[length++] = '\r';
    Serial_RxPacket[length++] = '\n';
    Serial_RxPacket[length++] = 0;
    Bluetooth_SendString(Serial_RxPacket);
}

uint8_t zjflag = 0;

/**
 * @brief 定时器中断服务函数
 * 假设你在 SysConfig 里配置了一个 50ms 或 100ms 触发一次的中断
 */

void HardFault_Handler(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "HARDFAULT");
    while (1)
        ;
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    JY60_Init();

    // 1. 先清空并使能所有串口中断！
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
	
	while(1){
		if(Bluetooth_RxFlag==1){
			//OLED_ShowNum(3,1,1,1);
			OLED_ShowString(1,1,Bluetooth_RxPacket);
			store_targets(Bluetooth_RxPacket);
			Bluetooth_RxFlag=0;
			break;
		}
	}
	
	OLED_ShowString(1,1,"ok");
	delay_ms(1000);
	OLED_Clear();
	search_path(0,0,0xff,0xff);//前两个参数是当前位置的xy，不用管目的地参数，目的地是预存好的，会跟着你到的位置更新
								//后两个参数表示在往哪个坐标走的时候遇到障碍物，如果正常行驶就赋值为0xff
	for(int i=0;i<path_len;++i){
		OLED_ShowNum(i/3+1,i%3*5+1,path[i].x,1);//路径存放在path[]中，这是个x，y坐标结构体，每次路径的长度是path_len
		OLED_ShowNum(i/3+1,i%3*5+3,path[i].y,1);
	}
	delay_ms(5000);
	OLED_Clear();
	
	search_path(1,3,2,3);
	for(int i=0;i<path_len;++i){
		OLED_ShowNum(i/3+1,i%3*5+1,path[i].x,1);
		OLED_ShowNum(i/3+1,i%3*5+3,path[i].y,1);
	}
	delay_ms(5000);
	OLED_Clear();
	
	search_path(2,3,0xff,0xff);
	for(int i=0;i<path_len;++i){
		OLED_ShowNum(i/3+1,i%3*5+1,path[i].x,1);
		OLED_ShowNum(i/3+1,i%3*5+3,path[i].y,1);
	}
	
	while(1);
	
	
	
	
	
	
	
	
	
	
	

    while (choose == 0) {
        OLED_ShowNum(1, 1, 6, 1);
        if (Key4_GetNum() == 1)
            choose = 1;
        if (Key5_GetNum() == 1)
            choose = 2;
    }
	
//	while(1){
//		Left_Motor_SetPWM(191);	
//        Right_Motor_SetPWM(200);
//	}
	
    OLED_Clear();
    if (choose == 1) {
        while (Key4_GetNum() == 0) {
            OLED_ShowString(1, 10, "white");
            No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
            Get_Anolog_Value(&sensor, Anolog);
            OLED_ShowNum(1, 1, Anolog[0], 4);
            OLED_ShowNum(1, 6, Anolog[1], 4);
            OLED_ShowNum(2, 1, Anolog[2], 4);
            OLED_ShowNum(2, 6, Anolog[3], 4);
            OLED_ShowNum(3, 1, Anolog[4], 4);
            OLED_ShowNum(3, 6, Anolog[5], 4);
            OLED_ShowNum(4, 1, Anolog[6], 4);
            OLED_ShowNum(4, 6, Anolog[7], 4);
            memset(rx_buff, 0, 256);
            delay_ms(1);
        }
        for (int i = 0; i < 8; ++i)
            white[i] = Anolog[i];
        OLED_Clear();
        OLED_ShowString(1, 10, "black");
        delay_ms(500);
        while (Key4_GetNum() == 0) {
            No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
            Get_Anolog_Value(&sensor, Anolog);
            OLED_ShowNum(1, 1, Anolog[0], 4);
            OLED_ShowNum(1, 6, Anolog[1], 4);
            OLED_ShowNum(2, 1, Anolog[2], 4);
            OLED_ShowNum(2, 6, Anolog[3], 4);
            OLED_ShowNum(3, 1, Anolog[4], 4);
            OLED_ShowNum(3, 6, Anolog[5], 4);
            OLED_ShowNum(4, 1, Anolog[6], 4);
            OLED_ShowNum(4, 6, Anolog[7], 4);
            memset(rx_buff, 0, 256);
            delay_ms(1);
        }
        for (int i = 0; i < 8; ++i)
            black[i] = Anolog[i];
        OLED_Clear();
        OLED_ShowString(1, 1, " ok           ok");
		Bluetooth_SendString("@A1NJ32\r\n");
		Bluetooth_SendString("@130mm/s\r\n");
        delay_ms(1000);
        OLED_Clear();
        No_MCU_Ganv_Sensor_Init(&sensor, white, black);

		int c=0;
        while (1) {
            No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
            Digtal = Get_Digtal_For_User(&sensor);
            Get_Normalize_For_User(&sensor, Normal);
            memset(rx_buff, 0, 256);
            OLED_ShowBinNum(1, 1, Digtal, 8);
            delay_ms(1);                                   //灰度
            uart2_process();
            JY60_GetData();
            uart1_process();
            
            if(c<5)
              c++;
            if(c==5&&(chao==0||chao==6))				
			      {transbluetooth();
						c=0;}
                uint16_t dist = distance_value;
                uint8_t conf = confidence_value;
                OLED_ShowNum(2, 1, dist, 4);
                //  OLED_ShowNum(2, 9, conf, 2);
                //  OLED_ShowSignedNum(3, 1, fAngle[0], 3);
                //  OLED_ShowSignedNum(3, 6, fAngle[1], 3);
                OLED_ShowSignedNum(3, 11, fAngle[2], 3);
                distance_ready = 0; // 清除标志
                distance = dist;
                if ((dist <= 200) && dist != 0)
                    speed = speed - 5;
                if ((dist > 200) && dist != 0)
                    speed = speed + 4;
                if ((dist > 300) || (dist == 0))
                    speed = max_speed;
                if (speed >= max_speed)
                    speed = max_speed;
            
            OLED_ShowSignedNum(4, 1,Current1 , 4);
            //OLED_ShowSignedNum(4, 8, g_motor_speed, 5);

           if ((distance <= 210) && (distance != 0) && Digtal == 255 && chao == 0&&fAngle[2]>0) {
                Left_Motor_SetPWM(-150);
                Right_Motor_SetPWM(150);
                chao = 1;
            }
						   if ((distance <= 210) && (distance != 0) && Digtal == 255 && chao == 0&&fAngle[2]<0) {
                Left_Motor_SetPWM(150);
                Right_Motor_SetPWM(-150);
                chao = 7;
            }
            if (chao == 1 && ((fAngle[2] >= 160) || (fAngle[2] <= -160))) {
                Left_Motor_SetPWM(390);
                Right_Motor_SetPWM(400);
                delay_ms(1000);
                Left_Motor_SetPWM(150);
                Right_Motor_SetPWM(-150);
                chao = 2;
            }
						if (chao == 7 && ((fAngle[2] >= 160) || (fAngle[2] <= -160))) {
                Left_Motor_SetPWM(390);
                Right_Motor_SetPWM(400);
                delay_ms(1000);
                Left_Motor_SetPWM(-150);
                Right_Motor_SetPWM(150);
                chao = 8;
            }
            if (chao == 2 && (fAngle[2] >= 80) && (fAngle[2] <= 100)) {
                Left_Motor_SetPWM(380);
                Right_Motor_SetPWM(400);
                delay_ms(3000);
                Left_Motor_SetPWM(150);
                Right_Motor_SetPWM(-150);
                chao = 3;
            }
						if (chao == 8 && (fAngle[2] >=-100) && (fAngle[2] <=-75)) {
                Left_Motor_SetPWM(390);
                Right_Motor_SetPWM(400);
                delay_ms(3000);
                Left_Motor_SetPWM(-150);
                Right_Motor_SetPWM(150);
                chao = 9;
            }
            if (chao == 3 && (fAngle[2] >= 55) && (fAngle[2] <= 70)) {
                Left_Motor_SetPWM(290);
                Right_Motor_SetPWM(300);
                Delay_ms(1000);
                Left_Motor_SetPWM(192);
                Right_Motor_SetPWM(200);
                chao = 4;
            }
					if (chao == 9 && (fAngle[2] >=-70) && (fAngle[2] <=-55)) {
                Left_Motor_SetPWM(290);
                Right_Motor_SetPWM(300);
                Delay_ms(1000);
                Left_Motor_SetPWM(192);
                Right_Motor_SetPWM(200);
                chao = 10;
            }
            if (chao == 4 && Digtal < 255) {
                chao = 5;
                delay_ms(200);
                Left_Motor_SetPWM(-100);
                Right_Motor_SetPWM(200);
            }
						 if (chao == 10 && Digtal < 255) {
                chao = 11;
                delay_ms(200);
                Left_Motor_SetPWM(200);
                Right_Motor_SetPWM(-100);
            }
            if (chao == 5 && (fAngle[2] >= 75) && (fAngle[2] <= 100))
                chao = 6;
						 if (chao == 11 && (fAngle[2] >=-100) && (fAngle[2] <=-75))
                chao = 6;
            if ((back == 0) && (Digtal == 0) && ((chao == 0) || (chao == 6))) {
                tuo = fAngle[2];
                back = back + 1;
                Left_Motor_SetPWM(150);
                Right_Motor_SetPWM(-150);
            }
						
            if ((fAngle[2] >= -10) && (fAngle[2] <= 10) && back == 1)
                back = back + 1;
            if (back > 1 && Digtal == 0)
                while (1) {
                    Left_Motor_SetPWM(0);
                    Right_Motor_SetPWM(0);
                }
            if (back != 1 && ((chao == 0) || (chao == 6)))
                track(Digtal, speed);
        }
    } else {
        OLED_Clear();
		
		{OLED_ShowString(1,1,"x:");
		int tempxy=0;
		while(!Key4_GetNum()){
			OLED_ShowNum(1,3,tempxy*10,2);
			if(Key5_GetNum())tempxy++;
			if(Key6_GetNum())tempxy--;
			if(tempxy>=10)tempxy-=10;
			if(tempxy<0)tempxy+=10;
		}
		heng=tempxy*10;
		tempxy=0;
		while(!Key4_GetNum()){
			OLED_ShowNum(1,3,heng+tempxy,2);
			if(Key5_GetNum())tempxy++;
			if(Key6_GetNum())tempxy--;
			if(tempxy>=10)tempxy-=10;
			if(tempxy<0)tempxy+=10;
		}
		heng+=tempxy;
		tempxy=0;
		OLED_ShowString(2,1,"y:");
		while(!Key4_GetNum()){
			OLED_ShowNum(2,3,tempxy*10,2);
			if(Key5_GetNum())tempxy++;
			if(Key6_GetNum())tempxy--;
			if(tempxy>=10)tempxy-=10;
			if(tempxy<0)tempxy+=10;
		}
		shu=tempxy*10;
		tempxy=0;
		while(!Key4_GetNum()){
			OLED_ShowNum(2,3,shu+tempxy,2);
			if(Key5_GetNum())tempxy++;
			if(Key6_GetNum())tempxy--;
			if(tempxy>=10)tempxy-=10;
			if(tempxy<0)tempxy+=10;
		}
		shu+=tempxy;
		OLED_ShowString(2, 7, " ok!");
        delay_ms(2000);
	}
		
        float angle_red;
        float angle_deg;
        angle_red = atan2(heng, shu);
        angle_deg = -angle_red * 180 / 3.1415;
	      chang=sqrt(heng*heng+shu*shu)*100;
        Left_Motor_SetPWM(110);
        Right_Motor_SetPWM(-110);
        while (1) {
            uart2_process();
            JY60_GetData();
            OLED_ShowSignedNum(3, 11, fAngle[2], 3);
            OLED_ShowSignedNum(3, 3, angle_deg, 3);
            //OLED_ShowSignedNum(4, 1, total_mm, 5);
            OLED_ShowSignedNum(4, 7, enc_delta, 5);
            if (fAngle[2] <= angle_deg)
                zhuan = 1;
            if (fAngle[2] == angle_deg && zhuan == 1) {
                Left_Motor_SetPWM(201);
                Right_Motor_SetPWM(200);
            }
            if (fAngle[2] > angle_deg && zhuan == 1) {
                Left_Motor_SetPWM(220);
                Right_Motor_SetPWM(200);
            }
            if (fAngle[2] < angle_deg && zhuan == 1) {
                Left_Motor_SetPWM(200);
                Right_Motor_SetPWM(220);
            }   
						float total_mm = Encoder_GetTotalDistance_mm();
			OLED_ShowSignedNum(4, 1, total_mm, 5);
						if(total_mm>=chang)
						   while(1)
							{Left_Motor_SetPWM(0);
              Right_Motor_SetPWM(0);}
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    static int times = 0;
    static int32_t speed_pulse_acc = 0;  // ← 新增：20ms内的脉冲累加器

    if( DL_TimerA_getPendingInterrupt(TIMER_0_INST) )
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            times++;
            HD_count++;
           // count += Key3_GetNum();

            // 每1ms读一次增量
            int16_t enc_delta = Encoder_GetDelta();
            Encoder_AccumulateDistance(enc_delta);  // 总距离照常累加
            speed_pulse_acc += enc_delta;           // ← 速度用的脉冲先攒着

            if(times >= 20)
            {
                times = 0;
                Out1 = Target1;
                Out2 = Target2;

                // 用20ms内的总脉冲数算速度
                // 单位 mm/s：脉冲数 / 每圈脉冲 × 周长mm × (1000ms/20ms)
                Current1 = (int16_t)((float)speed_pulse_acc
                            / ENCODER_PULSES_PER_WHEEL_TURN
                            * WHEEL_CIRCUMFERENCE_MM
                            * 50.0f);
                speed_pulse_acc = 0;  // ← 清零，准备下一个20ms

                if (Out1 >  1000) Out1 =  1000;
                if (Out1 < -1000) Out1 = -1000;
                if (Out2 >  1000) Out2 =  1000;
                if (Out2 < -1000) Out2 = -1000;

//                Left_Motor_SetPWM(Out1);
//                Right_Motor_SetPWM(Out2);
            }
        }
    }
}