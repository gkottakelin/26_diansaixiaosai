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
 
 
 /*
key1:选择途径点
key2:确认途径点
key3:依次输入完所有途径点后确认

左右视觉：
1.输出某参数，根据参数大小对比，决定小车在道路上的运动
2.输出是否达到路口，是否离开路口

前视觉：
识别红绿灯，识别障碍物

刷新位置函数触发：左右视觉的路口变量均改变

在路口的逻辑
1.当左右视觉的路口变量均改变时，代表进入路口，car_now_xy,car_now_dir刷新得到新的当前位置
2.路径规划
3.如果前进方向改变：先原地转弯，后直行
4.当离开路口后，接收到左右视觉的路口变量均改变，再次刷新当前位置
5.若转弯后识别到障碍物，则代入障碍物坐标，重新规划路线
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

/*
typedef enum {
    DIR_UP = 0,    // 朝向 Y 增加的方向 //向上
    DIR_RIGHT = 1, // 朝向 X 增加的方向 //向右
    DIR_DOWN = 2,  // 朝向 Y 减小的方向 //向下
    DIR_LEFT = 3   // 朝向 X 减小的方向 //向左
}NavDirection;

typedef struct {
    Point next_pos;       // 下一步的坐标
    NavDirection next_dir;   // 执行动作后，小车的新绝对朝向
    MoveAction action;    // 单片机需要执行的电机动作
}StepDecision;
*/
int now_x , now_y ;        //小车当前位置
NavDirection car_now_dir = DIR_UP; //小车当前运动方向 ，默认摆放位置向上

StepDecision car_next_action ;  //小车下一步的坐标，新朝向，需要执行的动作

uint8_t get_j = 0; //是否到达J点的标志，0为未到达，1为已到达

/* 途径点设置 */
// 可供选择的赛题节点 (起点 O, 途径点 A~J)
static const char available_nodes[11] = {'O', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'};
char task_sequence[32] = {0}; //途径点

/*左右视觉返回参数*/
int lukou_flag1 = 0 ; //在路口为0 ，不在路口为1
int lukou_flag2 = 0 ;
int zuo_num  ,  you_num ;//左右视觉返回的数值
int new_lukou_flag1 = 0 ; //视觉发送的lukou_flag暂存，作为比较
int new_lukou_flag2 = 0 ; //
int tingchewei = 0 ; //当识别到车位时，tingchewei=1，未识别到车位时，tingchewei=0


/*前视觉返回参数*/
int red_light_flag = 0 ; //红绿灯标志，识别到红灯为1，绿灯、其他为2
int obstacle_flag = 0 ; //障碍物标志，识别到障碍物为1，未识别到为0

/*
道路循迹    
a->zuo_num  b->you_num
a>b :向左转
a<b :向右转
*/
void track(int a, int b , int set_speed)
{
		OLED_ShowNum(1, 1, a, 4);
		OLED_ShowNum(4, 5, b, 4);
    int k = 1 ; //修改系数
		
    Left_Motor_SetPWM(set_speed-(a-b)*k);
    Right_Motor_SetPWM((set_speed+(a-b)*k));
}

/*
更新坐标
在每次进入/离开路口后刷新调用
根据前进方向刷新位置
*/
void update_cur_xy()
{
	if(car_now_dir == DIR_UP){
		now_y ++ ;
	}else if(car_now_dir == DIR_RIGHT){
		now_x ++ ;
	}else if(car_now_dir == DIR_DOWN){
		now_y -- ;
	}else if(car_now_dir == DIR_LEFT){
		now_x -- ;
	}
}

/*
原地转向
输入StepDecision.action
*/
void car_turn_dir(MoveAction action)  //原地转向函数
{
    if(action == ACT_TURN_RIGHT){
        Left_Motor_SetPWM(150);
        Right_Motor_SetPWM(-150);
        delay_ms(1000);
    }else if(action == ACT_TURN_LEFT){
        Left_Motor_SetPWM(-150);
        Right_Motor_SetPWM(150);
        delay_ms(1000);
    }else if(action == ACT_U_TURN){
        Left_Motor_SetPWM(150);
        Right_Motor_SetPWM(-150);
        delay_ms(2000); //假设转180度需要1秒，实际需要根据测试调整
    }
}

/*
设置途径点
key1:选择途径点
key2:确认途径点
key3:依次输入完所有途径点后确认
*/
char* Set_Targets_Via_Keys(void)
{
    uint8_t current_idx = 0;      
    uint8_t sequence_len = 0;     
    bool is_finished = false;     
    int i;                        
    
    // 初始化清空序列
    for(i = 0; i < 32; i++) {
        task_sequence[i] = '\0';
    }

    OLED_ShowString(1, 1, "Set Targets:");
    OLED_ShowString(2, 1, "                "); 
    OLED_ShowString(3, 1, "Next: ");
    OLED_ShowChar(3, 7, available_nodes[current_idx]); 

    while (!is_finished)
    {
        // Key1: 选择途径点
        if (Key1_GetNum() == 1)
        {
            current_idx++;
            // 直接使用数字11进行判断，规避任何宏定义和sizeof造成的编译异常
            if (current_idx >= 11)
            {
                current_idx = 0; 
            }
            OLED_ShowChar(3, 7, available_nodes[current_idx]);
        }

        // Key2: 确认途径点
        if (Key2_GetNum() == 1)
        {
            if (sequence_len < 16) 
            {
                task_sequence[sequence_len] = available_nodes[current_idx];
                sequence_len++;
                task_sequence[sequence_len] = '\0';
                
                current_idx = 1; // 默认切回'A'
                
                OLED_ShowString(2, 1, task_sequence);
                OLED_ShowChar(3, 7, available_nodes[current_idx]);
            }
        }

        // Key3: 完成确认
        if (Key3_GetNum() == 1)
        {
            if (sequence_len > 0) 
            {
                OLED_ShowString(4, 1, "Input Finished!");
                delay_ms(1000); 
                is_finished = true;
            }
        }
    }
    
    return task_sequence;
}

//最大值
int mymax(int x, int y)
{
    return x > y ? x : y;
}
//最小值
int mymin(int x, int y)
{
    return x < y ? x : y;
}
//直行
void run(int16_t speed)     
{
    Right_Motor_SetPWM(speed);
    Left_Motor_SetPWM(speed);
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
    init_map();

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
	
	now_x = 0 ;
	now_y = 0 ;
	car_now_dir = DIR_UP ;

    lukou_flag1 = 0;
    lukou_flag2 = 0;
    new_lukou_flag1 = 0;
    new_lukou_flag2 = 0;

    car_next_action.action = ACT_STOP ;
    car_next_action.next_dir = DIR_UP ;
    car_next_action.next_pos.x = 0 ;
    car_next_action.next_pos.y = 0 ;

	
    /*设置途径点*/
    char *targets = Set_Targets_Via_Keys();
    OLED_Clear();
    OLED_ShowString(1,1,"input map"); //导入途径点字符串，解析并存入全局变量targets中
    delay_ms(1000);
    OLED_Clear();
    /*将途径点导入bfs函数*/
    store_targets(targets);
    OLED_ShowString(1,1,"ok");
    delay_ms(1000);
    OLED_Clear();

    
    /*开始运行在地图中导航*/
    /*到达J点后跳出while循环*/
    /*
    期间在OLED显示当前坐标，当前方向
    下一步坐标，下一步方向
    */
 /*

在路口的逻辑
1.当左右视觉的路口变量均改变时，代表进入路口，car_now_xy,car_now_dir刷新得到新的当前位置
2.路径规划
3.如果前进方向改变：先原地转弯，后直行
4.当离开路口后，接收到左右视觉的路口变量均改变，再次刷新当前位置
5.若转弯后识别到障碍物，则代入障碍物坐标，重新规划路线
6.转弯，
*/

    while(get_j == 0) //外部中断判定是否到达，防止延迟  暂时放在void TIMER_0_INST_IRQHandler(void)
    {
        /*读取视觉参数*/
        /*左右视觉返回参数*/
        new_lukou_flag1 = ;  //在路口为0 ，不在路口为1
        new_lukou_flag2 = ;
        zuo_num  =  ;//左右视觉返回的数值
        you_num  =  ;//左右视觉返回的数值

        /*前视觉返回参数*/
        red_light_flag =  ; //红绿灯标志，识别到红灯为1，绿灯、其他为2
        obstacle_flag =  ; //障碍物标志，识别到障碍物为1，未识别到为0

        /*识别红绿灯*/
        while(red_light_flag == 1)
        {
            red_light_flag = ;//读取红绿灯标志，直到非红
            //停车
            Left_Motor_SetPWM(0);
            Right_Motor_SetPW(0);
            oled_ShowString(3, 1, "stop");
        }
        oled_ShowString(3, 1, "run ");

        //更新坐标
         if(lukou_flag1 != new_lukou_flag1 && lukou_flag2 != new_lukou_flag2) {
            update_cur_xy(); //根据当前运动方向更新坐标
            lukou_flag1 = new_lukou_flag1 ;
            lukou_flag2 = new_lukou_flag2;
        }
        //显示当前位置与运动方向
        OLED_ShowNum(1, 1, now_x, 2);
        OLED_ShowNum(1, 4, now_y, 2);
        if(car_now_dir == DIR_UP) {
            OLED_ShowString(1, 7, "UP ");
        } else if(car_now_dir == DIR_RIGHT) {
            OLED_ShowString(1, 7, "you");
        } else if(car_now_dir == DIR_DOWN) {
            OLED_ShowString(1, 7, "xia");
        } else if(car_now_dir == DIR_LEFT) {
            OLED_ShowString(1, 7, "zuo");
        }
        

        //抵达路口或者遇到障碍物后，进行路径规划 //在路口为0 ，不在路口为1
        //如果识别到障碍物，代入障碍物坐标，重新规划路线
        /*障碍物只会出现在路上
        当车在路口发现障碍物，则障碍物坐标为小车面前的坐标
        当车在道路上发现障碍物，则障碍物坐标为小车当前坐标
        */
       
        if((lukou_flag1 == 0 && lukou_flag2 == 0) || obstacle_flag == 1) {
            
            if(obstacle_flag == 1)
            {
                uint8_t obs_x, obs_y;
                if(lukou_flag1 == 0 && lukou_flag2 == 0) { //在路口发现障碍物
                    obs_x = now_x;
                    obs_y = now_y;
                    if(car_now_dir == DIR_UP) {
                        obs_y += 1;
                    } else if(car_now_dir == DIR_RIGHT) {
                        obs_x += 1;
                    } else if(car_now_dir == DIR_DOWN) {
                        obs_y -= 1;
                    } else if(car_now_dir == DIR_LEFT) {
                        obs_x -= 1;
                    }
                } else { //在道路上发现障碍物
                    obs_x = now_x;
                    obs_y = now_y;
                }
                car_next_action = search_path(now_x, now_y, car_now_dir, obs_x, obs_y); //代入障碍物坐标重新规划路线
            }
            else if(lukou_flag1 == 0 && lukou_flag2 == 0)
            {
                car_next_action = search_path(now_x, now_y, car_now_dir, 0xff, 0xff); //正常路径规划
            }
        }

        //显示下一步的行动
        OLED_ShowNum(2, 1, car_next_action.next_pos.x, 2);
        OLED_ShowNum(2, 4, car_next_action.next_pos.y, 2);
        if(car_next_action.next_dir == DIR_UP) {
            OLED_ShowString(2, 7, "UP ");
        } else if(car_next_action.next_dir == DIR_RIGHT) {
            OLED_ShowString(2, 7, "you");
        } else if(car_next_action.next_dir == DIR_DOWN) {
            OLED_ShowString(2, 7, "xia");
        } else if(car_next_action.next_dir == DIR_LEFT) {
            OLED_ShowString(2, 7, "zuo");
        }

        
        //调整方向，用的delay，所以不干扰循迹，以及转弯过程中的路口flag跳动
       if(car_now_dir != car_next_action.next_dir) {
            car_turn_dir(car_next_action.action);
            car_now_dir = car_next_action.next_dir;
        }

        track(zuo_num, you_num, 200); //循迹前进，参数为左右视觉返回的数值，根据数值调整运动状态

    }
    OLED_Clear();
    OLED_ShowString(1,1,"get J")
    /*从J点到停车场道闸
    1.转向朝右方向
    2.直行期间读取前视觉，闸道关闭发送红灯，闸道开启发送绿灯
    */
    if(car_now_dir == DIR_UP)
    {
        car_turn_dir(ACT_TURN_RIGHT);
    
    }
    else if(car_now_dir == DIR_DOWN)
    {
        car_turn_dir(ACT_TURN_LEFT);
    }
    else if(car_now_dir == DIR_LEFT)
    {
        car_turn_dir(ACT_U_TURN);
    }
    //方向调转完成
    //直行1.3米，根据闸道开关状态调整，视觉复用obstacle_flag参数，识别到障碍物为1，未识别到为0
    int distance_to_gate = 1300; //单位mm，假设从J点到道闸的距离为1.3米
    int distance = 0; //估算的行驶距离
    int distance_jinru_tingchechang = 1800 ;
    int get_in = 0;
    while(get_in == 0)
    {
        track(zuo_num, you_num, 200); //循迹前进，参数为左右视觉返回的数值，根据数值调整运动状态
        distance += Current1 * 20 / 1000; //根据速度估算距离，Current1单位mm/s，20ms更新一次，所以乘以20/1000

        obstacle_flag = ; //读取前视觉障碍物标志，识别到障碍物为1，未识别到为0

        //抵达闸道，根据视觉识别结果停车或继续前进
        if(distance >= distance_to_gate && obstacle_flag == 1) {
            //抵达闸道且识别到障碍物（假设障碍物即为关闭的闸道），停车
            Left_Motor_SetPWM(0);
            Right_Motor_SetPWM(0);
            OLED_ShowString(3, 1, "wait gate");
        } else if(distance >= distance_to_gate && obstacle_flag == 0) {
            //抵达闸道且未识别到障碍物（假设未识别到障碍物即为开启的闸道），继续前进
            OLED_ShowString(3, 1, "go on ");
        }

        if(distance >= distance_jinru_tingchechang) {
            get_in = 1; //进入停车场
        }
    }
    OLED_Clear();
    OLED_ShowString(1,1,"kaishi tingche");
    /*进入停车场后，寻找停车位并停车
    1.调整车头方向向下
    2.直行
    3.根据左视觉返回的数值调整运动状态，寻找空车位
    4.当识别到车位时，停车
    5.进入车位
    */
    int park_finished = 0;
    car_turn_dir(ACT_TURN_RIGHT);
     //方向调转完成
     while(park_finished == 0)
     {
          track(zuo_num, you_num, 150); //循迹前进，参数为左右视觉返回的数值，根据数值调整运动状态
    
          tingchewei = ; //读取左视觉返回的车位识别标志，当识别到车位时，tingchewei=1，未识别到车位时，tingchewei=0
    
          if(tingchewei == 1) {
                //识别到车位，停车
                Left_Motor_SetPWM(0);
                Right_Motor_SetPWM(0);
                OLED_ShowString(3, 1, "get park");
                delay_ms(1000);

                car_turn_dir(ACT_TURN_lEFT); //调整车头方向，假设车位在左侧，实际需要根据车位位置调整转向

                int distance_to_park = 500; //假设车位深度为0.5米
                int distance_in_park = 0;
                while(distance_in_park < distance_to_park) {
                    track(zuo_num, you_num, 100); //继续前进进入车位
                    distance_in_park += Current1 * 20 / 1000; //根据速度估算进入车位的距离
                }
                park_finished = 1;
          }
     }
    OLED_Clear();
    OLED_ShowString(1,1,"finish");
    

}

void TIMER_0_INST_IRQHandler(void)
{
    static int times = 0;
    static int32_t speed_pulse_acc = 0;  // ← 新增：20ms内的脉冲累加器

    if( DL_TimerA_getPendingInterrupt(TIMER_0_INST) )
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            //检测是否到达J点
            if(now_x == 6 && now_y == 8)
            {
                get_j = 1;
            }

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