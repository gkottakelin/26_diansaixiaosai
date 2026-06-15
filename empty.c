/*
key2:选择途径点
key3:确认途径点
key4:依次输入完所有途径点后确认

右视觉：
识别distance1,内置卡尔曼，寻路
识别路口is_at_intersection

前视觉：
识别红绿灯

激光测距：
判断停车

在路口的逻辑
1.一开始小车向上，再右转写死
2.之后依靠灰度判断路口，并根据bfs返回数据执行逻辑

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
#include "k230_receiver.h"

uint16_t num1 = 0;
uint8_t turns = 0;
int16_t Target1, Target2, Out1, Out2, Current1;
//uint16_t car_speed = 210;
uint16_t count = 0, HD_count = 0;
uint8_t choose = 0;


char Serial_RxPacket[100];
int enc_delta;
int g_encoder_raw, g_motor_speed;
int speed = 250;
const int max_speed=230;

/*--------------------------------参数声明-----------------------------------*/
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

/*右视觉返回参数*/
int lukou_flag = 0 ;//在路口为1 ，不在路口为0
int lukou_flag1 = 0 ; //在路口为0 ，不在路口为1
int you_num ;//左右视觉返回的数值
int new_lukou_flag1 = 0 ; //视觉发送的lukou_flag暂存，作为比较
float distance_step_toA = 0;
int A_flag = 0 ;

char lujing[17] = {0};  // 假设最多16个途径点，最后一个为字符串结束符
MoveAction actions[12]; //每个路口的动作
int actions_len ;   //总动作数量

/*前视觉返回参数*/
int red_light_flag = 0 ; //红绿灯标志，识别到红灯为1，绿灯、其他为0
// 读取代码 red_light_fla g= DL_GPIO_readPins(OPENMV_PORT, OPENMV_PIN_2_PIN)  

int total_time = 0 ;
int time_start = 0 ; //为0时不计时，为1时计时

uint8_t driven_len = 0;
int park_num = 0; //停车位编号，1-6

uint8_t digtal ;

/*-------------------------------函数声明------------------------------------*/
void track(int a, int b , int set_speed);	//循迹函数
void car_turn_dir(MoveAction action);  		//转向函数
void car_u_turn(MoveAction action);       //原地转向
void update_cur_xy();											//更新坐标
void display_driven_path(void);						//展示路径
void Set_Targets_Via_Keys(void);					//选择途径点

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
	
	//temp=distance;
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
void TIMER_0_INST_IRQHandler(void) //10ms定时器
{
    static int times = 0;
    static int32_t speed_pulse_acc = 0;  // ← 新增：20ms内的脉冲累加器

    if( DL_TimerA_getPendingInterrupt(TIMER_0_INST) )
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            // 每10ms读一次增量
            int16_t enc_delta = Encoder_GetDelta();
            Encoder_AccumulateDistance(enc_delta);  // 总距离照常累加
            speed_pulse_acc += enc_delta;           // ← 速度用的脉冲先攒着

            if(times >= 2)
            {
//							//路口识别
//							if(A_flag == 1)
//							{
//								No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
//								Digtal = Get_Digtal_For_User(&sensor);
//								OLED_ShowBinNum(2, 1, Digtal, 8);
//								if ((( Digtal / 4)%2 == 1) && ((Digtal/32)%2 == 1))
//								{
//									lukou_flag = 1;
//								}
//							}
								
                if(time_start == 1)
                {
                    total_time++;
                }

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

            }
        }
    }
}


/* UART0 ISR — cam0 */
void UART_0_INST_IRQHandler(void)  // 名字必须是 UART_0_INST_IRQHandler
{
    /* 使用 UART_0_INST 替换原先的 UART0 */
    while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
        char byte = (char)DL_UART_receiveData(UART_0_INST);
        K230_FeedByte(&g_rxbuf0, &g_k230_cam0, byte);
    }
}
 
/* UART3 ISR — cam1 */
void UART_3_INST_IRQHandler(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_3_INST) == false) {
        char byte = (char)DL_UART_receiveData(UART_3_INST);
        K230_FeedByte(&g_rxbuf1, &g_k230_cam1, byte);
    }
}


//int main()
//{
//		SYSCFG_DL_init();
//    OLED_Init();
//    init_map();
//		K230_Init();
//	
//	NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
//    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
//    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
//	
//		OLED_Clear();
//		OLED_ShowString(1,1,"start");

//		car_turn_dir(ACT_TURN_LEFT);
//	
//	Left_Motor_SetPWM(0);
//	Right_Motor_SetPWM(0);


//}
int a,b ;
int action_step = 0;
int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    init_map();
		K230_Init();

    // 1. 先清空并使能所有串口中断！
//    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
//    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
		NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	
	now_x = 0 ;
	now_y = 0 ;
	car_now_dir = DIR_UP ;

    lukou_flag1 = 1;
    new_lukou_flag1 = 0;

    car_next_action.action = ACT_STOP ;
    car_next_action.next_dir = DIR_UP ;
    car_next_action.next_pos.x = 0 ;
    car_next_action.next_pos.y = 0 ;
		

	//设置途径点
    Set_Targets_Via_Keys();
    //等待处理，显示接下来路口的动作
    actions_len = compute_turn_sequence(lujing, DIR_RIGHT, DIR_RIGHT, actions);
    delay_ms(1000);

	OLED_Clear();
	OLED_ShowString(1,1,"start");
	OLED_ShowNum(2,1,g_k230_cam1.distance1,4);

    //先直行，再右转，写死代码前往A点
    Encoder_ResetDistance();
    while( Encoder_GetTotalDistance_mm() < 1100 ) //直行1米
    {
				OLED_ShowNum(2,1,g_k230_cam1.distance1,4);
				distance_step_toA = Encoder_GetTotalDistance_mm() ;
				track(115,g_k230_cam1.distance1, 200);
    }
		car_turn_dir(ACT_TURN_RIGHT);

//    Left_Motor_SetPWM(200);
//    Right_Motor_SetPWM(-200);
//    delay_ms(850);
//		
//		Left_Motor_SetPWM(0);
//    Right_Motor_SetPWM(0);
//		delay_ms(200);
//		
//		A_flag = 1;
//		Left_Motor_SetPWM(200);
//		Right_Motor_SetPWM(197);
//		delay_ms(1500);
//		
//		Left_Motor_SetPWM(0);
//    Right_Motor_SetPWM(0);
		OLED_Clear();

	int hong = 0 ;
    //现在面前A点，进入while开始正式在地图中寻路
	while( action_step<actions_len )
	{
		if(DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_2)>>2 != 0)
		{
			hong = 1;
			
		}else
		{
			hong = 0 ;
			
		}
		OLED_ShowNum(4,1,hong,2);
		if(hong == 1)
		{
				Left_Motor_SetPWM(0);
				Right_Motor_SetPWM(0);
		}else if( hong == 0 )
		{
		if (DL_GPIO_readPins(KEY_KEY2_PORT, KEY_KEY2_PIN)>>18 == 0)
        {
            a = 1;
        }else a = 0;
		if (DL_GPIO_readPins(KEY_KEY3_PORT, KEY_KEY3_PIN)>>19 == 0)
        {
            b = 1;
        }else b = 0;
        if(a==1 || b==1)
        {
            lukou_flag = 1;
        }
        if(lukou_flag == 1)//根据actions数组执行路径规划好的动作
        {
            car_turn_dir(actions[action_step]);
            action_step++;
            lukou_flag = 0;
        }
        track(115,g_k230_cam1.distance1, 200); //第一个参数为期望距离，第二个为实际右侧距离，第三个为期望速度
				OLED_ShowNum(2,1,g_k230_cam1.distance1,4);
				OLED_ShowNum(1,1,lukou_flag,2);
				OLED_ShowNum(1,4,action_step,2);
				OLED_ShowNum(3,1,a,2);
				OLED_ShowNum(3,4,b,2);
			}
    }
		 Encoder_ResetDistance();
    while( Encoder_GetTotalDistance_mm() < 1100 ) //直行1米
    {
				OLED_ShowNum(2,1,g_k230_cam1.distance1,4);
				distance_step_toA = Encoder_GetTotalDistance_mm() ;
				track(110,g_k230_cam1.distance1, 200);
    }
		Left_Motor_SetPWM(0);
		Right_Motor_SetPWM(0);
		
}
    




/*
道路循迹    
a->zuo_num  b->you_num
a>b :向左转
a<b :向右转
*/
  void track(int target, int actual, int set_speed)
  {
      static float integral = 0;
      static int   last_error = 0;

      /* ---- PID 参数（实测时调整这3个值）---- */
      const float Kp = 1.0f;   // 比例：快速响应
      const float Ki = 0.0f;  // 积分：消除稳态误差
      const float Kd = 0.0f;   // 微分：抑制振荡

      /* ---- 误差计算 ---- */
      int error = target - actual;   // actual<target → error>0 → 左转

      /* ---- 积分（抗饱和）---- */
      integral += error;
      if (integral >  300.0f) integral =  300.0f;
      if (integral < -300.0f) integral = -300.0f;

      /* ---- 微分 ---- */
      int derivative = error - last_error;
      last_error = error;

      /* ---- PID 输出 ---- */
      int pid_out = (int)(Kp * error + Ki * integral + Kd * derivative);

      /* ---- 电机控制 ---- */
      int left_pwm  = set_speed - pid_out;   // error>0 → 左轮慢 → 左转
      int right_pwm = set_speed + pid_out;   // error>0 → 右轮快 → 左转

      /* 限幅 */
      #define PWM_MAX 400
      if (left_pwm  >  PWM_MAX) left_pwm  =  PWM_MAX;
      if (left_pwm  < -PWM_MAX) left_pwm  = -PWM_MAX;
      if (right_pwm >  PWM_MAX) right_pwm =  PWM_MAX;
      if (right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;

      Left_Motor_SetPWM(left_pwm);
      Right_Motor_SetPWM(right_pwm);

      /* OLED 调试 */
//      OLED_ShowNum(4, 1, target, 4);
//      OLED_ShowNum(4, 5, actual, 4);
  }

	
/*
转向
输入StepDecision.action
*/
void car_turn_dir(MoveAction action)  //原地转向函数
{
		if(action == ACT_FORWARD){
			Left_Motor_SetPWM(200);
      Right_Motor_SetPWM(197);
      delay_ms(2500);
		}
    else if(action == ACT_TURN_RIGHT){
        Left_Motor_SetPWM(260);
        Right_Motor_SetPWM(120);
        delay_ms(2350);
    }else if(action == ACT_TURN_LEFT){
        Left_Motor_SetPWM(120);
        Right_Motor_SetPWM(260);
        delay_ms(2350);
    }else if(action == ACT_U_TURN){
        Left_Motor_SetPWM(150);
        Right_Motor_SetPWM(-150);
        delay_ms(2000); //假设转180度需要1秒，实际需要根据测试调整
    }else if(ACT_STOP){
			Left_Motor_SetPWM(0);
        Right_Motor_SetPWM(0);
        delay_ms(1000);
		}
}



/*
原地掉头
当遇到障碍物并且离开路口时，进行原地掉头
*/
void car_u_turn(MoveAction action)
{
    if(action == ACT_TURN_RIGHT){
        Left_Motor_SetPWM(200);
        Right_Motor_SetPWM(-200);
        delay_ms(205);
    }else if(action == ACT_TURN_LEFT){
        Left_Motor_SetPWM(-200);
        Right_Motor_SetPWM(200);
        delay_ms(200);
    }else if(action == ACT_U_TURN){
        Left_Motor_SetPWM(200);
        Right_Motor_SetPWM(-200);
        delay_ms(405); //假设转180度需要1秒，实际需要根据测试调整
    }
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
展示行驶过的路径
OLED先显示在9*9中的地图中的坐标变化，
再显示停车位置为x号停车位
 */
void display_driven_path(void)
{
    //显示行驶过的坐标，读取driven_history数组
    OLED_Clear();
    OLED_ShowString(1, 1, "Path:");
		
    //显示停车位置
    OLED_ShowString(4, 1, "Parked at:");
    OLED_ShowNum(4, 12 , park_num , 1);
}



/*
设置途径点
key2:选择途径点
key3:确认途径点
key4:依次输入完所有途径点后确认
*/

void Set_Targets_Via_Keys(void)
{
    uint8_t lujing_len=0;
    while(1){
        uint8_t tempxy=1;
        while(!Key4_GetNum()){
            OLED_ShowChar(1,lujing_len+1,tempxy+'A'-1);
            if(Key5_GetNum())tempxy++;
            if(Key6_GetNum())tempxy--;
            if(tempxy>=11)tempxy=1;
            if(tempxy<=0)tempxy=10;
        }
        lujing[lujing_len++]=tempxy+'A'-1;
				if(tempxy==10)break;
        
    }
    store_targets(lujing);
    OLED_ShowString(2,1,"ok");
    delay_ms(1000);
    OLED_Clear();
    
}





//int main()
//{
//	SYSCFG_DL_init();

//		OLED_Init();
//    /* 2. 初始化K230接收模块 */
//    K230_Init();
// 
//    /* 3. 使能两路UART的RX中断 */

//    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

//		NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
//	
////	delay_ms(3000);
//	
//		char *targets = Set_Targets_Via_Keys();
//		OLED_ShowString(1,1,targets);

//        delay_ms(1000);
//	while(1)
//	{
//		track(125, g_k230_cam1.distance1 , 200);
//		OLED_ShowNum(2,1,g_k230_cam1.distance1,4);
//		Left_Motor_SetPWM(198);
//		Right_Motor_SetPWM(200);
//	}
//	
//	
//}
/*
typedef enum {
    ACT_FORWARD,   // 直行进入下一格
    ACT_TURN_RIGHT,// 右转90度后直行
    ACT_U_TURN,    // 掉头180度
    ACT_TURN_LEFT, // 左转90度后直行
    ACT_STOP       // 停止 (到达终点或遇障)
}MoveAction;
*/

//void track(int a, int b , int set_speed)
//{
//		OLED_ShowNum(1, 1, a, 4);
//		OLED_ShowNum(4, 5, b, 4);
//    int k = 1 ; //修改系数
//		
//    Left_Motor_SetPWM(set_speed-(a-b)*k);
//    Right_Motor_SetPWM((set_speed+(a-b)*k));
//}



///* ================================================================
// *  K230、Openmv串口接收调试主函数
// * ================================================================ */
//int main(void)
//{
//    /* 1. SysConfig 生成的统一初始化（时钟、GPIO、UART等） */
//    SYSCFG_DL_init();

//		OLED_Init();
//    /* 2. 初始化K230接收模块 */
//    K230_Init();
// 
//    /* 3. 使能两路UART的RX中断 */

//    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

//		NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
//    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
// 
///* 4. 主循环：使用解析好的数据 */
//    while (1)
//    {
//		
//			red_light_flag= DL_GPIO_readPins(OPENMV_PORT,OPENMV_PIN_2_PIN)  ;
//			OLED_ShowNum(3, 1,red_light_flag,4);
//        /* ---------- 处理 cam1 数据 ---------- */
//        if (g_k230_cam1.valid)
//        {
//            /* 先拷贝再清标志，防止与串口RX中断发生数据竞争 */
//            K230_Frame_t snap1 = g_k230_cam1;
//            g_k230_cam1.valid  = false;
//            
//            /* --- 在 OLED 上实时显示 ma (干线角度) --- */
//            OLED_ShowString(1, 1, "ma: ");
//            /* main_angle 是 int16_t，所以必须用 ShowSignedNum 显示带符号数 
//               长度设为3，加上符号位共占用4个字符，从第5列开始显示 */
//            OLED_ShowSignedNum(1, 5, snap1.distance1 , 3); 
//						OLED_ShowSignedNum(2, 5, snap1.is_at_intersection , 3); 
//					
//            /* -------------------------------------- */

//            /* ---- 根据状态执行你的控制逻辑 ---- */
//            switch (snap1.state)
//            {
//                case K230_STATE_TRACKING:
//                    /*
//                     * 可用字段：
//                     * snap0.main_angle  干线角度（相对水平，单位degree）
//                     * snap0.main_cx/cy  干线中心坐标
//                     * snap0.main_x1/y1, main_x2/y2  干线两端点
//                     *
//                     * 示例：用中心x偏差做转向误差
//                     * int16_t err = snap0.main_cx - 120;  // 120 = 图像宽度/2
//                     * steer_cam0(err);
//                     */
//                    break;
// 
//                case K230_STATE_INTERSECT:
//                    /*
//                     * 路口节点：snap0.node1_x/y，snap0.node2_x/y
//                     * 0值表示该节点不存在
//                     */
//                    break;
// 
//                case K230_STATE_IDLE:
//                    /* 停车/空闲，无有效数据 */
//                    break;
// 
//                case K230_STATE_PARKING:
//                    /* 车库停车，干线数据有效，节点无效 */
//                    break;
// 
//                default:
//                    break;
//            }
//        }
// 
//        /* ---------- 处理 cam0 数据 ---------- */
//        // ... (保持原样) ...
//    }
//}