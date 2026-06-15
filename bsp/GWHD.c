#include "GWHD.h"

static uint8_t BIT[10];

void GWHD_Init(){
	No_MCU_Ganv_Sensor_Init_Frist(&sensor);
	No_MCU_Ganv_Sensor_Init(&sensor,white,black);
}

void GWHD_Jiaozhun(){
	uint8_t KeyNum=0;
	
	OLED_ShowString(1,1,"white");
	delay_ms(500);
	OLED_Clear();
	while(!KeyNum){
		No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		Get_Anolog_Value(&sensor,Anolog);
		OLED_ShowNum(1,1,Anolog[0],4);
		OLED_ShowNum(1,6,Anolog[1],4);
		OLED_ShowNum(2,1,Anolog[2],4);
		OLED_ShowNum(2,6,Anolog[3],4);
		OLED_ShowNum(3,1,Anolog[4],4);
		OLED_ShowNum(3,6,Anolog[5],4);
		OLED_ShowNum(4,1,Anolog[6],4);
		OLED_ShowNum(4,6,Anolog[7],4);
		memset(rx_buff,0,256);
		Delay_ms(1);
		KeyNum=Key2_GetNum();
	}
	for(int i=0;i<8;++i)white[i]=Anolog[i];
	
	OLED_Clear();
	KeyNum=0;
	OLED_ShowString(1,1,"black");
	delay_ms(500);
	OLED_Clear();
	while(!KeyNum){
		No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		Get_Anolog_Value(&sensor,Anolog);
		OLED_ShowNum(1,1,Anolog[0],4);
		OLED_ShowNum(1,6,Anolog[1],4);
		OLED_ShowNum(2,1,Anolog[2],4);
		OLED_ShowNum(2,6,Anolog[3],4);
		OLED_ShowNum(3,1,Anolog[4],4);
		OLED_ShowNum(3,6,Anolog[5],4);
		OLED_ShowNum(4,1,Anolog[6],4);
		OLED_ShowNum(4,6,Anolog[7],4);
		memset(rx_buff,0,256);
		Delay_ms(1);
		KeyNum=Key2_GetNum();
	}
	for(int i=0;i<8;++i)black[i]=Anolog[i];
	KeyNum=0;
	OLED_Clear();
	OLED_ShowString(1,1,"ok");
	delay_ms(500);
	OLED_Clear();
	No_MCU_Ganv_Sensor_Init(&sensor,white,black);
}

void GWHD_Work(){
	No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
	Digtal=Get_Digtal_For_User(&sensor);
	Get_Normalize_For_User(&sensor,Normal);
	
	for(int i=0;i<8;++i){
		BIT[i+1]=(Digtal>>i)&0x01;
		OLED_ShowNum(1,i+1,BIT[i+1],1);
	}
//	OLED_ShowNum(1,1,(Digtal>>0)&0x01,1);
//	OLED_ShowNum(1,2,(Digtal>>1)&0x01,1);
//	OLED_ShowNum(1,3,(Digtal>>2)&0x01,1);
//	OLED_ShowNum(1,4,(Digtal>>3)&0x01,1);
//	OLED_ShowNum(1,5,(Digtal>>4)&0x01,1);
//	OLED_ShowNum(1,6,(Digtal>>5)&0x01,1);
//	OLED_ShowNum(1,7,(Digtal>>6)&0x01,1);
//	OLED_ShowNum(1,8,(Digtal>>7)&0x01,1);
	
	memset(rx_buff,0,256);
	Delay_ms(1);
}

uint16_t GWHD_AnalyzeData(uint16_t num){
    uint16_t err1 = 5, err2 = 8, err3 = 12;
    uint8_t leftnum = BIT[1] + BIT[2] + BIT[3] + BIT[4];
    uint8_t rightnum = BIT[5] + BIT[6] + BIT[7] + BIT[8];
    
//    if(leftnum + rightnum == 0) return 400;    // 全黑，即停止信号
//    if(leftnum + rightnum == 8 && num==400) return 180;
	
    if(num == 200 || num == 180){              // 急左转未完成
        if(leftnum >= 2 && (BIT[4] == 0||BIT[5]==0)) return 0;
        else return 180;
    }
    
    if(num == 300 || num == 80){               // 急右转未完成
        if(rightnum >= 1 && (BIT[5] == 0||BIT[6]==0)) return 0;
        else return 80;
    }
	
	//if(leftnum + rightnum == 8)return 200;
    
    if(leftnum < rightnum){                    // 确定需向左偏转程度
        if(leftnum <= 1) return 200;           // 急左转
        if(BIT[1] == 0) return 200;
        if(BIT[2] == 0) return err2 + 100;
        if(BIT[3] == 0) return err1 + 100;
    }
    
    if(leftnum > rightnum){                    // 确定需向右偏转程度
        if(rightnum <= 1) return 300;          // 急右转
        if(BIT[8] == 0) return err3;
        if(BIT[7] == 0) return err2;
        if(BIT[6] == 0) return err1;
    }
    
    return 0;
}
