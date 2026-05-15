#include "JY60.h"
#include "wit_c_sdk.h"
#include "OLED.h"
#include "board.h"
float fAcc[3], fGyro[3], fAngle[3];
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}

}
static void ShowHelp(void)
{
//	printf("\r\n************************	 WIT_SDK_DEMO	************************");
//	printf("\r\n************************          HELP           ************************\r\n");
//	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
//	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
//	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
//	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
//	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
//	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
//	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
//	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
//	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
//	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
//	printf("UART SEND:h\\r\\n   help.\r\n");
//	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set AccCali Error");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set MagCali Error");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				OLED_ShowString(1,1,"Set MagCali Error");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set Bandwidth Error");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set Bandwidth Error");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set Baud Error");
			else 
				Uart2Init(c_uiBaud[WIT_BAUD_115200]);											
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				OLED_ShowString(1,1,"Set Baud Error");
			else 
				Uart2Init(c_uiBaud[WIT_BAUD_9600]);												
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set Rate Error");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set Rate Error");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set RSW Error");
			break;
		case 'c':	
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
				OLED_ShowString(1,1,"Set RSW Error");
			break;
		case 'h':
			ShowHelp();
			break;
	}
	s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
	/* 【修复说明】
	 * 原版直接调用 delay_ms()，wit_c_sdk 在主循环里回调此函数时
	 * 会阻塞数十~数百ms，导致：
	 *   1. uart2_process() 无法及时消费 buffer → buffer溢出 → SDK卡死
	 *   2. ADC 采样的 SysTick 计时被打乱 → 灰度数据异常
	 *
	 * AutoScanSensor() 在 JY60_Init() 里只跑一次，其内部直接调用
	 * delay_ms(100) 而非通过此注册函数，所以初始化扫描不受影响。
	 * 正常运行期间 SDK 无需主动延时，此处置空即可。
	 */
	(void)ucMs;
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
    int i, iRetry; 
	
    for(i = 1; i < 10; i++)
    {
        Uart2Init(c_uiBaud[i]);
        iRetry = 2;
        do
        {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            delay_ms(100);
            if(s_cDataUpdate != 0)
            {
                
//                OLED_ShowNum(1,1,c_uiBaud[i],6);
//                OLED_ShowString(2,1, "baud find sensor");
                ShowHelp();
                return ;
            }
            iRetry--;
        }while(iRetry);		
    }
    
//    OLED_ShowString(1,1, "can not find sensor");
//    
//    OLED_ShowString(2,1, "check connection");
}

void JY60_Init(){
	Uart2Init(9600);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	
	WitDelayMsRegister(Delayms);
	
	//printf("\r\n********************** wit-motion normal example  ************************\r\n");
	AutoScanSensor();
	OLED_Clear();
}

void JY60_GetData(){
	CmdProcess();
	if(s_cDataUpdate)
	{
		for(int i = 0; i < 3; i++)
		{
			fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
			fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
			fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
		}
		if(s_cDataUpdate & ACC_UPDATE)
		{
//			OLED_ShowSignedNum(1,1,fAcc[0],3);
//			OLED_ShowSignedNum(1,6,fAcc[1],3);
//			OLED_ShowSignedNum(1,11,fAcc[2],3);
			//printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
			s_cDataUpdate &= ~ACC_UPDATE;
		}
		if(s_cDataUpdate & GYRO_UPDATE)
		{
//			OLED_ShowSignedNum(2,1,fGyro[0],3);
//			OLED_ShowSignedNum(2,6,fGyro[1],3);
//			OLED_ShowSignedNum(2,11,fGyro[2],3);
			//printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
			s_cDataUpdate &= ~GYRO_UPDATE;
		}
		if(s_cDataUpdate & ANGLE_UPDATE)
		{
//			OLED_ShowSignedNum(3,1,fAngle[0],3);
//			OLED_ShowSignedNum(3,6,fAngle[1],3);
//			OLED_ShowSignedNum(3,11,fAngle[2],3);
			//printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
			s_cDataUpdate &= ~ANGLE_UPDATE;
		}
		if(s_cDataUpdate & MAG_UPDATE)
		{
//			OLED_ShowSignedNum(4,1,sReg[HX],3);
//			OLED_ShowSignedNum(4,6,sReg[HY],3);
//			OLED_ShowSignedNum(4,11,sReg[HZ],3);
			//printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
			s_cDataUpdate &= ~MAG_UPDATE;
		}
	}
}