#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "ti_msp_dl_config.h"

/* 函数功能：采集8个通道的模拟值并进行均值滤波
   参数说明：result - 存储8个通道处理结果的数组 */
void Get_Analog_value(unsigned short *result)
{
    unsigned char i,j;
    unsigned int Anolag=0;
    
    for(i=0;i<8;i++)
    {
        Switch_Address_0(!(i&0x01));
        Switch_Address_1(!(i&0x02));
        Switch_Address_2(!(i&0x04));

        /* 切换通道后，等待多路选择器稳定。
           UART中断可能在地址切换后、ADC采样前插入，
           导致采到错误通道的值，引起灰区误判。
           此处不做 delay，而是通过多次采样 + 丢弃首次值来规避。*/
        Get_adc_of_user(); /* 丢弃第1次：地址切换后可能尚未稳定 */

        for(j=0;j<5;j++)
        {
            Anolag+=Get_adc_of_user();
        }
        if(!Direction) result[i]  = Anolag/5;
        else           result[7-i]= Anolag/5;
        Anolag=0;
    }
}

/* 函数功能：将模拟值转换为数字信号（二值化处理）
 *
 * 【BUG修复说明】
 * 原版使用双阈值滞回（白/黑各一个阈值），中间灰区"保持原状"。
 * 当UART中断频繁打断ADC采样时，采到的值更容易落入灰区，
 * 导致 Digital 字节长时间不被更新，表现为"卡死"。
 * 模拟量 Anolog 直接赋值不受影响，因此能正常刷新。
 *
 * 修复方案：改为单阈值（取白/黑阈值中点），每次强制写入0或1，
 * 彻底消除"灰区保持"问题，中断干扰只影响单次采样精度，
 * 不会导致状态锁死。
 *
 * 若确实需要抗抖动滞回特性，可在上层做软件滤波（如连续N次一致
 * 才更新），而不是在底层保留不更新的状态。
 */
void convertAnalogToDigital(unsigned short *adc_value,
                             unsigned short *Gray_white,
                             unsigned short *Gray_black,
                             unsigned char  *Digital)
{
    for (int i = 0; i < 8; i++)
    {
        /* 取白/黑阈值的中点作为单一判决门限 */
        unsigned short mid = (Gray_white[i] + Gray_black[i]) / 2;

        if (adc_value[i] > mid)
            *Digital |=  (1 << i);   /* 偏亮 → 白（置1） */
        else
            *Digital &= ~(1 << i);   /* 偏暗 → 黑（置0） */
    }
}

/* 函数功能：归一化ADC值到指定范围 */
void normalizeAnalogValues(unsigned short *adc_value,
                           double         *Normal_factor,
                           unsigned short *Calibrated_black,
                           unsigned short *result,
                           double          bits)
{
    for (int i = 0; i < 8; i++)
    {
        unsigned short n;
        if(adc_value[i] < Calibrated_black[i])
            n = 0;
        else
            n = (adc_value[i] - Calibrated_black[i]) * Normal_factor[i];

        if (n > bits) n = bits;
        result[i] = n;
    }
}

/* 函数功能：传感器结构体初始化（首次） */
void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor *sensor)
{
    memset(sensor->Calibrated_black, 0, 16);
    memset(sensor->Calibrated_white, 0, 16);
    memset(sensor->Normal_value,     0, 16);
    memset(sensor->Analog_value,     0, 16);

    for(int i = 0; i < 8; i++)
        sensor->Normal_factor[i] = 0.0;

    sensor->Digtal   = 0;
    sensor->Time_out = 0;
    sensor->Tick     = 0;
    sensor->ok       = 0;
}

/* 函数功能：传感器完整初始化（带校准参数） */
void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor  *sensor,
                              unsigned short *Calibrated_white,
                              unsigned short *Calibrated_black)
{
    No_MCU_Ganv_Sensor_Init_Frist(sensor);

    if     (Sensor_ADCbits == _8Bits)  sensor->bits = 255.0;
    else if(Sensor_ADCbits == _10Bits) sensor->bits = 1024.0;
    else if(Sensor_ADCbits == _12Bits) sensor->bits = 4096.0;
    else if(Sensor_ADCbits == _14Bits) sensor->bits = 16384.0;

    if(Sensor_Edition == Class) sensor->Time_out = 1;
    else                        sensor->Time_out = 10;

    double         Normal_Diff[8];
    unsigned short temp;

    for (int i = 0; i < 8; i++)
    {
        if(Calibrated_black[i] >= Calibrated_white[i])
        {
            temp                 = Calibrated_white[i];
            Calibrated_white[i]  = Calibrated_black[i];
            Calibrated_black[i]  = temp;
        }

        sensor->Gray_white[i]      = (Calibrated_white[i]*2 + Calibrated_black[i]) / 3;
        sensor->Gray_black[i]      = (Calibrated_white[i]   + Calibrated_black[i]*2) / 3;
        sensor->Calibrated_black[i]= Calibrated_black[i];
        sensor->Calibrated_white[i]= Calibrated_white[i];

        if ((Calibrated_white[i] == 0 && Calibrated_black[i] == 0) ||
            (Calibrated_white[i] == Calibrated_black[i]))
        {
            sensor->Normal_factor[i] = 0.0;
            continue;
        }

        Normal_Diff[i]           = (double)Calibrated_white[i] - (double)Calibrated_black[i];
        sensor->Normal_factor[i] = sensor->bits / Normal_Diff[i];
    }
    sensor->ok = 1;
}

/* 函数功能：传感器主任务（无定时器版本） */
void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor *sensor)
{
    Get_Analog_value(sensor->Analog_value);
    convertAnalogToDigital(sensor->Analog_value,
                           sensor->Gray_white,
                           sensor->Gray_black,
                           &sensor->Digtal);
    normalizeAnalogValues(sensor->Analog_value,
                          sensor->Normal_factor,
                          sensor->Calibrated_black,
                          sensor->Normal_value,
                          sensor->bits);
}

/* 函数功能：传感器主任务（带定时器版本） */
void No_Mcu_Ganv_Sensor_Task_With_tick(No_MCU_Sensor *sensor)
{
    if(sensor->Tick >= sensor->Time_out)
    {
        Get_Analog_value(sensor->Analog_value);
        convertAnalogToDigital(sensor->Analog_value,
                               sensor->Gray_white,
                               sensor->Gray_black,
                               &sensor->Digtal);
        normalizeAnalogValues(sensor->Analog_value,
                              sensor->Normal_factor,
                              sensor->Calibrated_black,
                              sensor->Normal_value,
                              sensor->bits);
        sensor->Tick = 0;
    }
}

/* 函数功能：定时器tick递增 */
void Task_tick(No_MCU_Sensor *sensor)
{
    sensor->Tick++;
}

/* 函数功能：获取数字信号状态 */
unsigned char Get_Digtal_For_User(No_MCU_Sensor *sensor)
{
    return sensor->Digtal;
}

/* 函数功能：获取归一化后的数据 */
unsigned char Get_Normalize_For_User(No_MCU_Sensor *sensor, unsigned short *result)
{
    if(!sensor->ok) return 0;
    memcpy(result, sensor->Normal_value, 16);
    return 1;
}

/* 函数功能：获取原始模拟数据 */
unsigned char Get_Anolog_Value(No_MCU_Sensor *sensor, unsigned short *result)
{
    Get_Analog_value(sensor->Analog_value);
    memcpy(result, sensor->Analog_value, 16);
    if(!sensor->ok) return 0;
    else            return 1;
}