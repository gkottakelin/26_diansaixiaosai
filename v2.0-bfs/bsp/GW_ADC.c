#include "GW_ADC.h"
#include "board.h"
unsigned int adc_getValue(void)
{
    unsigned int gAdcResult = 0;
		
    //使能ADC转换
    DL_ADC12_enableConversions(ADC1_INST);
    //软件触发ADC开始转换
    DL_ADC12_startConversion(ADC1_INST);
	
		//timeout使用
		uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;
		ticks = 500 * (80000000 / 1000000);//根据自己主频来我这里是80Mhz        ---------------------- 500us超时
    told = SysTick->VAL;
    //如果当前状态 不是 空闲状态
    while (DL_ADC12_getStatus(ADC1_INST) != DL_ADC12_STATUS_CONVERSION_IDLE )
		{
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;
            if (tcnt >= ticks)
                break;
        }
    }

    //清除触发转换状态
    DL_ADC12_stopConversion(ADC1_INST);
    //失能ADC转换
    DL_ADC12_disableConversions(ADC1_INST);

    //获取数据
    gAdcResult = DL_ADC12_getMemResult(ADC1_INST, ADC1_ADCMEM_ADC_Channel0);

    return gAdcResult;
}
