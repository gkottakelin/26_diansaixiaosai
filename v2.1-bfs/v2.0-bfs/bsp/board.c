#include "ti_msp_dl_config.h"

void delay_us(uint32_t us){
	uint32_t ticks=us*(CPUCLK_FREQ/1000000);
	uint32_t count_new=0,count_old=0;
	uint32_t count=0;
	
	count_old=SysTick->VAL;
	
	while(1){
		count_new=SysTick->VAL;
		
		if(count_new!=count_old){
			if(count_new<count_old){
				count=count+(count_old-count_new);
			}else if(count_new>count_old){
				count=count+SysTick->LOAD-count_new+count_old;
			}
			
			count_old=count_new;
			
			if(count>=ticks)return;
		}
	}
}

/**
 * 毫秒级延时（基于微秒延时实现）
 * @param _ms 要延时的毫秒数
 */
void delay_ms(uint32_t _ms)
{
    delay_us(_ms * 1000);
}

// 兼容其他延时函数接口
void Delay_ms(uint32_t ms)
{
    delay_ms(ms);
}

void Delay_us(uint32_t us)
{
    delay_us(us);
}
    