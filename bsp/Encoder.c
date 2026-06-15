/***********编码器距离累计***********/
#include "Encoder.h"
#include "ti_msp_dl_config.h"

/* 累计脉冲数（有符号 32 位，防止 int16 溢出；正转为正，反转为负） */
static volatile int32_t s_total_pulses = 0;

/* ------------------------------------------------------------------ */
/*  读取 QEI 增量并清零（保持与原 Encoder_Get 完全相同的行为）         */
/* ------------------------------------------------------------------ */
int16_t Encoder_GetDelta(void)
{
    int16_t delta = (int16_t)DL_Timer_getTimerCount(ENCODER1_INST);
    DL_Timer_setTimerCount(ENCODER1_INST, 0);
    return delta;
}

/* ------------------------------------------------------------------ */
/*  在定时器中断里调用：把增量累加到总脉冲数                           */
/* ------------------------------------------------------------------ */
void Encoder_AccumulateDistance(int16_t delta)
{
    s_total_pulses += (int32_t)delta;
}

/* ------------------------------------------------------------------ */
/*  对外查询：返回开机至今的累计距离（mm）                             */
/* ------------------------------------------------------------------ */
float Encoder_GetTotalDistance_mm(void)
{
    /* 读取时短暂关中断，防止读到一半被中断打断导致数据撕裂 */
    __disable_irq();
    int32_t pulses = s_total_pulses;
    __enable_irq();

    return (float)pulses * MM_PER_PULSE;
}

/* ------------------------------------------------------------------ */
/*  重置累计距离                                                       */
/* ------------------------------------------------------------------ */
void Encoder_ResetDistance(void)
{
    __disable_irq();
    s_total_pulses = 0;
    __enable_irq();
}