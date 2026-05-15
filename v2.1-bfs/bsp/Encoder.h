#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ============================================================
 *  编码器参数说明
 *  - 编码器 PPR        : 13
 *  - QEI 正交解码倍频  : ×4
 *  - 减速比            : 1:28
 *  - 每圈总脉冲        : 13 × 4 × 28 = 1456 脉冲/车轮转一圈
 *  - 车轮直径          : 67 mm  → 周长 = π × 67 ≈ 210.49 mm
 *  - 每脉冲距离        : 210.49 / 1456 ≈ 0.14456 mm/脉冲
 * ============================================================ */

#define ENCODER_PULSES_PER_WHEEL_TURN   (1456)          /* 脉冲/圈   */
#define WHEEL_CIRCUMFERENCE_MM          (210.49f)       /* mm/圈     */
#define MM_PER_PULSE                    (WHEEL_CIRCUMFERENCE_MM / ENCODER_PULSES_PER_WHEEL_TURN)  /* ≈0.14456 mm */

/* 对外接口 */

/**
 * @brief  读取编码器增量并清零计数器（用于速度计算，与原逻辑一致）
 * @return 本次调用距上次调用之间的脉冲增量（有符号，负值=反转）
 */
int16_t Encoder_GetDelta(void);

/**
 * @brief  获取开机至今的累计行驶距离（mm，有符号，倒退为负）
 * @return float，单位 mm
 */
float   Encoder_GetTotalDistance_mm(void);

/**
 * @brief  重置累计距离为 0（例如每段路程重新计算时调用）
 */
void    Encoder_ResetDistance(void);

/**
 * @brief  在定时器中断里调用，传入本次脉冲增量，内部累加距离
 *         注意：只需把原来 Encoder_Get() 的返回值传进来即可
 * @param  delta  本次脉冲增量
 */
void    Encoder_AccumulateDistance(int16_t delta);

#endif /* __ENCODER_H__ */