/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_Motor */
#define PWM_Motor_INST                                                     TIMG6
#define PWM_Motor_INST_IRQHandler                               TIMG6_IRQHandler
#define PWM_Motor_INST_INT_IRQN                                 (TIMG6_INT_IRQn)
#define PWM_Motor_INST_CLK_FREQ                                          1000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_Motor_C0_PORT                                             GPIOA
#define GPIO_PWM_Motor_C0_PIN                                     DL_GPIO_PIN_21
#define GPIO_PWM_Motor_C0_IOMUX                                  (IOMUX_PINCM46)
#define GPIO_PWM_Motor_C0_IOMUX_FUNC                 IOMUX_PINCM46_PF_TIMG6_CCP0
#define GPIO_PWM_Motor_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_Motor_C1_PORT                                             GPIOA
#define GPIO_PWM_Motor_C1_PIN                                     DL_GPIO_PIN_22
#define GPIO_PWM_Motor_C1_IOMUX                                  (IOMUX_PINCM47)
#define GPIO_PWM_Motor_C1_IOMUX_FUNC                 IOMUX_PINCM47_PF_TIMG6_CCP1
#define GPIO_PWM_Motor_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for ENCODER1 */
#define ENCODER1_INST                                                      TIMG8
#define ENCODER1_INST_IRQHandler                                TIMG8_IRQHandler
#define ENCODER1_INST_INT_IRQN                                  (TIMG8_INT_IRQn)
/* Pin configuration defines for ENCODER1 PHA Pin */
#define GPIO_ENCODER1_PHA_PORT                                             GPIOA
#define GPIO_ENCODER1_PHA_PIN                                      DL_GPIO_PIN_1
#define GPIO_ENCODER1_PHA_IOMUX                                   (IOMUX_PINCM2)
#define GPIO_ENCODER1_PHA_IOMUX_FUNC                  IOMUX_PINCM2_PF_TIMG8_CCP0
/* Pin configuration defines for ENCODER1 PHB Pin */
#define GPIO_ENCODER1_PHB_PORT                                             GPIOA
#define GPIO_ENCODER1_PHB_PIN                                      DL_GPIO_PIN_0
#define GPIO_ENCODER1_PHB_IOMUX                                   (IOMUX_PINCM1)
#define GPIO_ENCODER1_PHB_IOMUX_FUNC                  IOMUX_PINCM1_PF_TIMG8_CCP1


/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (9999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_31
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_28
#define GPIO_UART_0_IOMUX_RX                                      (IOMUX_PINCM6)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM3)
#define GPIO_UART_0_IOMUX_RX_FUNC                       IOMUX_PINCM6_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                       IOMUX_PINCM3_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_0_FBRD_4_MHZ_9600_BAUD                                          (3)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_1_FBRD_4_MHZ_115200_BAUD                                       (11)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOA
#define GPIO_UART_2_TX_PORT                                                GPIOA
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_24
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_23
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM54)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM53)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM54_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM53_PF_UART2_TX
#define UART_2_BAUD_RATE                                                  (9600)
#define UART_2_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_2_FBRD_4_MHZ_9600_BAUD                                          (3)
/* Defines for UART_3 */
#define UART_3_INST                                                        UART3
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_RX_PORT                                                GPIOA
#define GPIO_UART_3_TX_PORT                                                GPIOA
#define GPIO_UART_3_RX_PIN                                        DL_GPIO_PIN_25
#define GPIO_UART_3_TX_PIN                                        DL_GPIO_PIN_26
#define GPIO_UART_3_IOMUX_RX                                     (IOMUX_PINCM55)
#define GPIO_UART_3_IOMUX_TX                                     (IOMUX_PINCM59)
#define GPIO_UART_3_IOMUX_RX_FUNC                      IOMUX_PINCM55_PF_UART3_RX
#define GPIO_UART_3_IOMUX_TX_FUNC                      IOMUX_PINCM59_PF_UART3_TX
#define UART_3_BAUD_RATE                                                  (9600)
#define UART_3_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_3_FBRD_4_MHZ_9600_BAUD                                          (3)





/* Defines for ADC1 */
#define ADC1_INST                                                           ADC1
#define ADC1_INST_IRQHandler                                     ADC1_IRQHandler
#define ADC1_INST_INT_IRQN                                       (ADC1_INT_IRQn)
#define ADC1_ADCMEM_ADC_Channel0                              DL_ADC12_MEM_IDX_0
#define ADC1_ADCMEM_ADC_Channel0_REF             DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC1_ADCMEM_ADC_Channel0_REF_VOLTAGE                                      -1 // VDDA cannot be determined
#define GPIO_ADC1_C0_PORT                                                  GPIOA
#define GPIO_ADC1_C0_PIN                                          DL_GPIO_PIN_15



/* Defines for KEY1: GPIOA.7 with pinCMx 14 on package pin 13 */
#define KEY_KEY1_PORT                                                    (GPIOA)
#define KEY_KEY1_PIN                                             (DL_GPIO_PIN_7)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM14)
/* Port definition for Pin Group GPIO_IN */
#define GPIO_IN_PORT                                                     (GPIOA)
/* Defines for GPIO_A17: GPIOA.17 with pinCMx 42 on package pin 36 */
#define GPIO_IN_GPIO_A17_PIN                                    (DL_GPIO_PIN_17)
#define GPIO_IN_GPIO_A17_IOMUX                                   (IOMUX_PINCM42)
/* Defines for GPIO_A18: GPIOA.18 with pinCMx 43 on package pin 35 */
#define GPIO_IN_GPIO_A18_PIN                                    (DL_GPIO_PIN_18)
#define GPIO_IN_GPIO_A18_IOMUX                                   (IOMUX_PINCM43)
/* Defines for KEY2: GPIOB.18 with pinCMx 44 on package pin 37 */
#define KEY_KEY2_PORT                                                    (GPIOB)
#define KEY_KEY2_PIN                                            (DL_GPIO_PIN_18)
#define KEY_KEY2_IOMUX                                           (IOMUX_PINCM44)
/* Defines for KEY3: GPIOB.19 with pinCMx 45 on package pin 38 */
#define KEY_KEY3_PORT                                                    (GPIOB)
#define KEY_KEY3_PIN                                            (DL_GPIO_PIN_19)
#define KEY_KEY3_IOMUX                                           (IOMUX_PINCM45)
/* Defines for KEY4: GPIOB.20 with pinCMx 48 on package pin 41 */
#define KEY_KEY4_PORT                                                    (GPIOB)
#define KEY_KEY4_PIN                                            (DL_GPIO_PIN_20)
#define KEY_KEY4_IOMUX                                           (IOMUX_PINCM48)
/* Defines for KEY5: GPIOB.24 with pinCMx 52 on package pin 42 */
#define KEY_KEY5_PORT                                                    (GPIOB)
#define KEY_KEY5_PIN                                            (DL_GPIO_PIN_24)
#define KEY_KEY5_IOMUX                                           (IOMUX_PINCM52)
/* Defines for KEY6: GPIOA.27 with pinCMx 60 on package pin 47 */
#define KEY_KEY6_PORT                                                    (GPIOA)
#define KEY_KEY6_PIN                                            (DL_GPIO_PIN_27)
#define KEY_KEY6_IOMUX                                           (IOMUX_PINCM60)
/* Defines for Left_Motor1: GPIOA.8 with pinCMx 19 on package pin 16 */
#define Motor_Left_Motor1_PORT                                           (GPIOA)
#define Motor_Left_Motor1_PIN                                    (DL_GPIO_PIN_8)
#define Motor_Left_Motor1_IOMUX                                  (IOMUX_PINCM19)
/* Defines for Left_Motor2: GPIOA.9 with pinCMx 20 on package pin 17 */
#define Motor_Left_Motor2_PORT                                           (GPIOA)
#define Motor_Left_Motor2_PIN                                    (DL_GPIO_PIN_9)
#define Motor_Left_Motor2_IOMUX                                  (IOMUX_PINCM20)
/* Defines for Right_Motor1: GPIOB.8 with pinCMx 25 on package pin 22 */
#define Motor_Right_Motor1_PORT                                          (GPIOB)
#define Motor_Right_Motor1_PIN                                   (DL_GPIO_PIN_8)
#define Motor_Right_Motor1_IOMUX                                 (IOMUX_PINCM25)
/* Defines for Right_Motor2: GPIOB.9 with pinCMx 26 on package pin 23 */
#define Motor_Right_Motor2_PORT                                          (GPIOB)
#define Motor_Right_Motor2_PIN                                   (DL_GPIO_PIN_9)
#define Motor_Right_Motor2_IOMUX                                 (IOMUX_PINCM26)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOB)

/* Defines for OLED_SCL: GPIOB.2 with pinCMx 15 on package pin 14 */
#define OLED_OLED_SCL_PIN                                        (DL_GPIO_PIN_2)
#define OLED_OLED_SCL_IOMUX                                      (IOMUX_PINCM15)
/* Defines for OLED_SDA: GPIOB.3 with pinCMx 16 on package pin 15 */
#define OLED_OLED_SDA_PIN                                        (DL_GPIO_PIN_3)
#define OLED_OLED_SDA_IOMUX                                      (IOMUX_PINCM16)
/* Port definition for Pin Group GWHD */
#define GWHD_PORT                                                        (GPIOA)

/* Defines for AD0: GPIOA.12 with pinCMx 34 on package pin 27 */
#define GWHD_AD0_PIN                                            (DL_GPIO_PIN_12)
#define GWHD_AD0_IOMUX                                           (IOMUX_PINCM34)
/* Defines for AD1: GPIOA.13 with pinCMx 35 on package pin 28 */
#define GWHD_AD1_PIN                                            (DL_GPIO_PIN_13)
#define GWHD_AD1_IOMUX                                           (IOMUX_PINCM35)
/* Defines for AD2: GPIOA.14 with pinCMx 36 on package pin 29 */
#define GWHD_AD2_PIN                                            (DL_GPIO_PIN_14)
#define GWHD_AD2_IOMUX                                           (IOMUX_PINCM36)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_Motor_init(void);
void SYSCFG_DL_ENCODER1_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_UART_3_init(void);
void SYSCFG_DL_ADC1_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
