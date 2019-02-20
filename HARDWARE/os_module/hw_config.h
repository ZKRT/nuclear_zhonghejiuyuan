/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  ZKRT
  * @version V0.0.1
  * @date    13-December-2016
  * @brief   hardware configure message
  ******************************************************************************
  * @attention
  *
  * ...
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported macro ------------------------------------------------------------*/

/** @defgroup hardware open or close control  
  * @{
  */
/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup clock label
  * @{
  */
#define SYSTEM_CLK                168
#define APB1_CLK                  42
#define APB2_CLK                  84
#define APB1_TIMER_CLK            84
#define APB2_TIMER_CLK            168
/**
  * @}
  */

/** @defgroup fast timer task config
  * @{
  */
#define FTTC_RCC_CLK                                            RCC_APB1Periph_TIM12
#define FTTC_TIM_NUM                                            TIM12
#define FTTC_TIMER_CLK                                          APB1_TIMER_CLK
#define TIMER_IRQ_CHANNEL_1ST                                   TIM8_BRK_TIM12_IRQn
#define NVIC_TIMER_PreemptionPriority_1ST                       0
#define NVIC_TIMER_SubPriority_1ST                              1
/**
  * @}
  */

/** @defgroup quick fast timer config
  * @{
  */
#define QTTC_RCC_CLK                                            RCC_APB1Periph_TIM14
#define QTTC_TIM_NUM                                            TIM14
#define QTTC_TIMER_CLK                                          APB1_TIMER_CLK
#define TIMER_IRQ_CHANNEL_2ND                                   TIM8_TRG_COM_TIM14_IRQn
#define NVIC_TIMER_PreemptionPriority_2ND                       0     //中断优先级最高
#define NVIC_TIMER_SubPriority_2ND                              2
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup nvic priority config
  * @{
  */
//usart1 nvic
#define NVIC_PPRIORITY_U1                        50
#define NVIC_SUBPRIORITY_U1                      50
//uart6 nvic
#define NVIC_PPRIORITY_U6                        50
#define NVIC_SUBPRIORITY_U6                      50
/**
  * @}
  */


/** @defgroup pin define
  * @{
  */
/**
  * @}
  */


/* Exported functions ------------------------------------------------------- */

#endif /* __HW_CONFIG_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT ZKRT *****END OF FILE****/

