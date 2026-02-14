/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_PWM_ARR 179
#define LED_PWM_BIT_0 43
#define LED_PWM_BIT_1 86
#define ESC_1_Pin GPIO_PIN_2
#define ESC_1_GPIO_Port GPIOE
#define F1_2_Pin GPIO_PIN_3
#define F1_2_GPIO_Port GPIOE
#define F2_3_Pin GPIO_PIN_4
#define F2_3_GPIO_Port GPIOE
#define F3_4_Pin GPIO_PIN_5
#define F3_4_GPIO_Port GPIOE
#define F4_5_Pin GPIO_PIN_6
#define F4_5_GPIO_Port GPIOE
#define F5_6_Pin GPIO_PIN_13
#define F5_6_GPIO_Port GPIOC
#define F6_7_Pin GPIO_PIN_14
#define F6_7_GPIO_Port GPIOC
#define F7_8_Pin GPIO_PIN_15
#define F7_8_GPIO_Port GPIOC
#define F8_9_Pin GPIO_PIN_3
#define F8_9_GPIO_Port GPIOF
#define F9_10_Pin GPIO_PIN_4
#define F9_10_GPIO_Port GPIOF
#define F10_11_Pin GPIO_PIN_5
#define F10_11_GPIO_Port GPIOF
#define F11_12_Pin GPIO_PIN_7
#define F11_12_GPIO_Port GPIOF
#define F12_13_Pin GPIO_PIN_8
#define F12_13_GPIO_Port GPIOF
#define PrtSc_14_Pin GPIO_PIN_9
#define PrtSc_14_GPIO_Port GPIOF
#define ScrLk_15_Pin GPIO_PIN_10
#define ScrLk_15_GPIO_Port GPIOF
#define Pause_16_Pin GPIO_PIN_0
#define Pause_16_GPIO_Port GPIOF
#define Grave_17_Pin GPIO_PIN_1
#define Grave_17_GPIO_Port GPIOF
#define Key1_18_Pin GPIO_PIN_0
#define Key1_18_GPIO_Port GPIOC
#define Key2_19_Pin GPIO_PIN_1
#define Key2_19_GPIO_Port GPIOC
#define Key3_20_Pin GPIO_PIN_2
#define Key3_20_GPIO_Port GPIOC
#define Key4_21_Pin GPIO_PIN_3
#define Key4_21_GPIO_Port GPIOC
#define Key5_22_Pin GPIO_PIN_2
#define Key5_22_GPIO_Port GPIOF
#define Key6_23_Pin GPIO_PIN_1
#define Key6_23_GPIO_Port GPIOA
#define Key7_24_Pin GPIO_PIN_2
#define Key7_24_GPIO_Port GPIOA
#define Key8_25_Pin GPIO_PIN_3
#define Key8_25_GPIO_Port GPIOA
#define Key9_26_Pin GPIO_PIN_4
#define Key9_26_GPIO_Port GPIOA
#define Key0_27_Pin GPIO_PIN_5
#define Key0_27_GPIO_Port GPIOA
#define Minus_28_Pin GPIO_PIN_6
#define Minus_28_GPIO_Port GPIOA
#define Plus_29_Pin GPIO_PIN_7
#define Plus_29_GPIO_Port GPIOA
#define Backspace_30_Pin GPIO_PIN_4
#define Backspace_30_GPIO_Port GPIOC
#define Insert_31_Pin GPIO_PIN_5
#define Insert_31_GPIO_Port GPIOC
#define Home_32_Pin GPIO_PIN_0
#define Home_32_GPIO_Port GPIOB
#define PgUp_33_Pin GPIO_PIN_1
#define PgUp_33_GPIO_Port GPIOB
#define Tab_34_Pin GPIO_PIN_2
#define Tab_34_GPIO_Port GPIOB
#define Q_35_Pin GPIO_PIN_11
#define Q_35_GPIO_Port GPIOF
#define W_36_Pin GPIO_PIN_12
#define W_36_GPIO_Port GPIOF
#define E_37_Pin GPIO_PIN_13
#define E_37_GPIO_Port GPIOF
#define R_38_Pin GPIO_PIN_14
#define R_38_GPIO_Port GPIOF
#define T_39_Pin GPIO_PIN_15
#define T_39_GPIO_Port GPIOF
#define Y_40_Pin GPIO_PIN_7
#define Y_40_GPIO_Port GPIOE
#define U_41_Pin GPIO_PIN_8
#define U_41_GPIO_Port GPIOE
#define I_42_Pin GPIO_PIN_9
#define I_42_GPIO_Port GPIOE
#define O_43_Pin GPIO_PIN_10
#define O_43_GPIO_Port GPIOE
#define P_44_Pin GPIO_PIN_11
#define P_44_GPIO_Port GPIOE
#define Key__45_Pin GPIO_PIN_12
#define Key__45_GPIO_Port GPIOE
#define Key__46_Pin GPIO_PIN_13
#define Key__46_GPIO_Port GPIOE
#define Key__47_Pin GPIO_PIN_14
#define Key__47_GPIO_Port GPIOE
#define Delete_48_Pin GPIO_PIN_15
#define Delete_48_GPIO_Port GPIOE
#define End_49_Pin GPIO_PIN_10
#define End_49_GPIO_Port GPIOB
#define PgDn_50_Pin GPIO_PIN_11
#define PgDn_50_GPIO_Port GPIOB
#define Caps_51_Pin GPIO_PIN_12
#define Caps_51_GPIO_Port GPIOB
#define A_52_Pin GPIO_PIN_13
#define A_52_GPIO_Port GPIOB
#define S_53_Pin GPIO_PIN_14
#define S_53_GPIO_Port GPIOB
#define D_54_Pin GPIO_PIN_15
#define D_54_GPIO_Port GPIOB
#define F_55_Pin GPIO_PIN_8
#define F_55_GPIO_Port GPIOD
#define G_56_Pin GPIO_PIN_9
#define G_56_GPIO_Port GPIOD
#define H_57_Pin GPIO_PIN_10
#define H_57_GPIO_Port GPIOD
#define J_58_Pin GPIO_PIN_11
#define J_58_GPIO_Port GPIOD
#define K_59_Pin GPIO_PIN_12
#define K_59_GPIO_Port GPIOD
#define L_60_Pin GPIO_PIN_13
#define L_60_GPIO_Port GPIOD
#define Key__61_Pin GPIO_PIN_14
#define Key__61_GPIO_Port GPIOD
#define Key__62_Pin GPIO_PIN_15
#define Key__62_GPIO_Port GPIOD
#define Enter_63_Pin GPIO_PIN_6
#define Enter_63_GPIO_Port GPIOC
#define LShift_64_Pin GPIO_PIN_7
#define LShift_64_GPIO_Port GPIOC
#define Z_65_Pin GPIO_PIN_0
#define Z_65_GPIO_Port GPIOG
#define X_66_Pin GPIO_PIN_1
#define X_66_GPIO_Port GPIOG
#define C_67_Pin GPIO_PIN_2
#define C_67_GPIO_Port GPIOG
#define V_68_Pin GPIO_PIN_3
#define V_68_GPIO_Port GPIOG
#define B_69_Pin GPIO_PIN_4
#define B_69_GPIO_Port GPIOG
#define N_70_Pin GPIO_PIN_8
#define N_70_GPIO_Port GPIOC
#define M_71_Pin GPIO_PIN_9
#define M_71_GPIO_Port GPIOC
#define Key__72_Pin GPIO_PIN_8
#define Key__72_GPIO_Port GPIOA
#define Key__73_Pin GPIO_PIN_9
#define Key__73_GPIO_Port GPIOA
#define Key__74_Pin GPIO_PIN_10
#define Key__74_GPIO_Port GPIOA
#define RShift_75_Pin GPIO_PIN_13
#define RShift_75_GPIO_Port GPIOA
#define Up_76_Pin GPIO_PIN_6
#define Up_76_GPIO_Port GPIOF
#define LCtrl_77_Pin GPIO_PIN_14
#define LCtrl_77_GPIO_Port GPIOA
#define Win_78_Pin GPIO_PIN_15
#define Win_78_GPIO_Port GPIOA
#define LAlt_79_Pin GPIO_PIN_10
#define LAlt_79_GPIO_Port GPIOC
#define Space_80_Pin GPIO_PIN_11
#define Space_80_GPIO_Port GPIOC
#define RAlt_81_Pin GPIO_PIN_12
#define RAlt_81_GPIO_Port GPIOC
#define Menu_82_Pin GPIO_PIN_2
#define Menu_82_GPIO_Port GPIOD
#define Fn_83_Pin GPIO_PIN_3
#define Fn_83_GPIO_Port GPIOD
#define RCtrl_84_Pin GPIO_PIN_4
#define RCtrl_84_GPIO_Port GPIOD
#define Left_85_Pin GPIO_PIN_5
#define Left_85_GPIO_Port GPIOD
#define Down_86_Pin GPIO_PIN_6
#define Down_86_GPIO_Port GPIOD
#define Right_87_Pin GPIO_PIN_7
#define Right_87_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
