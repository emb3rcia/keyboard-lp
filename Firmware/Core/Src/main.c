/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbpd.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_UCPD1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t keyboard_report[11];
// made with **help** of Google's NotebookLM
#define NUM_LEDS 87
#define BITS_PER_LED 24
#define PWM_HI 130
#define PWM_LO 43
uint32_t led_buffer[NUM_LEDS * BITS_PER_LED]; // 87 diodes, 24 bits per diode 1 byte per color
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_UCPD1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
4
  /* USER CODE END 2 */

  /* USBPD initialisation ---------------------------------*/
  MX_USBPD_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    USBPD_DPM_Run();

    /* USER CODE BEGIN 3 */
    //made with **help** of chatgpt cuz it is my first usb-hid project in stm32cube and not kmk
    for(int i=0; i<11; i++) keyboard_report[i] = 0;

    //check every gpio pin
    if(HAL_GPIO_ReadPin(GPIOE, ESC_1_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, F1_2_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, F2_3_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, F3_4_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, F4_5_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, F5_6_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, F6_7_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, F7_8_Pin) == GPIO_PIN_RESET) keyboard_report[0] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, F8_9_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, F9_10_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, F10_11_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, F11_12_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, F12_13_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, PrtSc_14_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, ScrLk_15_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Pause_16_Pin) == GPIO_PIN_RESET) keyboard_report[1] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, Grave_17_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, Key1_18_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, Key2_19_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, Key3_20_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, Key4_21_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, Key5_22_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, Key6_23_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Key7_24_Pin) == GPIO_PIN_RESET) keyboard_report[2] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, Key8_25_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, Key9_26_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, Key0_27_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, Minus_28_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, Plus_29_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, Backspace_30_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, Insert_31_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Home_32_Pin) == GPIO_PIN_RESET) keyboard_report[3] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, PgUp_33_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, Tab_34_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, Q_35_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, W_36_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, E_37_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, R_38_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, T_39_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Y_40_Pin) == GPIO_PIN_RESET) keyboard_report[4] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, U_41_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, I_42_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, O_43_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, P_44_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, Key__45_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, Key__46_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, Key__47_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Delete_48_Pin) == GPIO_PIN_RESET) keyboard_report[5] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, End_49_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, PgDn_50_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, Caps_51_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, A_52_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, S_53_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, D_54_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, F_55_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, G_56_Pin) == GPIO_PIN_RESET) keyboard_report[6] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, H_57_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, J_58_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, K_59_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, L_60_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, Key__61_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, Key__62_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, Enter_63_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, LShift_64_Pin) == GPIO_PIN_RESET) keyboard_report[7] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, Z_65_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, X_66_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, C_67_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, V_68_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, B_69_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, N_70_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, M_71_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Key__72_Pin) == GPIO_PIN_RESET) keyboard_report[8] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, Key__73_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, Key__74_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, RShift_75_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, Up_76_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, LCtrl_77_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, Win_78_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, LAlt_79_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 6);
    if(HAL_GPIO_ReadPin(GPIOE, Space__80_Pin) == GPIO_PIN_RESET) keyboard_report[9] |= (1 << 7);
    if(HAL_GPIO_ReadPin(GPIOE, RAlt_81_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 0);
    if(HAL_GPIO_ReadPin(GPIOE, Menu_82_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 1);
    if(HAL_GPIO_ReadPin(GPIOE, Fn_83_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 2);
    if(HAL_GPIO_ReadPin(GPIOE, RCtrl_84_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 3);
    if(HAL_GPIO_ReadPin(GPIOE, Left_85_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 4);
    if(HAL_GPIO_ReadPin(GPIOE, Down_86_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 5);
    if(HAL_GPIO_ReadPin(GPIOE, Right_87_Pin) == GPIO_PIN_RESET) keyboard_report[10] |= (1 << 6);

    // made with **help** of chatgpt and notebooklm too, help = he explained how to do it, i made the code, verified with him if it will work, and he confirmed or said it wont and said what to fix
    uint32_t color_grb;
    for(int i = 0; i < NUM_LEDS; i++) {
    	int byte = i / 8;
    	int bit = i % 8;
    	if(keyboard_report[byte] & (1 << bit)) {
    		// key pressed -> led under it goes to red
    		color_grb = (0x00 << 16) | (0xFF << 8) | 0x00;
    	} else {
    		// other leds -> white, 70% of white as per producer words
    		color_grb = (178 << 16) | (178 << 8) | 178;
    	}


		for(int b = 0; b < BITS_PER_LED; b++) {
			if(color_grb & (1 << (23 - b))) {
				led_buffer[i * BITS_PER_LED + b] = PWM_HI;
			} else {
				led_buffer[i * BITS_PER_LED + b] = PWM_LO;
			}
		}
    }

    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, led_buffer, NUM_LEDS * BITS_PER_LED);

    //send report
    USBD_HID_SendReport(&hUsbDeviceFS, keyboard_report, 11);

    //delay
    HAL_Delay(4);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = 47999;
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 179;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**UCPD1 GPIO Configuration
  PB4   ------> UCPD1_CC2
  PB6   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* UCPD1 DMA Init */

  /* UCPD1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_UCPD1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_UCPD1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* UCPD1 interrupt Init */
  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : ESC_1_Pin F1_2_Pin F2_3_Pin F3_4_Pin
                           F4_5_Pin Y_40_Pin U_41_Pin I_42_Pin
                           O_43_Pin P_44_Pin Key__45_Pin Key__46_Pin
                           Key__47_Pin Delete_48_Pin */
  GPIO_InitStruct.Pin = ESC_1_Pin|F1_2_Pin|F2_3_Pin|F3_4_Pin
                          |F4_5_Pin|Y_40_Pin|U_41_Pin|I_42_Pin
                          |O_43_Pin|P_44_Pin|Key__45_Pin|Key__46_Pin
                          |Key__47_Pin|Delete_48_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : F5_6_Pin F6_7_Pin F7_8_Pin Key1_18_Pin
                           Key2_19_Pin Key3_20_Pin Key4_21_Pin Backspace_30_Pin
                           Insert_31_Pin Enter_63_Pin LShift_64_Pin N_70_Pin
                           M_71_Pin LAlt_79_Pin Space_80_Pin RAlt_81_Pin */
  GPIO_InitStruct.Pin = F5_6_Pin|F6_7_Pin|F7_8_Pin|Key1_18_Pin
                          |Key2_19_Pin|Key3_20_Pin|Key4_21_Pin|Backspace_30_Pin
                          |Insert_31_Pin|Enter_63_Pin|LShift_64_Pin|N_70_Pin
                          |M_71_Pin|LAlt_79_Pin|Space_80_Pin|RAlt_81_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : F8_9_Pin F9_10_Pin F10_11_Pin F11_12_Pin
                           F12_13_Pin PrtSc_14_Pin ScrLk_15_Pin Pause_16_Pin
                           Grave_17_Pin Key5_22_Pin Q_35_Pin W_36_Pin
                           E_37_Pin R_38_Pin T_39_Pin Up_76_Pin */
  GPIO_InitStruct.Pin = F8_9_Pin|F9_10_Pin|F10_11_Pin|F11_12_Pin
                          |F12_13_Pin|PrtSc_14_Pin|ScrLk_15_Pin|Pause_16_Pin
                          |Grave_17_Pin|Key5_22_Pin|Q_35_Pin|W_36_Pin
                          |E_37_Pin|R_38_Pin|T_39_Pin|Up_76_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : Key6_23_Pin Key7_24_Pin Key8_25_Pin Key9_26_Pin
                           Key0_27_Pin Minus_28_Pin Plus_29_Pin Key__72_Pin
                           Key__73_Pin Key__74_Pin RShift_75_Pin LCtrl_77_Pin
                           Win_78_Pin */
  GPIO_InitStruct.Pin = Key6_23_Pin|Key7_24_Pin|Key8_25_Pin|Key9_26_Pin
                          |Key0_27_Pin|Minus_28_Pin|Plus_29_Pin|Key__72_Pin
                          |Key__73_Pin|Key__74_Pin|RShift_75_Pin|LCtrl_77_Pin
                          |Win_78_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Home_32_Pin PgUp_33_Pin Tab_34_Pin End_49_Pin
                           PgDn_50_Pin Caps_51_Pin A_52_Pin S_53_Pin
                           D_54_Pin */
  GPIO_InitStruct.Pin = Home_32_Pin|PgUp_33_Pin|Tab_34_Pin|End_49_Pin
                          |PgDn_50_Pin|Caps_51_Pin|A_52_Pin|S_53_Pin
                          |D_54_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : F_55_Pin G_56_Pin H_57_Pin J_58_Pin
                           K_59_Pin L_60_Pin Key__61_Pin Key__62_Pin
                           Menu_82_Pin Fn_83_Pin RCtrl_84_Pin Left_85_Pin
                           Down_86_Pin Right_87_Pin */
  GPIO_InitStruct.Pin = F_55_Pin|G_56_Pin|H_57_Pin|J_58_Pin
                          |K_59_Pin|L_60_Pin|Key__61_Pin|Key__62_Pin
                          |Menu_82_Pin|Fn_83_Pin|RCtrl_84_Pin|Left_85_Pin
                          |Down_86_Pin|Right_87_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Z_65_Pin X_66_Pin C_67_Pin V_68_Pin
                           B_69_Pin */
  GPIO_InitStruct.Pin = Z_65_Pin|X_66_Pin|C_67_Pin|V_68_Pin
                          |B_69_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
