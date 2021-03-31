/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char global_buffer[100]; //buffer for usart transmission
int16_t accelero_values[3]; //temp array for accelerometer values
int16_t old_accelero_values[3]; //keep track of last accelero values for calculating change
int current_note = 0; //for keeping track of which note is playing
int current_volume = 4; //for keeping track of volume level, 5 levels of volume from 0 to 4

int timer_counter = 0;

//sample arrays for pitches
uint8_t C6_samples[43];
uint8_t D6_samples[36];
uint8_t E6_samples[33];
uint8_t F6_samples[30];
uint8_t G6_samples[27];
uint8_t A6_samples[24];
uint8_t B6_samples[21];
uint8_t C7_samples[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// populate the sine wave sampling array for a C6 tone
void C6_sample_populator() {
	for (int i = 0; i < 43; i++) {
		float modulus = (float) i / 43;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		C6_samples[i] = (uint8_t) radians;
	}
}

// populate the sine wave sampling array for a D6 tone
void D6_sample_populator() {
	for (int i = 0; i < 36; i++) {
		float modulus = (float) i / 36;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		D6_samples[i] = (uint8_t) radians;
	}
}

// populate the sine wave sampling array for an E6 tone
void E6_sample_populator() {
	for (int i = 0; i < 33; i++) {
		float modulus = (float) i / 33;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1) * 85;
		E6_samples[i] = (uint8_t) radians;
	}
}

// populate the sine wave sampling array for an F6 tone
void F6_sample_populator() {
	for (int i = 0; i < 30; i++) {
		float modulus = (float) i / 30;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1) * 85;
		F6_samples[i] = (uint8_t) radians;
	}
}

// populate the sine wave sampling array for a G6 tone
void G6_sample_populator() {
	for (int i = 0; i < 27; i++) {
		float modulus = (float) i / 27;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1) * 85;
		G6_samples[i] = (uint8_t) radians;
	}

}

// populate the sine wave sampling array for an A6 tone
void A6_sample_populator() {
	for (int i = 0; i < 24; i++) {
		float modulus = (float) i / 24;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1) * 85;
		A6_samples[i] = (uint8_t) radians;
	}
}

// populate the sine wave sampling array for an B6 tone
void B6_sample_populator() {
	for (int i = 0; i < 21; i++) {
		float modulus = (float) i / 21;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1) * 85;
		B6_samples[i] = (uint8_t) radians;
	}
}

// populate the sine wave sampling array for an C7 tone
void C7_sample_populator() {
	for (int i = 0; i < 20; i++) {
		float modulus = (float) i / 20;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1) * 85;
		C7_samples[i] = (uint8_t) radians;
	}
}

void pitch_volume_changer(int16_t old_accelero_values[3],
		int16_t new_accelero_values[3]) {
	int16_t difference_values[3];
	difference_values[0] = new_accelero_values[0] - old_accelero_values[0];
	difference_values[1] = new_accelero_values[1] - old_accelero_values[1];
	difference_values[2] = new_accelero_values[2] - old_accelero_values[2];

	if ((difference_values[0] > 30) || (difference_values[0] < -30)) {
		if ((difference_values[0] > 0) && (current_note > 0) && (difference_values[0] < 100)) {
			current_note -= 1;
		} else if ((difference_values[0] < 0) && (current_note < 7) && (difference_values[0] > -100)) {
			current_note += 1;
		}
	}
	if (abs(difference_values[1] > 12)) {
		if ((difference_values[1] < 0) && (current_volume > 0)) {
			current_volume -= 1;
		} else if ((difference_values[1] > 0) && (current_volume < 4)) {
			current_volume += 1;
		}
	}
}

void pitch_volume_setter() {
	uint8_t *pitch_pointer;
	int volume;
	int sample_size;
	switch (current_note) {
	case 0:
		pitch_pointer = C6_samples;
		sample_size = 43;
		break;
	case 1:
		pitch_pointer = D6_samples;
		sample_size = 36;
		break;
	case 2:
		pitch_pointer = E6_samples;
		sample_size = 33;
		break;
	case 3:
		pitch_pointer = F6_samples;
		sample_size = 30;
		break;
	case 4:
		pitch_pointer = G6_samples;
		sample_size = 27;
		break;
	case 5:
		pitch_pointer = A6_samples;
		sample_size = 24;
		break;
	case 6:
		pitch_pointer = B6_samples;
		sample_size = 21;
		break;
	case 7:
		pitch_pointer = C7_samples;
		sample_size = 20;
		break;

	}
	switch (current_volume) {
	case 0:
		volume = 0;
		break;
	case 1:
		volume = 22;
		break;
	case 2:
		volume = 43;
		break;
	case 3:
		volume = 64;
		break;
	case 4:
		volume = 85;
		break;
	}

	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, pitch_pointer, sample_size,
	DAC_ALIGN_8B_R);
}

/*
// basic interrupt for C6 tone with no DMA
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		timer_countounter % 44;
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R,
				C6_samples[timer_counter]er = timer_c * 85);
		timer_counter++;

	}
}
*/

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DAC1_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	C6_sample_populator();
	D6_sample_populator();
	E6_sample_populator();
	F6_sample_populator();
	G6_sample_populator();
	A6_sample_populator();
	B6_sample_populator();
	C7_sample_populator();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	BSP_ACCELERO_AccGetXYZ(accelero_values);
	memset(global_buffer, 0, sizeof(global_buffer));
	// first value is side to side (long side), second value is up-down, third value is front-back (short side)
	sprintf(global_buffer, "Accelerometer values are %d, %d, %d Current note: %d",
			(int) accelero_values[0], (int) accelero_values[1],
			(int) accelero_values[2], current_note);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, C6_samples, 43,
		DAC_ALIGN_8B_R);

	HAL_UART_Transmit(&huart1, (uint8_t*) global_buffer, sizeof(global_buffer),
			1000);
	old_accelero_values[0] = accelero_values[0];
	old_accelero_values[1] = accelero_values[1];
	old_accelero_values[2] = accelero_values[2];
	HAL_Delay(1000);

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		BSP_ACCELERO_AccGetXYZ(accelero_values);
		memset(global_buffer, 0, sizeof(global_buffer));
		// first value is side to side (long side), second value is up-down, third value is front-back (short side)
		sprintf(global_buffer, "Accelerometer values are %d, %d, %d Current note: %d",
				(int) accelero_values[0], (int) accelero_values[1],
				(int) accelero_values[2], current_note);
		pitch_volume_changer(old_accelero_values, accelero_values);
		pitch_volume_setter();

		HAL_UART_Transmit(&huart1, (uint8_t*) global_buffer,
				sizeof(global_buffer), 1000);
		old_accelero_values[0] = accelero_values[0];
		old_accelero_values[1] = accelero_values[1];
		old_accelero_values[2] = accelero_values[2];
		HAL_Delay(1000);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 60;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */
	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x307075B1;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1814;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
