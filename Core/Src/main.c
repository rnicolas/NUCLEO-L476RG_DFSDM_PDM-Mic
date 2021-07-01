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

#include "dfsdm.h"
#include "dma.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */
bool flag = true;
int32_t Buff[FFT_SampleNum * 2] = {0};

arm_rfft_fast_instance_f32 S;
float FFT_SampleRate;
int32_t FFT_inp_int32[FFT_SampleNum] = {0};
float FFT_inp[FFT_SampleNum] = {0.0f};
float FFT_oup[FFT_SampleNum] = {0.0f};
float FFT_mag[FFT_SampleNum / 2] = {0.0f};
float FFT_dB[FFT_SampleNum / 2] = {0.0f};
float FFT_frq[FFT_SampleNum / 2] = {0.0f};
float FFT_window[FFT_SampleNum] = {0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART2_UART_Init();
	MX_DFSDM1_Init();
	/* USER CODE BEGIN 2 */
	printf("\r\n***** Program start! *****\r\n");
	HAL_Delay(100);
	if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, Buff, FFT_SampleNum) != HAL_OK) {
		Error_Handler();
	}

	// FFT init
	FFT_SampleRate = SystemCoreClock / hdfsdm1_channel0.Init.OutputClock.Divider /
					 hdfsdm1_filter0.Init.FilterParam.Oversampling /
					 hdfsdm1_filter0.Init.FilterParam.IntOversampling;

	// Hanning window
	const float tmp = 2.0f * M_PI / (float)FFT_SampleNum;
	for (uint32_t i = 0; i < FFT_SampleNum; i++)
		*(FFT_window + i) = 0.5f - 0.5f * arm_cos_f32((float)i * tmp);

	for (uint32_t i = 0; i < FFT_SampleNum / 2; i++)
		*(FFT_frq + i) = (float)i * (float)FFT_SampleRate / (float)FFT_SampleNum;

	arm_rfft_fast_init_f32(&S, FFT_SampleNum);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// Wait
		while (flag)
			;

		// Raw data output
		// for (uint32_t i = 0; i < FFT_SampleNum; i++) printf("%d\r\n", FFT_inp_int32[i]);

		// Set input data
		for (uint32_t i = 0; i < FFT_SampleNum; i++) FFT_inp[i] = (float)FFT_inp_int32[i];

		// Windowing
		arm_mult_f32(FFT_inp, FFT_window, FFT_inp, FFT_SampleNum);

		// Execute FFT
		arm_rfft_fast_f32(&S, FFT_inp, FFT_oup, 0);

		// calculate magnitude
		arm_cmplx_mag_f32(FFT_oup, FFT_mag, FFT_SampleNum / 2);

		// Normalization (Unitary transformation) of magnitude
		arm_scale_f32(FFT_mag, 1.0f / sqrtf((float)FFT_SampleNum), FFT_mag, FFT_SampleNum / 2);

		// AC coupling
		for (uint32_t i = 0; i < FFT_SampleNum / 2; i++) {
			if (*(FFT_frq + i) < FFT_AC_COUPLING_HZ)
				FFT_mag[i] = 1.0f;
			else
				break;
		}

		float inv_dB_base_mag = 1.0f / 1.0f;
		for (uint32_t i = 0; i < FFT_SampleNum / 2; i++)
			FFT_dB[i] = 10.0f * log10f(FFT_mag[i] * inv_dB_base_mag);

		// calc max mag
		// float mag_max, frq_max;
		// uint32_t maxIndex;
		// arm_max_f32(FFT_mag, FFT_SampleNum / 2, &mag_max, &maxIndex);
		// frq_max = *(FFT_frq + maxIndex);

		// printf("SampleRate=%d, frq_max = %.1f, mag_max = %f\r\n", (int)FFT_SampleRate, frq_max,
		// mag_max);
		for (uint32_t i = 0; i < FFT_SampleNum / 2; i++) {
			printf("%.1f  %f  %f\r\n", FFT_frq[i], FFT_mag[i], FFT_dB[i]);
		}

		printf("=====\r\n");

		// while(1);
		// HAL_Delay(2000);
		flag = true;  // <- Continuous transformation
					  /* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType =
		RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_DFSDM1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Half regular conversion complete callback.
 * @param  hdfsdm_filter : DFSDM filter handle.
 * @retval None
 */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	/*static int i = 0;
	printf("%d\r\n", i);
	i++;*/
}

/**
  * @brief  Regular conversion complete callback.
  * @note   In interrupt mode, user has to read conversion value in this function
			using HAL_DFSDM_FilterGetRegularValue.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	if (flag) {
		for (uint32_t i = 0; i < FFT_SampleNum; i++) { FFT_inp_int32[i] = Buff[i]; }
		flag = false;
	}
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 0xFFFFFFFF);
	return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {}
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
