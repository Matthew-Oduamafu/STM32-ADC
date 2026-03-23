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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <inttypes.h>
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

// Create a pointer to the calibration register:
volatile uint16_t *VREFINT_CAL = (volatile uint16_t *)0x1FFF7A2AUL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef ReadADC(uint32_t *result, uint32_t readings_to_average);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	// Calculate the voltage of the internal reference based on
	// the calibration register value and the Vdda voltage with which
	// this calibration register was measured at the factory.
	float vrefint_voltage_calibrated = ((*VREFINT_CAL) * 3.0 / 4095);

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

   // Run the ADC self-calibration routine. If it fails, print
   //  an error message and call Error_Handler().
//   if (HAL_ADC(&hadc1) != HAL_OK) {
//	   printf("ADC Calibration failed.\r\n");
//       Error_Handler();
//   }
//
//   // Wait for the calibration to complete:
//   HAL_Delay(100);

   ADC_ChannelConfTypeDef sConfig = {0};

   uint32_t vrefint_raw_reading = 0;

   // Configure ADC! to measure the Vrefint channel.
   // If the HAL function fails, print an error message and call Error_Handler().
   sConfig.Channel = ADC_CHANNEL_VREFINT;
   sConfig.Rank = 1;
   sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
   sConfig.Offset = 0;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
	 printf("Failed to select Vrefint channel!\r\n");
	 Error_Handler();
   }

   // Wait for ADC Channel Switching:
   HAL_Delay(100);

   // Read the VREFINT channel 20 times and average the result:
   if (ReadADC(&vrefint_raw_reading, 20) != HAL_OK) {
	 printf("ReadADC failed!\r\n");
	 Error_Handler();
   }

   // Use your Vrefint measurement and calculated Vrefint voltage to determine the actual
   // Vdda voltage of your specific board:
   float vdda_voltage_calibrated = (vrefint_voltage_calibrated * 4095.0) / vrefint_raw_reading;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   // Configure the ADC1 to measure the LDR channel.
   // If the HAL function fails, print an error message and call Error_Handler().

   uint32_t ldr_raw_reading = 0;

   sConfig.Channel = ADC_CHANNEL_1;
   sConfig.Rank = 1;
   sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
   sConfig.Offset = 0;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
	 printf("Failed to select potentiometer channel!\r\n");
	 Error_Handler();
   }

   // Wait for ADC Channel Switching:
   HAL_Delay(100);

   printf("ADC Calibration Result Comparison\r\n\r\n");

  while (1)
  {
	  // Read the LDR channel 20 times and average the result:
	  if (ReadADC(&ldr_raw_reading, 20) != HAL_OK) {
		 printf("ReadADC failed!\r\n");
		 Error_Handler();
	   }
	  float ldr_voltage_calibrated = (ldr_raw_reading * vdda_voltage_calibrated) / 4095;
	  float ldr_voltage_uncalibrated = (ldr_raw_reading * 3.3) / 4095;

	  // LDR reading converted to voltage assuming Vdda = 3.3V
	  printf("LDR uncalibrated voltage:     %.6fV\r\n", ldr_voltage_uncalibrated);
	  // LDR reading converted to voltage using measured Vdda
	  printf("LDR calibrated voltage:       %.6fV\r\n\r\n", ldr_voltage_calibrated);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Read ADC multiple times and return the average reading
 *
 * @param result pointer to variable where the average of the readings should be stored
 * @param readings_to_average number of ADC readings to perform and average.
 * @return HAL_OK if successful, otherwise the error code returned by the failing HAL function.
 */
static HAL_StatusTypeDef ReadADC(uint32_t *result, uint32_t readings_to_average) {

  // ====== STEP 3: ADC Measurements =======

  HAL_StatusTypeDef err = HAL_OK;

  // Prevent a division-by-zero:
  if (readings_to_average == 0) {
    return HAL_ERROR;
  }

  // Sum of all readings:
  uint64_t sum = 0;

  for (size_t i = 0; i < readings_to_average; i++) {
    // Start ADC conversion. If it fails, print an error message and
    // return the error code it returned.
    err = HAL_ADC_Start(&hadc1);
    if (err != HAL_OK) {
      printf("Failed to start ADC measurement!\r\n");
      return err;
    }

    // Wait for ADC conversion to finish. If it fails,
    // print an error message and return the error code it returned.
    err = HAL_ADC_PollForConversion(&hadc1, 1000);
    if (err != HAL_OK) {
      printf("Failed to wait for ADC measurement!\r\n");
      return err;
    }

    // Retrieve the conversion result and add it to the sum:
    sum += HAL_ADC_GetValue(&hadc1);
  }

  // Calculate the average of all readings:
  *result = sum / readings_to_average;

  // Return OK:
  return HAL_OK;
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
