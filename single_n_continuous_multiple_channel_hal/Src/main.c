/* USER CODE BEGIN Header */
/**
 * ADC multi-channel — HAL single mode and continuous mode
 * Channels: IN1 (PA1, LDR)  |  IN4 (PA4, Potentiometer)
 *
 * printf is routed via semihosting or UART depending on your
 * project setup. If you see no output, enable semihosting in
 * your debug configuration or retarget printf to UART.
 */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

#define ADC_NUM_CHANNELS  2

/* Single mode results */
uint32_t single_results[ADC_NUM_CHANNELS];

/* Continuous mode results */
uint32_t cont_results[ADC_NUM_CHANNELS];
volatile uint8_t cont_channel_idx;
volatile uint8_t cont_done;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN 0 */

static uint8_t raw_to_percent(uint32_t raw)
{
    return (uint8_t)((raw * 100UL) / 4095UL);
}

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_ADC1_Init();

    /* USER CODE BEGIN 2 */

    /* ── PART 1: Single mode — run once before the main loop ────
     *
     * Trigger one full sequence, read both channels, print, stop.
     * ───────────────────────────────────────────────────────── */

    printf("=== PART 1: Single mode (one scan) ===\r\n");

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++)
    {
        if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
        {
            Error_Handler();
        }
        single_results[ch] = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    printf("LDR  (PA1 CH1): %4lu raw  %3u%%\r\n",
           single_results[0],
           raw_to_percent(single_results[0]));

    printf("Pot  (PA4 CH4): %4lu raw  %3u%%\r\n",
           single_results[1],
           raw_to_percent(single_results[1]));

    /* ── PART 2: Continuous mode — collect one set then stop ────
     *
     * Reconfigure hadc1 for continuous mode at runtime.
     * HAL_ADC_Start_IT() starts the ADC and enables the EOC
     * interrupt. HAL_ADC_ConvCpltCallback() fires after each
     * channel and fills cont_results[]. We wait for cont_done
     * then stop and print.
     *
     * Requires ADC1 global interrupt enabled in NVIC Settings
     * (CubeMX) or manually via HAL_NVIC_EnableIRQ(ADC_IRQn).
     * ───────────────────────────────────────────────────────── */

    printf("\r\n=== PART 2: Continuous mode (one set collected) ===\r\n");

    hadc1.Init.ContinuousConvMode = ENABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    cont_channel_idx = 0;
    cont_done        = 0;

    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    while (!cont_done) { }

    HAL_ADC_Stop_IT(&hadc1);

    printf("LDR  (PA1 CH1): %4lu raw  %3u%%\r\n",
           cont_results[0],
           raw_to_percent(cont_results[0]));

    printf("Pot  (PA4 CH4): %4lu raw  %3u%%\r\n",
           cont_results[1],
           raw_to_percent(cont_results[1]));

    /* Switch back to single mode for the main loop */
    hadc1.Init.ContinuousConvMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */

    printf("\r\n=== PART 3: Single mode repeating in main loop ===\r\n");

    uint32_t count = 0;

    while (1)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }

        for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++)
        {
            if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
            {
                Error_Handler();
            }
            single_results[ch] = HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);

        printf("[%lu] LDR (PA1): %4lu raw %3u%%  |  Pot (PA4): %4lu raw %3u%%\r\n",
               count,
               single_results[0], raw_to_percent(single_results[0]),
               single_results[1], raw_to_percent(single_results[1]));

        count++;
        HAL_Delay(500);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
 * HAL_ADC_ConvCpltCallback
 *
 * Fires after every single-channel EOC during continuous mode
 * (HAL_ADC_Start_IT). Reads one channel per call, tracks position
 * with cont_channel_idx, sets cont_done when both channels read.
 *
 * Do NOT call HAL_ADC_Stop_IT() here — stop from main() after
 * seeing cont_done to avoid HAL state conflicts inside the ISR.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        if (cont_channel_idx < ADC_NUM_CHANNELS)
        {
            cont_results[cont_channel_idx] = HAL_ADC_GetValue(hadc);
            cont_channel_idx++;
        }

        if (cont_channel_idx >= ADC_NUM_CHANNELS)
        {
            cont_done = 1;
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
    /* USER CODE BEGIN ADC1_Init 0 */
    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */
    /* USER CODE END ADC1_Init 1 */

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank    = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN ADC1_Init 2 */

    /*
     * Enable ADC1 interrupt in NVIC — required for
     * HAL_ADC_Start_IT() used in continuous mode (Part 2).
     * If you enabled this in CubeMX NVIC Settings, remove these
     * two lines to avoid a duplicate enable.
     */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) { }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif
