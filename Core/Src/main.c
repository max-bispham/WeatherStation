/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306.h"
#include "stdio.h"
#include "ssd1306_fonts.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Temperature
static const uint8_t SHT30_ADDR = 0x44 << 1;
static const uint8_t REG_TEMP1 = 0x2c;
static const uint8_t REG_TEMP2 = 0x10;

//Magonetometer
static const uint8_t MMC_ADDR = 0x30 << 1;
static uint8_t reg = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int32_t convert_to_20bit_integer(uint32_t val) {
	if (val & 0x800000) {
		return val - 0x100000;
	}
	return val;
}
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
	// Temperature and Humidity
	HAL_StatusTypeDef ret;
	uint8_t buf[8];
	uint16_t raw_temp;
	uint16_t raw_hum;
	float temperature;
	float humidity;
	char temp[50];
	char hum[50];

	// Magnetic Field Sensor
	uint8_t set[2] = {0x1B, 0x08};
	uint8_t cont[2] = {0x1A, 0x1C};
	uint8_t status = 0x18;
	uint8_t Cmm_freq_en[2] = {0x1B, 0x80};
	uint8_t Cmm_en[2] = {0x1D, 0x10};

	uint8_t pos[9];
	uint32_t raw_xvec;
	uint32_t raw_yvec;
	uint32_t raw_zvec;
	double xvec;
	double yvec;
	double zvec;

	uint8_t ctrl_0 = 0x1B;
	uint8_t res;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, set, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
  	  printf("Error setting mmc\n");
    } else {
  	  printf("mmc set\n");
  	  HAL_Delay(10);
  	  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, cont, 2, HAL_MAX_DELAY);
  	  if ( ret != HAL_OK ) {
  		  printf("Error setting continuous mode\n");
  	  } else {
  		  printf("Continuous mode set\n");
  		  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, Cmm_freq_en, 2, HAL_MAX_DELAY);
  		  if ( ret != HAL_OK ) {
  			  printf("Error setting Cmm_freq_en\n");
  		  } else {
  			  printf("CMM_freq_en set\n");
  			  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, Cmm_en, 2, HAL_MAX_DELAY);
  			  if ( ret != HAL_OK ) {
  				  printf("Error setting Cmm_en\n");
  			  } else {
  				  printf("Cmm_en set\n");
  			  }
  		  }
  	  }
    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  HAL_Delay(100);
	  buf[0] = REG_TEMP1;
	  buf[1] = REG_TEMP2;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, SHT30_ADDR, buf, 2, HAL_MAX_DELAY);
	  if (ret != HAL_OK ) {
//		  printf("Error Sending Temperature\n");
	  } else {
		  ret = HAL_I2C_Master_Receive(&hi2c1, SHT30_ADDR, buf, 6, HAL_MAX_DELAY);
		  if (ret != HAL_OK ) {
//			  printf("Error Receiving Temperature\n");
		  } else {
			  raw_temp = ((uint16_t)buf[0] << 8) | buf[1];
			  raw_hum = ((uint16_t)buf[3] << 8) | (uint16_t)buf[4];
			  temperature = -45 + 175 * (((float)raw_temp)/65535);
			  humidity = 100 * (float)raw_hum/65535;
			  sprintf(temp, "Temperature: %.2f F\n", temperature);
			  sprintf(hum, "Humidity: %.2f %%\n", humidity);
//			  printf(temp);
//			  printf(hum);

			  ssd1306_SetCursor(5, 5);
			  ssd1306_WriteString(temp, Font_7x10, White);
			  ssd1306_SetCursor(5, 20);
			  ssd1306_WriteString(hum, Font_7x10, White);
			  ssd1306_UpdateScreen();
		  	  }
		 }


	  ret = HAL_I2C_Master_Receive(&hi2c1, MMC_ADDR, &status, 1, HAL_MAX_DELAY);
	  if ( ret != HAL_OK ){
		  printf("Error reading status\n");
	  } else {
	  if ( (status & 0x40) == 0x40 ) {
		  printf("Data is ready!\n");
		 ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, &reg, 1, HAL_MAX_DELAY);
		 if (ret != HAL_OK) {
			 printf("Error Sending Magnet\n");
		 } else {
			 ret = HAL_I2C_Master_Receive(&hi2c1, MMC_ADDR, pos, 9, HAL_MAX_DELAY);
			 if (ret != HAL_OK) {
				 printf("Error Receiving Magnet\n");
			 } else {

				 raw_xvec = ((uint32_t)pos[0] << 12) | ((uint32_t)pos[1] << 4) | (((uint32_t)pos[6] >> 4) & 0x0F);
				 raw_yvec = ((uint32_t)pos[2] << 12) | ((uint32_t)pos[3] << 4) | (((uint32_t)pos[7] >> 4) & 0x0F);
				 raw_zvec = ((uint32_t)pos[4] << 12) | ((uint32_t)pos[5] << 4) | (((uint32_t)pos[8] >> 4) & 0x0F);

				 xvec = (double)convert_to_20bit_integer(raw_xvec);
				 yvec = (double)convert_to_20bit_integer(raw_yvec);
				 zvec = (double)convert_to_20bit_integer(raw_zvec);

				 printf("X: %.2f\n", xvec);
				 printf("Y: %.2f\n", yvec);
				 printf("Z: %.2f\n", zvec);
		  }

			}
		  }
	  }

  }
}

    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;

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
