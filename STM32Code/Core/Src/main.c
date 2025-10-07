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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
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

/* Definitions for Blink01 */
osThreadId_t Blink01Handle;
const osThreadAttr_t Blink01_attributes = {
  .name = "Blink01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Blink02 */
osThreadId_t Blink02Handle;
const osThreadAttr_t Blink02_attributes = {
  .name = "Blink02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for I2CRead */
osThreadId_t I2CReadHandle;
const osThreadAttr_t I2CRead_attributes = {
  .name = "I2CRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
static const uint8_t MMC_ADDR = 0x30 << 1; // 8 bit address
static const uint8_t ODR = 0x1A;
static const uint8_t internalC0 = 0x1B;
static const uint8_t internalC2 = 0x1D;
static const int i2c_delay = 500;
double x_offset = 0;
double y_offset = 0;
double z_offset = 0;
double vec[3];
double *ve;
typedef struct {
	uint8_t bytes[9];
} MagVector_t;

//FreeRTOS Queue Declaration
QueueHandle_t xQueue1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartBlink01(void *argument);
void StartBlink02(void *argument);
void StartI2CRead(void *argument);
/* USER CODE BEGIN PFP */
double convert_to_20bit_double(uint32_t val) {
	int32_t new = (int32_t)val - 524288;
	return (double)new * 0.00625;
}

void set_magnetometer() {
	HAL_StatusTypeDef ret;
	uint8_t set[2] = {internalC0, 0x08};
	uint8_t continuous_rate[2] = {ODR, 0x1C};
	uint8_t Cmm_freq_en[2] = {internalC0, 0x80};
	uint8_t Cmm_en[2] = {internalC2, 0x10};
	uint8_t buf[50];


	ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, set, 2, i2c_delay);

	      if (ret != HAL_OK) {
	    	  strcpy((char*)buf, "Error setting mmc\r\n");
	      } else

	      {
	    	  strcpy((char*)buf, "mmc set\r\n");
	    	  HAL_Delay(10);

	    	  //set the measurement rate of the sensor
	    	  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, continuous_rate, 2, i2c_delay);
	    	  if ( ret != HAL_OK ) {
	    		  strcpy((char*)buf, "Error setting continuous mode\n");
	    	  } else

	    	  {
	    		  strcpy((char*)buf, "Continuous mode set\n");

	    		  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, Cmm_freq_en, 2, i2c_delay);
	    		  if ( ret != HAL_OK ) {
	    			  strcpy((char*)buf, "Error setting Cmm_freq_en\n");
	    		  } else {
	    			  strcpy((char*)buf, "CMM_freq_en set\n");
	    			  ret = HAL_I2C_Master_Transmit(&hi2c1, MMC_ADDR, Cmm_en, 2, i2c_delay);
	    			  if ( ret != HAL_OK ) {
	    				  strcpy((char*)buf, "Error setting Cmm_en\n");
	    			  } else {
	    				  strcpy((char*)buf, "Cmm_en set\n");
	    			  }
	    		  }
	    	  }
	      }

}

void read_magnetometer(uint8_t *pos) {
	HAL_StatusTypeDef ret;
	uint8_t out = 0x00;
	char bufc[20];
	ret = HAL_I2C_Mem_Read(&hi2c1, MMC_ADDR, out, I2C_MEMADD_SIZE_8BIT, pos, 9, 500);

	if (ret == HAL_OK)
	  {
		return;

	  } else
	  {
		  strcpy((char*)bufc, "Error Read\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*)bufc, strlen(bufc), i2c_delay);
	  }
}

void raw_to_real(uint8_t *pos, double *out) {
	uint32_t raw_xvec;
	uint32_t raw_yvec;
	uint32_t raw_zvec;

	raw_xvec = ((uint32_t)pos[0] << 12) | ((uint32_t)pos[1] << 4) | ((pos[6] >> 4));
	raw_yvec = ((uint32_t)pos[2] << 12) | ((uint32_t)pos[3] << 4) | ((pos[7] >> 4));
	raw_zvec = ((uint32_t)pos[4] << 12) | ((uint32_t)pos[5] << 4) | ((pos[8] >> 4));

	out[0] = convert_to_20bit_double(raw_xvec) - x_offset;
	out[1] = convert_to_20bit_double(raw_yvec) - y_offset;
	out[2] = convert_to_20bit_double(raw_zvec) - z_offset;
}


void display_vectors(double *vector)
{
	char buf[100];
	snprintf(buf, sizeof(buf), "X = %0.2f, Y = %.2f, Z = %.2f\r\n", vector[0], vector[1], vector[2]);
	HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), i2c_delay);
}

void calibrate_sensor_offset()
{
	uint8_t raw_vec[3];
	double temp_vec[3];
	set_magnetometer();
	read_magnetometer(raw_vec);
	raw_to_real(raw_vec, temp_vec);
	double max_x = temp_vec[0];
	double max_y = temp_vec[1];
	double max_z = temp_vec[2];
	double min_x = temp_vec[0];
	double min_y = temp_vec[1];
	double min_z = temp_vec[2];

	// Polling for max and min values
	for (int i = 0; i < 200; i ++) {
		read_magnetometer(raw_vec);
		raw_to_real(raw_vec, temp_vec);
		if (temp_vec[0] > max_x) {
			max_x = temp_vec[0];
		} else if (temp_vec[0] < min_x){
			min_x = temp_vec[0];
		}
		if (temp_vec[1] > max_y) {
					max_y = temp_vec[1];
		} else if (temp_vec[1] < min_y){
			min_y = temp_vec[1];
		}
		if (temp_vec[2] > max_z) {
			max_z = temp_vec[2];
		} else if (temp_vec[2] < min_z){
			min_z = temp_vec[2];
		}
		HAL_Delay(10);
	}

	// calculating offsets
		x_offset = (min_x + max_x)/2;
		y_offset = (min_y + max_y)/2;
		z_offset = (min_z + max_z)/2;
		// snprintf(buf, sizeof(buf), "offsetx: %0.2f, offsety: %0.2f, offsetz: %0.2f", x_offset, y_offset, z_offset);
		//HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
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
  set_magnetometer();
  calibrate_sensor_offset();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xQueue1 = xQueueCreate(3, 3*sizeof(MagVector_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Blink01 */
  Blink01Handle = osThreadNew(StartBlink01, NULL, &Blink01_attributes);

  /* creation of Blink02 */
  //Blink02Handle = osThreadNew(StartBlink02, NULL, &Blink02_attributes);

  /* creation of I2CRead */
  I2CReadHandle = osThreadNew(StartI2CRead, NULL, &I2CRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlink01 */
/**
  * @brief  Function implementing the Blink01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(500);
  }
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlink02 */
/**
* @brief Function implementing the Blink02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink02 */
void StartBlink02(void *argument)
{
  /* USER CODE BEGIN StartBlink02 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    osDelay(200);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartBlink02 */
}

/* USER CODE BEGIN Header_StartI2CRead */
/**
* @brief Function implementing the I2CRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2CRead */
void StartI2CRead(void *argument)
{
  /* USER CODE BEGIN StartI2CRead */

	double position[3];
  /* Infinite loop */
  for(;;)
  {
	  MagVector_t vec;
	  read_magnetometer(vec.bytes);
	  xQueueSend(xQueue1, &vec, 0);
	  // HAL_UART_Transmit(&huart2, (uint8_t*)"UART ready\r\n", 12, HAL_MAX_DELAY);

    osDelay(200);
  }
  /* USER CODE END StartI2CRead */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
