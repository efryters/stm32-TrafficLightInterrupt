
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void prvIntTest(void *p);
static void prvIntPedestrian(void *p);
static void prvTaskA(void *p);
static void prvTaskB(void *p);
static void prvDebTimer( TimerHandle_t xTimer );
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
TaskHandle_t taskA = NULL;
TaskHandle_t taskB = NULL;
TaskHandle_t isr_test = NULL;
TaskHandle_t isr_ped = NULL;
TimerHandle_t debounce_tim = NULL;
SemaphoreHandle_t isr = NULL;
SemaphoreHandle_t lock = NULL;
SemaphoreHandle_t s1 = NULL;
SemaphoreHandle_t s2 = NULL;
BaseType_t debounce_bool = pdFALSE;
BaseType_t in_task = pdFALSE;
BaseType_t ped_request = pdFALSE;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USB_OTG_FS_HCD_Init();
  /* USER CODE BEGIN 2 */
  xTaskCreate(prvIntPedestrian, "PedestrianBlinker", configMINIMAL_STACK_SIZE, NULL, 5, &isr_ped);
  xTaskCreate(prvIntTest, "InterruptTest", configMINIMAL_STACK_SIZE, NULL, 5, &isr_test);
  xTaskCreate(prvTaskA, "TaskA", configMINIMAL_STACK_SIZE, NULL, 2, &taskA);
  xTaskCreate(prvTaskB, "TaskB", configMINIMAL_STACK_SIZE, NULL, 2, &taskB);

  vTaskSuspend(isr_test);

  HAL_GPIO_WritePin(TS2_RED_GPIO_Port, TS2_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TS1_GRN_GPIO_Port, TS1_GRN_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  isr = xSemaphoreCreateBinary();
  lock = xSemaphoreCreateBinary();
  s1 = xSemaphoreCreateBinary();
  s2 = xSemaphoreCreateBinary();

  xSemaphoreGive(lock);
  xSemaphoreGive(s1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  debounce_tim = xTimerCreate("deb", pdMS_TO_TICKS(100), (UBaseType_t) pdFALSE, 0, prvDebTimer);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB_OTG_FS init function */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PED_WHT_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TS1_RED_Pin|TS2_RED_Pin|TS1_YEL_Pin|TS2_YEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TS1_GRN_Pin|TS2_GRN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin TS1_RED_Pin TS2_RED_Pin TS1_YEL_Pin 
                           TS2_YEL_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|TS1_RED_Pin|TS2_RED_Pin|TS1_YEL_Pin 
                          |TS2_YEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_BTN_Pin */
  GPIO_InitStruct.Pin = BLUE_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PED_WHT_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = PED_WHT_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TS1_GRN_Pin TS2_GRN_Pin */
  GPIO_InitStruct.Pin = TS1_GRN_Pin|TS2_GRN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

static void prvIntPedestrian(void *p)
{
	(void) p;

	while(1)
	{
		if(xSemaphoreTake(isr, portMAX_DELAY))
		{
			ped_request = pdTRUE;
		}
	}
}

static void prvIntTest(void *p)
{
	(void) p;

	while(1)
	{
		if(xSemaphoreTake(isr, portMAX_DELAY))
		{
			vTaskSuspend(taskA);
			vTaskSuspend(taskB);

			//flash LEDs
			HAL_GPIO_WritePin(TS2_YEL_GPIO_Port, TS2_YEL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TS1_YEL_GPIO_Port, TS1_YEL_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TS1_GRN_GPIO_Port, TS1_GRN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TS2_GRN_GPIO_Port, TS2_GRN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TS1_RED_GPIO_Port, TS1_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TS2_RED_GPIO_Port, TS2_RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(PED_WHT_GPIO_Port, PED_WHT_Pin, GPIO_PIN_SET);

			vTaskDelay(pdMS_TO_TICKS(500));
			HAL_GPIO_TogglePin(PED_WHT_GPIO_Port, PED_WHT_Pin);
			vTaskDelay(pdMS_TO_TICKS(500));
			HAL_GPIO_TogglePin(PED_WHT_GPIO_Port, PED_WHT_Pin);
			vTaskDelay(pdMS_TO_TICKS(500));
			HAL_GPIO_TogglePin(PED_WHT_GPIO_Port, PED_WHT_Pin);
			vTaskDelay(pdMS_TO_TICKS(500));
			HAL_GPIO_WritePin(PED_WHT_GPIO_Port, PED_WHT_Pin, GPIO_PIN_RESET);

			in_task = pdFALSE;
			vTaskResume(taskB);
			vTaskResume(taskA);
		}
	}
}

//Light set 1
static void prvTaskA(void *p)
{
	(void) p;

	while(1)
	{
		if(xSemaphoreTake(s1, portMAX_DELAY))
		{
			if(xSemaphoreTake(lock, portMAX_DELAY))
			{
				//run light set 1

				//grn on 4 seconds
				//yellow on 1.5 seconds
				//red on 1 second, then task 2
				HAL_GPIO_WritePin(TS1_RED_GPIO_Port, TS1_RED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS1_GRN_GPIO_Port, TS1_GRN_Pin, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(2000));
				if (ped_request == pdTRUE) 	// if pedetrian pushed, blink wht 2 seconds after
				{
					HAL_GPIO_WritePin(PED_WHT_GPIO_Port, PED_WHT_Pin, GPIO_PIN_SET);
					ped_request = pdFALSE;
					in_task = pdFALSE;
				}
				vTaskDelay(pdMS_TO_TICKS(2000));
				HAL_GPIO_WritePin(TS1_GRN_GPIO_Port, TS1_GRN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PED_WHT_GPIO_Port, PED_WHT_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS1_YEL_GPIO_Port, TS1_YEL_Pin, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(1500));
				HAL_GPIO_WritePin(TS1_YEL_GPIO_Port, TS1_YEL_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS1_RED_GPIO_Port, TS1_RED_Pin, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(1000));

				xSemaphoreGive(lock);
				xSemaphoreGive(s2);
			}
		}
	}
}

//Light set 2
static void prvTaskB(void *p)
{
	(void) p;

	while(1)
	{
		if(xSemaphoreTake(s2, portMAX_DELAY))
		{
			if(xSemaphoreTake(lock, portMAX_DELAY))
			{
				//run light set 2
				HAL_GPIO_WritePin(TS2_RED_GPIO_Port, TS2_RED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS2_GRN_GPIO_Port, TS2_GRN_Pin, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(2000));
				if (ped_request == pdTRUE) 	// if pedetrian pushed, blink wht 2 seconds after
				{
					HAL_GPIO_WritePin(PED_WHT_GPIO_Port, PED_WHT_Pin, GPIO_PIN_SET);
					ped_request = pdFALSE;
					in_task = pdFALSE;
				}
				vTaskDelay(pdMS_TO_TICKS(2000));
				HAL_GPIO_WritePin(TS2_GRN_GPIO_Port, TS2_GRN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PED_WHT_GPIO_Port, PED_WHT_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS2_YEL_GPIO_Port, TS2_YEL_Pin, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(1500));
				HAL_GPIO_WritePin(TS2_YEL_GPIO_Port, TS2_YEL_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS2_RED_GPIO_Port, TS2_RED_Pin, GPIO_PIN_SET);
				vTaskDelay(pdMS_TO_TICKS(1000));

				xSemaphoreGive(lock);
				xSemaphoreGive(s1);
			}
		}
	}
}

static void prvDebTimer( TimerHandle_t xTimer )
{
	(void) xTimer;

	if (debounce_bool != pdTRUE)
		debounce_bool = pdTRUE;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
