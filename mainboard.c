/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int TotalPetrol = 50000;
int pump1 = 0;
int pump2 = 0;
int pump3 = 0;
int pump4 = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for Pump1 */
osThreadId_t Pump1Handle;
const osThreadAttr_t Pump1_attributes = {
  .name = "Pump1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Pump2 */
osThreadId_t Pump2Handle;
const osThreadAttr_t Pump2_attributes = {
  .name = "Pump2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Pump3 */
osThreadId_t Pump3Handle;
const osThreadAttr_t Pump3_attributes = {
  .name = "Pump3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Pump4 */
osThreadId_t Pump4Handle;
const osThreadAttr_t Pump4_attributes = {
  .name = "Pump4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Sem1 */
osSemaphoreId_t Sem1Handle;
const osSemaphoreAttr_t Sem1_attributes = {
  .name = "Sem1"
};
/* Definitions for Sem2 */
osSemaphoreId_t Sem2Handle;
const osSemaphoreAttr_t Sem2_attributes = {
  .name = "Sem2"
};
/* Definitions for Sem3 */
osSemaphoreId_t Sem3Handle;
const osSemaphoreAttr_t Sem3_attributes = {
  .name = "Sem3"
};
/* Definitions for Sem4 */
osSemaphoreId_t Sem4Handle;
const osSemaphoreAttr_t Sem4_attributes = {
  .name = "Sem4"
};
/* USER CODE BEGIN PV */
static bool pump1flag = false;
static bool pump2flag = false;
static bool pump3flag = false;
static bool pump4flag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartPump1(void *argument);
void StartPump2(void *argument);
void StartPump3(void *argument);
void StartPump4(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(char Message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI0_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
	{
		pump1flag = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	}
}


void EXTI1_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
	{
		pump2flag = true;
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
	}
}

void EXTI4_IRQHandler(void)
{
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
	  {
		  pump3flag = true;
		  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
	  }
}

void EXTI9_5_IRQHandler(void)
{
	  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
	  {
		  pump4flag = true;
		  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
	  }
}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem1 */
  Sem1Handle = osSemaphoreNew(1, 1, &Sem1_attributes);

  /* creation of Sem2 */
  Sem2Handle = osSemaphoreNew(1, 1, &Sem2_attributes);

  /* creation of Sem3 */
  Sem3Handle = osSemaphoreNew(1, 1, &Sem3_attributes);

  /* creation of Sem4 */
  Sem4Handle = osSemaphoreNew(1, 1, &Sem4_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Pump1 */
  Pump1Handle = osThreadNew(StartPump1, NULL, &Pump1_attributes);

  /* creation of Pump2 */
  Pump2Handle = osThreadNew(StartPump2, NULL, &Pump2_attributes);

  /* creation of Pump3 */
  Pump3Handle = osThreadNew(StartPump3, NULL, &Pump3_attributes);

  /* creation of Pump4 */
  Pump4Handle = osThreadNew(StartPump4, NULL, &Pump4_attributes);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pump1_Pin Pump2_Pin */
  GPIO_InitStruct.Pin = Pump1_Pin|Pump2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Pump3_Pin Pump4_Pin */
  GPIO_InitStruct.Pin = Pump3_Pin|Pump4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Task_action(char Message)
{
	ITM_SendChar(Message);
	ITM_SendChar('\n');
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPump1 */
/**
  * @brief  Function implementing the Pump1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPump1 */
void StartPump1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	    osSemaphoreAcquire(Sem1Handle, osWaitForever);
	    if(pump1flag == true && TotalPetrol > 0)
	    {
	    	Task_action('1');
	    	pump1flag = false;

	    	pump1 = pump1 + 1;
	    	TotalPetrol = TotalPetrol - 1;
	    }
		else if(TotalPetrol == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
//	    osSemaphoreRelease(Sem2Handle);
//	    osThreadYield();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPump2 */
/**
* @brief Function implementing the Pump2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPump2 */
void StartPump2(void *argument)
{
  /* USER CODE BEGIN StartPump2 */
  /* Infinite loop */
  for(;;)
  {
//	    osSemaphoreAcquire(Sem2Handle, osWaitForever);
	    if(pump2flag == true && TotalPetrol > 0)
	    {
	    	Task_action('2');
	    	pump2flag = false;

	    	pump2 = pump2 + 1;
	    	TotalPetrol = TotalPetrol - 1;
	    }
		else if(TotalPetrol == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
//	    osSemaphoreRelease(Sem3Handle);
//	    osThreadYield();
  }
  /* USER CODE END StartPump2 */
}

/* USER CODE BEGIN Header_StartPump3 */
/**
* @brief Function implementing the Pump3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPump3 */
void StartPump3(void *argument)
{
  /* USER CODE BEGIN StartPump3 */
  /* Infinite loop */
  for(;;)
  {
//	    osSemaphoreAcquire(Sem3Handle, osWaitForever);
	    if(pump3flag == true && TotalPetrol > 0)
	    {
	    	Task_action('3');
	    	pump3flag = false;

	    	pump3 = pump3 + 1;
	    	TotalPetrol = TotalPetrol - 1;
	    }
		else if(TotalPetrol == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
//	    osSemaphoreRelease(Sem4Handle);
//	    osThreadYield();
  }
  /* USER CODE END StartPump3 */
}

/* USER CODE BEGIN Header_StartPump4 */
/**
* @brief Function implementing the Pump4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPump4 */
void StartPump4(void *argument)
{
  /* USER CODE BEGIN StartPump4 */
  /* Infinite loop */
  for(;;)
  {
//	    osSemaphoreAcquire(Sem4Handle, osWaitForever);
	    if(pump4flag == true && TotalPetrol > 0)
	    {
	    	Task_action('4');
	    	pump4flag = false;

	    	pump4 = pump4 + 1;
	    	TotalPetrol = TotalPetrol - 1;
	    }
		else if(TotalPetrol == 0)											// out of fuel
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
//	    osSemaphoreRelease(Sem1Handle);
//	    osThreadYield();
  }
  /* USER CODE END StartPump4 */
}

void StartDisplay(void *argument)
{
  /* USER CODE BEGIN 5 */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 100; // Adjust as per your requirement
    char message[100];

  /* Infinite loop */
  for(;;)
  {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);

      // Format the message
      unsigned int x=sprintf(message, "Total petrol : %d, pump 1 = %d, pump 2 = %d, pump 3 = %d, pump 4 = %d\n",
              TotalPetrol, pump1, pump2, pump3, pump4);

		  for (int i = 0; i<x; ++i) {
			  ITM_SendChar(message[i]);
		  }

      osThreadYield();
  }
  /* USER CODE END 5 */
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
