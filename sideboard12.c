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
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int pump1count = 0;
int pump2count = 0;
int TotalPetrol = 50000;
int NOP = 5000;
uint32_t startTick = 0;
uint32_t endTick = 0;
uint32_t timeTaken = 0;
int z=0;

// Define a debounce delay (in ms)
#define DEBOUNCE_DELAY 500

// Define a variable to store the timestamp of the last button press
volatile uint32_t last_pump1_press_time = 0;
volatile uint32_t last_pump1_stop_press_time = 0;
volatile uint32_t last_pump2_press_time = 0;
volatile uint32_t last_pump2_stop_press_time = 0;
volatile uint32_t last_stop_pump_press_time = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

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
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* USER CODE BEGIN PV */
static bool pump1flag = false;
static bool pump2flag = false;
static bool pumpstopflag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartPump1(void *argument);
void StartPump2(void *argument);
void StartLCD(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
////pump1 (A1 pin)
//void EXTI1_IRQHandler(void)
//{
//	uint32_t current_time = HAL_GetTick();
//	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
//	{
//		if ((current_time - last_pump1_press_time) > DEBOUNCE_DELAY){
//			last_pump1_press_time = current_time;
//			pump1flag = true;
//		}
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
//	}
//}
//
////stop pump1 (A2)
//void EXTI4_IRQHandler(void)
//{
//	uint32_t current_time = HAL_GetTick();
//	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
//	{
//		if ((current_time - last_pump1_stop_press_time) > DEBOUNCE_DELAY){
//			last_pump1_stop_press_time = current_time;
//			pump1flag = false;
//		}
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
//	}
//}
//
//// pump2 (A5)
//void EXTI0_IRQHandler(void)
//{
//	uint32_t current_time = HAL_GetTick();
//	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
//	{
//		if ((current_time - last_pump2_press_time) > DEBOUNCE_DELAY){
//			last_pump2_press_time = current_time;
//			pump2flag = true;
//		}
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
//	}
//}
//
////stop pump2 (D2)
//void EXTI15_10_IRQHandler(void)
//{
//	uint32_t current_time = HAL_GetTick();
//	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
//	{
//		if ((current_time - last_pump2_stop_press_time) > DEBOUNCE_DELAY){
//			last_pump2_stop_press_time = current_time;
//			pump2flag = false;
//		}
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
//	}
//}
//
////pump stop (all)
//void EXTI6_IRQHandler(void)
//{
//	uint32_t current_time = HAL_GetTick();
//	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
//	{
//		if ((current_time - last_stop_pump_press_time) > DEBOUNCE_DELAY){
//			last_stop_pump_press_time = current_time;
//			pumpstopflag = true;
//		}
//		  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
//	  }
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	uint32_t current_time = HAL_GetTick();

	//stop all
	if (GPIO_Pin == stop_all_Pin){

		pumpstopflag = true;
	}

	// pump1 start
	else if (GPIO_Pin == pump1_Pin){
		if ((current_time - last_pump1_press_time) > DEBOUNCE_DELAY){

			startTick = SysTick->VAL;
			z++;

			last_pump1_press_time = current_time;
			pump1flag = true;
		}
	}

	//pump1 stop
	else if (GPIO_Pin == pump1_stop_Pin){
		if ((current_time - last_pump1_stop_press_time) > DEBOUNCE_DELAY){
			last_pump1_stop_press_time = current_time;
			pump1flag = false;
		}
	}

	//pump2 start
	else if (GPIO_Pin == pump2_Pin){
		if ((current_time - last_pump2_press_time) > DEBOUNCE_DELAY){
			last_pump2_press_time = current_time;
			pump2flag = true;
		}
	}

	//pump2 stop
	else if (GPIO_Pin == pump2_stop_Pin){
		if ((current_time - last_pump2_stop_press_time) > DEBOUNCE_DELAY){
			last_pump2_stop_press_time = current_time;
			pump2flag = false;
		}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Pump1 */
  Pump1Handle = osThreadNew(StartPump1, NULL, &Pump1_attributes);

  /* creation of Pump2 */
  Pump2Handle = osThreadNew(StartPump2, NULL, &Pump2_attributes);

  /* creation of LCD */
  LCDHandle = osThreadNew(StartLCD, NULL, &LCD_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, pump2_out_Pin|pump1_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pump2_Pin stop_all_Pin */
  GPIO_InitStruct.Pin = pump2_Pin|stop_all_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : pump1_Pin pump1_stop_Pin pump2_stop_Pin */
  GPIO_InitStruct.Pin = pump1_Pin|pump1_stop_Pin|pump2_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : pump2_out_Pin pump1_out_Pin */
  GPIO_InitStruct.Pin = pump2_out_Pin|pump1_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

		if(pump1flag && pumpstopflag == false && TotalPetrol > 0){

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			for (int i = 0; i<NOP; i++){
				__NOP();
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);


			osMutexAcquire(myMutex01Handle, osWaitForever);
			TotalPetrol--;
			pump1count++;
			osMutexRelease(myMutex01Handle);

			endTick = SysTick->VAL;
			timeTaken = (startTick >= endTick) ? (startTick - endTick) : (SysTick->LOAD + startTick - endTick);
			z++;
	}
			osThreadYield();
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
		if(pump2flag && pumpstopflag == false && TotalPetrol > 0){

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			for (int i = 0; i<NOP; i++){
				__NOP();
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

			osMutexAcquire(myMutex01Handle, osWaitForever);
			TotalPetrol--;
			pump2count++;
			osMutexRelease(myMutex01Handle);
		}
		osThreadYield();
  }
  /* USER CODE END StartPump2 */
}

/* USER CODE BEGIN Header_StartLCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD */
void StartLCD(void *argument)
{
  /* USER CODE BEGIN StartLCD */
	  /* Initialize LCD */
	  HD44780_Init(2);
	  HD44780_Clear();

	  char buffer[16];


	  // Display "Pump 1: " at (0,0)
	  HD44780_SetCursor(0, 0);
	  HD44780_PrintStr("Pump 1: ");

	  // Display "Pump 2: " at (0,1)
	  HD44780_SetCursor(0, 1);
	  HD44780_PrintStr("Pump 2: ");

  /* Infinite loop */
  for(;;)
  {
	    // Display the value of pump1count at (8,0)
	    HD44780_SetCursor(7, 0);
	    snprintf(buffer, sizeof(buffer), " %7d", pump1count);
	    HD44780_PrintStr(buffer);

	    // Display the value of pump2count at (8,1)
	    HD44780_SetCursor(7, 1);
	    snprintf(buffer, sizeof(buffer), " %7d", pump2count);
	    HD44780_PrintStr(buffer);
  }
  /* USER CODE END StartLCD */
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
