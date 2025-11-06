/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include "myprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TaskHandle_t TaskTempHandle;
TaskHandle_t TaskSpeedHandle;
//TaskHandle_t TaskRxHandle;

osThreadId TempTaskHandle;
osThreadId SpeedTaskHandle;
osThreadId RxTaskHandle;

osSemaphoreId mySemHandle;

osSemaphoreDef(mySem);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
	#define TICKS_PER_MS (osKernelSysTickFrequency / 1000)

	CAN_TxHeaderTypeDef TxHeader;
	CAN_TxHeaderTypeDef TempTxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t TxData[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	uint8_t RxData[8];
	float temp = 0;
	uint8_t	speed = 0;
	uint8_t button_status = 0;

	uint32_t txMailbox;
	CAN_FilterTypeDef sf;

	osMutexId canMutexHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void StartTempTask(void const * argument);
void StartSpeedTask(void const * argument);
void StartRxTask(void const * argument);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(canMutex);
  canMutexHandle = osMutexCreate(osMutex(canMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */


   // Crear semáforo binario con contador inicial = 0
  mySemHandle = osSemaphoreCreate(osSemaphore(mySem), 1);
    // Si quieres que empiece bloqueado, NO llames osSemaphoreRelease al inicio
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(SpeedTask, StartSpeedTask, osPriorityNormal, 0, 512);
  osThreadDef(TempTask, StartTempTask, osPriorityNormal, 0, 512);
  osThreadDef(RxTask, StartRxTask, osPriorityHigh, 0, 512);

  SpeedTaskHandle = osThreadCreate(osThread(SpeedTask), NULL);
  TempTaskHandle = osThreadCreate(osThread(TempTask), NULL);
  RxTaskHandle = osThreadCreate(osThread(RxTask), NULL);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterIdHigh = 0x0000;
  sf.FilterIdLow = 0x0000;
  sf.FilterMaskIdHigh = 0x0000;
  sf.FilterMaskIdLow = 0x0000;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;
  sf.SlaveStartFilterBank = 15;

  if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
     Error_Handler();
   }
  TxHeader.StdId = 0x10; // Este ID activa la respuesta del ESP8266
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.ExtId = 0x00;
  TxHeader.TransmitGlobalTime = DISABLE;

  TempTxHeader.DLC = 8;
  TempTxHeader.IDE = CAN_ID_STD;
  TempTxHeader.RTR = CAN_RTR_DATA;
  TempTxHeader.StdId = 0x11;
  TempTxHeader.ExtId = 0x00;
  TempTxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartTempTask(void const * argument)
{
	uint32_t PERIOD_MS = 2000;
	uint32_t lastWakeTime = osKernelSysTick();
	uint32_t periodTicks = PERIOD_MS * TICKS_PER_MS;
	for(;;)
	{
		osMutexWait(canMutexHandle, osWaitForever);


		// Enviar mensaje CAN con ID 0x10

	   if (HAL_CAN_AddTxMessage(&hcan, &TempTxHeader, TxData, &txMailbox) == HAL_OK) {

		  // printf("CAN TX: ID=0x%03lX Data:", TempTxHeader.StdId);
		   for (int i = 0; i < TempTxHeader.DLC; i++) {
			  // printf(" %02X", TxData[i]);
		   }
		   printf("Temperature request...\r\n");
		  // printf("\r\n");
	   } else {
		   //printf("Error al enviar CAN\r\n");
	   }
		osMutexRelease(canMutexHandle);
	   // --- Calcular siguiente tick absoluto ---
		 lastWakeTime += periodTicks;

		 // --- Esperar hasta ese tick ---
		 uint32_t now = osKernelSysTick();
		 if ((int32_t)(lastWakeTime - now) > 0) {
			 osDelay((lastWakeTime - now) / TICKS_PER_MS);
		 }

	}

}

void StartSpeedTask(void const * argument)
{
	uint32_t PERIOD_MS = 1000;
	uint32_t lastWakeTime = osKernelSysTick();
	uint32_t periodTicks = PERIOD_MS * TICKS_PER_MS;
	for(;;)
	{
		osMutexWait(canMutexHandle, osWaitForever);


	   // Enviar mensaje CAN con ID 0x10
	   if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &txMailbox) == HAL_OK) {

		   //printf("CAN TX: ID=0x%03lX Data:", TxHeader.StdId);
		   for (int i = 0; i < TxHeader.DLC; i++) {
			   //printf(" %02X", TxData[i]);
		   }
			printf("Speed request...\r\n");
		   //printf("\r\n");
	   } else {
		   //printf("Error al enviar CAN\r\n");
	   }
		 osMutexRelease(canMutexHandle);
	   // --- Calcular siguiente tick absoluto ---
		 lastWakeTime += periodTicks;

		 // --- Esperar hasta ese tick ---

		 uint32_t now = osKernelSysTick();
		 if ((int32_t)(lastWakeTime - now) > 0) {
			 osDelay((lastWakeTime - now) / TICKS_PER_MS);
		 }

	}

}

void StartRxTask(void const * argument)
{
	for(;;)
	{
	   // Esperar a que la ISR libere el semáforo
	osStatus statusSem = osSemaphoreWait(mySemHandle, osWaitForever);
	  if (statusSem == osOK) {

		  // Intentar tomar el mutex (bloquea si otra tarea lo está usando)

		 osStatus statusMutex = osMutexWait(canMutexHandle, osWaitForever);
		  printf("entra a rx");
						  printf("\r\n");
		 // if (statusMutex == osOK) {

						 // printf("CAN ID: 0x%03lX  Data:", RxHeader.StdId);
						  printf("toma mensaje exitoso");
						  printf("\r\n");

						  button_status = (0x01 & RxData[0]);
						  speed = (0xFF & RxData[1]);
						  uint32_t tempBits = ((uint32_t)RxData[2]) |
											  ((uint32_t)RxData[3] << 8) |
											  ((uint32_t)RxData[4] << 16) |
											  ((uint32_t)RxData[5] << 24);
						  memcpy(&temp, &tempBits, sizeof(float));
						  int tempamdanr = (int)temp;  // si 'temperatura' es float

						  printf("%d,%d,%d\n", speed, button_status, tempamdanr);

			 osMutexRelease(canMutexHandle);
		 // }

	  }



	}
	osMutexRelease(canMutexHandle);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	 BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
	    {
	        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	        {
	            // Libera el semáforo desde ISR
	        	//printf("semaforo");
	            xSemaphoreGiveFromISR(mySemHandle, &xHigherPriorityTaskWoken);
	        }



	    }
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }
  for(;;)
  {
	osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
