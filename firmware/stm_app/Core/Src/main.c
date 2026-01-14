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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOOTLOADER_FLAG 0xDEADBEEF
#define BUFFER_SIZE 64
#define MANUAL_ENTER_FLAG   (1U << 0)
#define MANUAL_EXIT_FLAG    (1U << 1)
#define UART_CMD_READY_FLAG (1U << 0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ledMutex */
osMutexId_t ledMutexHandle;
const osMutexAttr_t ledMutex_attributes = {
  .name = "ledMutex"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* Definitions for ledSemaphore */
osSemaphoreId_t ledSemaphoreHandle;
const osSemaphoreAttr_t ledSemaphore_attributes = {
  .name = "ledSemaphore"
};
/* USER CODE BEGIN PV */
extern int _bflag;
uint32_t *bootloader_flag;
uint32_t push_count = 0;
char transmit_buffer[BUFFER_SIZE];
char receive_buffer[BUFFER_SIZE];
char dma_buffer[BUFFER_SIZE];
volatile uint8_t is_manual_mode = 0;
volatile uint32_t buffer_length = 0;
char* signal;
char* status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void ManualOverride(void *argument);
void CommandTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  __HAL_UART_FLUSH_DRREGISTER(&huart1);
  __HAL_UART_FLUSH_DRREGISTER(&huart2);
  //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);

  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == BTN_Pin){
		//printf("Button Pressed \r\n");
		GPIO_PinState pinState = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
		if(pinState == GPIO_PIN_RESET){
			push_count = HAL_GetTick();
		}
		else{
			//printf("Button released \r\n");
			if(HAL_GetTick() - push_count > 1000){
				*bootloader_flag = BOOTLOADER_FLAG;
				HAL_NVIC_SystemReset();
			}
			push_count = 0;
		}
	}
}

void process_character(char ch)
{
    static uint16_t idx = 0;

    if (ch == '\0' || ch == '\n' || ch == '\r')
    {
    	//printf("inside_if\r\n");
    	if(idx > 0){
        receive_buffer[idx] = '\0';
        idx = 0;

        // ONLY signal that a command is ready
        osThreadFlagsSet(myTask03Handle, UART_CMD_READY_FLAG);
    	}
        //printf("commandtask called\r\n");

    }
    else if (idx < BUFFER_SIZE - 1)
    {
        receive_buffer[idx++] = ch;
        //printf("%c", ch);
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t offset) {

    static uint16_t last_offset = 0;
    if (offset != last_offset) {
        while (offset < last_offset && last_offset < BUFFER_SIZE){
        	process_character((char) dma_buffer[last_offset]);
        	++last_offset;
        }
        last_offset = 0;
        while (last_offset < offset) {
            process_character((char) dma_buffer[last_offset]);
            ++last_offset;
        }
    }
    //printf("In\r\n");
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)dma_buffer, BUFFER_SIZE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    printf("UART Error\n");
    Error_Handler();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	bootloader_flag = (uint32_t *)(& _bflag);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n\r\n Application Starting \r\n\r\n");
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) &dma_buffer, BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of ledMutex */
  ledMutexHandle = osMutexNew(&ledMutex_attributes);

  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ledSemaphore */
  ledSemaphoreHandle = osSemaphoreNew(1, 1, &ledSemaphore_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(ManualOverride, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(CommandTask, NULL, &myTask03_attributes);

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
  uint32_t now = 0, next_tick = 1000, next_blink = 500;
  while (1)
  {
	  now = uwTick;
	  if(now >= next_blink){
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  		  next_blink += 500;
  	  }
  	  if(now >= next_tick){
    	  printf("Tick %lu \r\n", now/1000);
    	  next_tick += 1000;
  	  }
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	//osStatus_t ret;
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(ledSemaphoreHandle, osWaitForever);

	  osMutexWait(ledMutexHandle, osWaitForever);
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  osMutexRelease(ledMutexHandle);
	  status = "RED";
	  osDelay(2000);

	  osMutexWait(ledMutexHandle, osWaitForever);
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  osMutexRelease(ledMutexHandle);
	  status = "YELLOW";
	  osDelay(2000);

	  osMutexWait(ledMutexHandle, osWaitForever);
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	  osMutexRelease(ledMutexHandle);
	  status = "GREEN";
	  osDelay(2000);
	  if(is_manual_mode == 0){
	  osSemaphoreRelease(ledSemaphoreHandle);
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ManualOverride */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ManualOverride */
void ManualOverride(void *argument)
{
  /* USER CODE BEGIN ManualOverride */
  /* Infinite loop */
  for(;;)
  {
	  //printf("in function\r\n");
	  osThreadFlagsWait(MANUAL_ENTER_FLAG, osFlagsWaitAny, osWaitForever);
	  //printf("Manual Override Triggered! \r\n");
	  is_manual_mode = 1;
	  uint8_t length = strlen(signal);
	  //printf("received %s , %d\r\n", signal, length);
	  osMutexAcquire(ledMutexHandle, osWaitForever);

	  if(length == 1){
	  char single_char = (char)(*signal);
	  //printf("received %c , %d\r\n", single_char, length);
	  switch(single_char){
	  case 'R':
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  	  status = "RED";
	  	  break;
	  case 'Y':
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    	  status = "BLUE";
    	  break;
	  case 'G':
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    	  status = "GREEN";
    	  break;
	  case 'E':
		  is_manual_mode = 0;
	  default:
		  is_manual_mode = 0;
	  }
	  }
	  else{
		  if(strcmp(signal, "I_9600") == 0){
			  printf("9600 change \n");
			  osDelay(100);
			  HAL_UART_DMAStop(&huart1);
			  huart1.Init.BaudRate = 9600;
			 // HAL_UART_Init(&huart1);
			  if (HAL_UART_Init(&huart1) == HAL_OK) {
				  printf("Baud rate changed to 9600 successfully\n");
			  }
			  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)dma_buffer, BUFFER_SIZE);
			  memset(signal, 0, length);
			  //printf("changed am here\n");
		  }
		  else if(strcmp(signal, "I_115200") == 0){
			  printf("115200 change \r\n");
			  HAL_UART_DMAStop(&huart1);
			  huart1.Init.BaudRate = 115200;
			  HAL_UART_Init(&huart1);
		  }
		  else if(strcmp(signal, "ENTER_BOOT") == 0){
			  printf("entering boot\r\n");
			  *bootloader_flag = BOOTLOADER_FLAG;
			  HAL_NVIC_SystemReset();
		  }
		  else if(strcmp(signal, "STATUS") == 0){
		  		  printf("Status : %s \n",status);
		  		  __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
		  		  HAL_UART_Transmit(&huart1, status, sizeof(status) + 1, 10);
		  	  }
		  //is_manual_mode = 0;
	  }
	  osMutexRelease(ledMutexHandle);
	  if(!is_manual_mode){
	  //printf("returning to default task! \r\n");
	  osSemaphoreRelease(ledSemaphoreHandle);
	  }

  }
  /* USER CODE END ManualOverride */
}

/* USER CODE BEGIN Header_CommandTask */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommandTask */
void CommandTask(void *argument)
{
  /* USER CODE BEGIN CommandTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(UART_CMD_READY_FLAG, osFlagsWaitAny, osWaitForever);
	  osMutexAcquire(uartMutexHandle, osWaitForever);
	  signal = receive_buffer;
	  osMutexRelease(uartMutexHandle);
	  osThreadFlagsSet(myTask02Handle, MANUAL_ENTER_FLAG);


  }
  /* USER CODE END CommandTask */
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
