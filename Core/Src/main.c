/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "string.h"
#include "tcp.h"
/* Private includes ----------------------------------------------------------*/
#include "tinyg.h"					// #1 There are some dependencies
#include "config.h"					// #2
#include "hardware.h"
#include "persistence.h"
#include "controller.h"
#include "canonical_machine.h"
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "switch.h"
#include "test.h"
#include "pwm.h"
/* Private variables ---------------------------------------------------------*/
USART_HandleTypeDef husart1;
struct netif gnetif; /* network interface structure */
osThreadId mainTaskHandle ;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_Init(void);
void StartDefaultTask(void const * argument);
void tinyG(void const * argument);

void uartStr(char* s){
	HAL_USART_Transmit(&husart1, (uint8_t*)s, strlen(s), 1);
}


/**
  * @brief  The application entry point.
  */
int main(void){
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_Init();
  //-------------------------------------------------------------------
  // mainTaskHandle
  osThreadDef(mTask, tinyG, osPriorityNormal, 0, 1024);
  mainTaskHandle = osThreadCreate(osThread(mTask), NULL);
  //__enable_irq();
  MX_LWIP_Init();
  printf("Start " __DATE__ " " __TIME__ " \r\n");//,HAL_RCC_GetPCLK2Freq());
  printf("Core clock %d MHz\r\n",(uint16_t)(SystemCoreClock / 1000000));
  printf("ST clock %d MHz\r\n",(uint16_t)(TIM_CLOCK / 1000000));
  printf("f= %f \r\n", 123.456);
  initTcpServer() ;
  /* Start scheduler */
  osKernelStart();
  //------------------------------------------------------------------------------------------------------------------

  /* We should never get here as control is now taken by the scheduler */
  while (1){

  }
}


void tinyG(void const * argument){

	stepper_timers_init();
	hardware_init();
	// do these next
	stepper_init(); 					// stepper subsystem 				- must precede gpio_init()
	encoder_init();						// virtual encoders
//--	switch_init();					// switches
//	gpio_init();						// parallel IO
//--	pwm_init();						// pulse width modulation drivers	- must follow gpio_init()

	controller_init();// must be first app init; reqs xio_init()
	config_init();						// config records from eeprom 		- must be next app init
	planner_init();						// motion planning subsystem
	canonical_machine_init();			// canonical machine				-
	rpt_print_system_ready_message();	// (LAST) announce system is ready

	for (;;) {
		controller_run( );			// single pass through the controller
	}
}
//
void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void){
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate 	= 2000000;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK){
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  /* USER CODE BEGIN Callback 0 */
	static uint16_t ms ;
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
		ms++;
		//
		if(ms >= 1000){
			/* toggle PA0 */
			//GPIOA -> ODR ^= GPIO_PIN_0;
			//sys.uptime++;
			ms = 0 ;
		}
	}
	/* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM2) {
		IRQ_TIMER_DDA();
	}
	if (htim->Instance == TIM3) {
		IRQ_TIMER_DWELL();
		//printf("TIMER_DWELL \r\n");
	}
	if (htim->Instance == TIM4) {
		//printf("TIMER_LOAD \r\n");
		IRQ_TIMER_LOAD();
	}
	if (htim->Instance == TIM5) {
		//printf("TIMER_EXEC \r\n");
		IRQ_TIMER_EXEC();
	}
	/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
