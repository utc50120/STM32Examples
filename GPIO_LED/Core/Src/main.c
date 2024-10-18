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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	OFF_MODE	0
#define	BLINK_MODE	1
#define	BOUNCE_MODE	2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t	_rx_buffer[1];


int _led_mode = OFF_MODE;

uint16_t led_pin[] = { LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin, LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin };
GPIO_TypeDef *led_port[] = { LED0_GPIO_Port, LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port,
		LED4_GPIO_Port, LED5_GPIO_Port, LED6_GPIO_Port, LED7_GPIO_Port
};

uint8_t _tx_buffer[100] = "";
uint8_t message_1[] = "\n\n**************************\n";
uint8_t message_2[] = "LED Emergency Mode Control\n";
uint8_t message_3[] = "**************************\n";
uint8_t message_4[] = "\n1. Set to [blink] mode\n";
uint8_t message_5[] = "2. Set to [bounce] mode\n";
uint8_t message_6[] = "3. Off all LEDs\n";
uint8_t message_7[] = "4. Inquire current mode\n";
uint8_t message_8[] = "\nType number : ";
uint8_t message_9[] = "\n\nNow, current LED mode is [blink] mode!\n";
uint8_t message_10[] = "\n\nNow, current LED mode is [bounce] mode!\n";
uint8_t message_11[] = "\n\nNow, all LEDs are [off]!\n";
uint8_t message_12[] = "\n\nCurrent LED mode is [blink] mode.\n";
uint8_t message_13[] = "\n\nCurrent LED mode is [bounce] mode.\n";
uint8_t message_14[] = "\n\nCurrent LED mode is [off] mode.\n";
uint8_t message_15[] = "\a\n\nWrong number! Type correct number!\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void display_menu()
{
	HAL_UART_Transmit(&huart2, message_1, sizeof(message_1), 1000);
	HAL_UART_Transmit(&huart2, message_2, sizeof(message_2), 1000);
	HAL_UART_Transmit(&huart2, message_3, sizeof(message_3), 1000);
	HAL_UART_Transmit(&huart2, message_4, sizeof(message_4), 1000);
	HAL_UART_Transmit(&huart2, message_5, sizeof(message_5), 1000);
	HAL_UART_Transmit(&huart2, message_6, sizeof(message_6), 1000);
	HAL_UART_Transmit(&huart2, message_7, sizeof(message_7), 1000);
	HAL_UART_Transmit(&huart2, message_8, sizeof(message_8), 1000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void led_blink()
{
	for(unsigned int i = 0; i < 4; i++)
	{
		HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_port[i+4], led_pin[i+4], GPIO_PIN_RESET);
	}
	HAL_Delay(500);
	if(_led_mode != BLINK_MODE)
		return;
	for(unsigned int i = 0; i < 4; i++)
	{
		HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_port[i+4], led_pin[i+4], GPIO_PIN_SET);
	}
	HAL_Delay(500);
}

void led_bounce()
{
	for(unsigned int i = 0; i < 7; i++)
	{
		HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_port[i+1], led_pin[i+1], GPIO_PIN_SET);
		HAL_Delay(100);
		if(_led_mode != BOUNCE_MODE)
			return;
	}

	for(unsigned int i = 7; i > 0; i--)
	{
		HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_port[i-1], led_pin[i-1], GPIO_PIN_SET);
		HAL_Delay(100);
		if(_led_mode != BOUNCE_MODE)
			return;
	}
}

void led_off()
{
	for(unsigned int i = 0; i < 8; i++)
		HAL_GPIO_WritePin(led_port[i], led_pin[i], GPIO_PIN_RESET);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

/*  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
  HAL_Delay(500);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0); */

  display_menu();

  HAL_UART_Receive_IT(&huart2, _rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(_led_mode == BLINK_MODE)
		  led_blink();
	  else if(_led_mode == BOUNCE_MODE)
		  led_bounce();
	  else
		  led_off();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED4_Pin LED5_Pin LED6_Pin LED7_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* step 3 JKIT-NUCLEO-64 board LED0 ~ LED3 Toggle using SW1 interrupt */
/* void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SW1_Pin)	// SW1 interrupt
	{
		if(HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin))	// LED3 ON check
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin | LED2_Pin | LED1_Pin | LED0_Pin, GPIO_PIN_RESET);	// LED0~LED3 OFF
		else
		  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin | LED2_Pin | LED1_Pin | LED0_Pin, GPIO_PIN_SET);	// LED0~LED3 ON
	}
} */
/* ------------------------------------------------------------------ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch(_rx_buffer[0])
	{
	case '1':
		_led_mode = BLINK_MODE;
		HAL_UART_Transmit(&huart2, message_9, sizeof(message_9), 1000);
		break;
	case '2':
		_led_mode = BOUNCE_MODE;
		HAL_UART_Transmit(&huart2, message_10, sizeof(message_10), 1000);
		break;
	case '3':
		_led_mode = OFF_MODE;
		HAL_UART_Transmit(&huart2, message_11, sizeof(message_11), 1000);
		break;
	case '4':
	{
		switch(_led_mode)
		{
		case BLINK_MODE:
			HAL_UART_Transmit(&huart2, message_12, sizeof(message_12), 1000);
			break;
		case BOUNCE_MODE:
			HAL_UART_Transmit(&huart2, message_13, sizeof(message_13), 1000);
			break;
		default:
			HAL_UART_Transmit(&huart2, message_14, sizeof(message_14), 1000);
			break;
		}
		break;
	}
	default:
		HAL_UART_Transmit(&huart2, message_15, sizeof(message_15), 1000);
		break;
	}
	display_menu();
	HAL_UART_Receive_IT(&huart2, _rx_buffer, 1);
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
