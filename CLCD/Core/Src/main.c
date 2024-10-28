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

#include <stdio.h>

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LCD_delay() {
/*	for(int i = 0; i < 16; i++) {
		__ASM volatile ("NOP");
	} */
	HAL_Delay(1);
}

void CLCD_cmd8(char cmd) {
	HAL_GPIO_WritePin(CLCD_RS_GPIO_Port, CLCD_RS_Pin, 0);

	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	HAL_GPIO_WritePin(CLCD_D7_GPIO_Port, CLCD_D7_Pin, (cmd >> 7) & 0x01);
	HAL_GPIO_WritePin(CLCD_D6_GPIO_Port, CLCD_D6_Pin, (cmd >> 6) & 0x01);
	HAL_GPIO_WritePin(CLCD_D5_GPIO_Port, CLCD_D5_Pin, (cmd >> 5) & 0x01);
	HAL_GPIO_WritePin(CLCD_D4_GPIO_Port, CLCD_D4_Pin, (cmd >> 4) & 0x01);
	HAL_GPIO_WritePin(CLCD_D3_GPIO_Port, CLCD_D3_Pin, (cmd >> 3) & 0x01);
	HAL_GPIO_WritePin(CLCD_D2_GPIO_Port, CLCD_D2_Pin, (cmd >> 2) & 0x01);
	HAL_GPIO_WritePin(CLCD_D1_GPIO_Port, CLCD_D1_Pin, (cmd >> 1) & 0x01);
	HAL_GPIO_WritePin(CLCD_D0_GPIO_Port, CLCD_D0_Pin, (cmd >> 0) & 0x01);

	HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 1);
	LCD_delay();

	HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 0);
	LCD_delay();
}

void CLCD_cmd4(char cmd) {
	HAL_GPIO_WritePin(CLCD_RS_GPIO_Port, CLCD_RS_Pin, 0);

	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	HAL_GPIO_WritePin(CLCD_D3_GPIO_Port, CLCD_D3_Pin, (cmd >> 3) & 0x01);
	HAL_GPIO_WritePin(CLCD_D2_GPIO_Port, CLCD_D2_Pin, (cmd >> 2) & 0x01);
	HAL_GPIO_WritePin(CLCD_D1_GPIO_Port, CLCD_D1_Pin, (cmd >> 1) & 0x01);
	HAL_GPIO_WritePin(CLCD_D0_GPIO_Port, CLCD_D0_Pin, (cmd >> 0) & 0x01);

	HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 1);
	LCD_delay();

	HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 0);
	LCD_delay();
}

void CLCD_str8(char x, char y, char *str) {
	CLCD_cmd8(((y == 0) ? 0x80 : 0xc0) + x);

	HAL_GPIO_WritePin(CLCD_RS_GPIO_Port, CLCD_RS_Pin, 1);

	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	while(*str) {
		HAL_GPIO_WritePin(CLCD_D3_GPIO_Port, CLCD_D3_Pin, (*str >> 7) & 0x01);
		HAL_GPIO_WritePin(CLCD_D2_GPIO_Port, CLCD_D2_Pin, (*str >> 6) & 0x01);
		HAL_GPIO_WritePin(CLCD_D1_GPIO_Port, CLCD_D1_Pin, (*str >> 5) & 0x01);
		HAL_GPIO_WritePin(CLCD_D0_GPIO_Port, CLCD_D0_Pin, (*str >> 4) & 0x01);
		HAL_GPIO_WritePin(CLCD_D3_GPIO_Port, CLCD_D3_Pin, (*str >> 3) & 0x01);
		HAL_GPIO_WritePin(CLCD_D2_GPIO_Port, CLCD_D2_Pin, (*str >> 2) & 0x01);
		HAL_GPIO_WritePin(CLCD_D1_GPIO_Port, CLCD_D1_Pin, (*str >> 1) & 0x01);
		HAL_GPIO_WritePin(CLCD_D0_GPIO_Port, CLCD_D0_Pin, (*str >> 0) & 0x01);

		HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 1);
		LCD_delay();

		HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 0);
		LCD_delay();

		str++;
	}
}

void CLCD_str4(char x, char y, char *str) {
	char cmd = ((y == 0) ? 0x80 : 0xc0) + x;
	CLCD_cmd4(cmd >> 4);
	CLCD_cmd4(cmd);

	HAL_GPIO_WritePin(CLCD_RS_GPIO_Port, CLCD_RS_Pin, 1);

	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	while(*str) {
		HAL_GPIO_WritePin(CLCD_D3_GPIO_Port, CLCD_D3_Pin, (*str >> 7) & 0x01);
		HAL_GPIO_WritePin(CLCD_D2_GPIO_Port, CLCD_D2_Pin, (*str >> 6) & 0x01);
		HAL_GPIO_WritePin(CLCD_D1_GPIO_Port, CLCD_D1_Pin, (*str >> 5) & 0x01);
		HAL_GPIO_WritePin(CLCD_D0_GPIO_Port, CLCD_D0_Pin, (*str >> 4) & 0x01);

		HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 1);
		LCD_delay();

		HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 0);
		LCD_delay();

		HAL_GPIO_WritePin(CLCD_D3_GPIO_Port, CLCD_D3_Pin, (*str >> 3) & 0x01);
		HAL_GPIO_WritePin(CLCD_D2_GPIO_Port, CLCD_D2_Pin, (*str >> 2) & 0x01);
		HAL_GPIO_WritePin(CLCD_D1_GPIO_Port, CLCD_D1_Pin, (*str >> 1) & 0x01);
		HAL_GPIO_WritePin(CLCD_D0_GPIO_Port, CLCD_D0_Pin, (*str >> 0) & 0x01);

		HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 1);
		LCD_delay();

		HAL_GPIO_WritePin(CLCD_EN_GPIO_Port, CLCD_EN_Pin, 0);
		LCD_delay();

		str++;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(200);

  /* function set */
//  CLCD_cmd(0x38);
  CLCD_cmd4(2);
  CLCD_cmd4(2);
  CLCD_cmd4(8);

  /* display on/off control */
//  CLCD_cmd(0x0c);
  CLCD_cmd4(0);
  CLCD_cmd4(12);

  /* display clear */
//  CLCD_cmd(0x01);
  CLCD_cmd4(0);
  CLCD_cmd4(1);
//  HAL_Delay(7); /* 6.2ms on sheet */
  HAL_Delay(20);

  /* entry mode set */
//  CLCD_cmd(0x06);
  CLCD_cmd4(0);
  CLCD_cmd4(6);

  /* cursor/display shift set */
//  CLCD_cmd(0x14);
  CLCD_cmd4(1);
  CLCD_cmd4(4);

  /* welcome display */
  CLCD_str4(0, 0, "MOCOM");
  HAL_Delay(1000);
  CLCD_str4(0, 1, "count = ");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char i = 0;
  char buf[17] = { 0 };
  while (1)
  {
	  sprintf(buf, "%2d", i);
	  CLCD_str4(8, 1, buf);
	  if(++i > 99)
		  i = 0;
	  HAL_Delay(200);

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
  HAL_GPIO_WritePin(GPIOC, CLCD_D0_Pin|CLCD_D1_Pin|CLCD_D2_Pin|CLCD_D3_Pin
                          |CLCD_D4_Pin|CLCD_D5_Pin|CLCD_D6_Pin|CLCD_D7_Pin
                          |CLCD_EN_Pin|CLCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLCD_D0_Pin CLCD_D1_Pin CLCD_D2_Pin CLCD_D3_Pin
                           CLCD_D4_Pin CLCD_D5_Pin CLCD_D6_Pin CLCD_D7_Pin
                           CLCD_EN_Pin CLCD_RS_Pin */
  GPIO_InitStruct.Pin = CLCD_D0_Pin|CLCD_D1_Pin|CLCD_D2_Pin|CLCD_D3_Pin
                          |CLCD_D4_Pin|CLCD_D5_Pin|CLCD_D6_Pin|CLCD_D7_Pin
                          |CLCD_EN_Pin|CLCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
