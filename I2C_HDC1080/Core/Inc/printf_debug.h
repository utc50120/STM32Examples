/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : printf_debug.h
  * @brief          : Header for printf function to debugging
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PRINTF_DEBUG_H
#define __PRINTF_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* ---------------------------------------------------------------------------*/
/* USART printf
 *
 * include user code PV (using huart2)
 */
//#define	__PRINTF_USART	1
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* TRACE (SWV) printf
 *
 * 1. Debug Configuration... > Serial Wire Viewer (SWV) check
 * 2. Debug Run, Window > Show View > SWV > SWV ITM Data Console
 *    ITM Activation Port: 0 check
 *    Start Trace toggle
 * 3. Resume
 */
// #define	__PRINTF_TRACE	2
/* ---------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/* semihosting (SWO) printf
 *
 * 1. System Core > SYS > Debug: Trace Asyncronous Sw
 * 2. Project > Properties > C/C++ build > Settings > Tool Settings > MCU GCC Linker
 *    Libraries: rdimon
 *    Miscellaneous: -specs=rdimon.specs
 * 3. Project Explorer > Core > Src > syscalls.c select and Resource Configurations > Exclude from Build...
 * 4. initialise_monitor_handles(); type in main() function
 * 5. Debug Configurations...
 *    Debug Probe: ST-LINK(OpenOCD)
 *    Show generator options... > Reset Mode: Software system reset
 *    Startup > initialization Commands: monitor arm semihosting enable
 */
// #define	__PRINTF_SWO	3
/* ---------------------------------------------------------------------------*/

#include <stdio.h>

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#ifdef __PRINTF_USART
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 * set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
{
	if(ch == '\n') HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}
#else
int fputc(int ch, FILE *f)
{
	if(ch == '\n') HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}
#endif /* __GNUC__ */

#ifdef __PRINTF_TRACE
int _write(int file, char* ptr, int len)
{
	for(int i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}
#endif

#ifdef __PRINTF_SWO
extern void initialise_monitor_handles(void);
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
