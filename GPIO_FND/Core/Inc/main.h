/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FND_SEL3_Pin GPIO_PIN_3
#define FND_SEL3_GPIO_Port GPIOC
#define FND_SEL2_Pin GPIO_PIN_4
#define FND_SEL2_GPIO_Port GPIOC
#define FNDA_Pin GPIO_PIN_0
#define FNDA_GPIO_Port GPIOB
#define FNDB_Pin GPIO_PIN_1
#define FNDB_GPIO_Port GPIOB
#define FNDC_Pin GPIO_PIN_2
#define FNDC_GPIO_Port GPIOB
#define FND_SEL0_Pin GPIO_PIN_10
#define FND_SEL0_GPIO_Port GPIOA
#define FND_SEL1_Pin GPIO_PIN_11
#define FND_SEL1_GPIO_Port GPIOA
#define FNDD_Pin GPIO_PIN_3
#define FNDD_GPIO_Port GPIOB
#define FNDE_Pin GPIO_PIN_4
#define FNDE_GPIO_Port GPIOB
#define FNDF_Pin GPIO_PIN_5
#define FNDF_GPIO_Port GPIOB
#define FNDG_Pin GPIO_PIN_6
#define FNDG_GPIO_Port GPIOB
#define FNDP_Pin GPIO_PIN_7
#define FNDP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
