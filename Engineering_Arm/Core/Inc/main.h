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
#include "stm32f4xx_hal.h"

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
#define Vcc2_Pin GPIO_PIN_4
#define Vcc2_GPIO_Port GPIOE
#define Gnd2_Pin GPIO_PIN_5
#define Gnd2_GPIO_Port GPIOE
#define Cup_Pin_Pin GPIO_PIN_6
#define Cup_Pin_GPIO_Port GPIOE
#define Vcc1_Pin GPIO_PIN_0
#define Vcc1_GPIO_Port GPIOF
#define Gnd2F1_Pin GPIO_PIN_1
#define Gnd2F1_GPIO_Port GPIOF
#define N2_Pin GPIO_PIN_0
#define N2_GPIO_Port GPIOC
#define Relays_Pin_Pin GPIO_PIN_2
#define Relays_Pin_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_3
#define M1_GPIO_Port GPIOC
#define N1_Pin GPIO_PIN_4
#define N1_GPIO_Port GPIOC
#define O1_Pin GPIO_PIN_5
#define O1_GPIO_Port GPIOC
#define M2_Pin GPIO_PIN_1
#define M2_GPIO_Port GPIOB
#define VALVE_Pin GPIO_PIN_0
#define VALVE_GPIO_Port GPIOB
#define QiBeng1_Pin GPIO_PIN_12
#define QiBeng1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_uart8_rx;
extern UART_HandleTypeDef huart8;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
