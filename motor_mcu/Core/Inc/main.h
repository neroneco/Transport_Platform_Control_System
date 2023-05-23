/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define X_EN_Pin GPIO_PIN_5
#define X_EN_GPIO_Port GPIOC
#define Y_EN_Pin GPIO_PIN_12
#define Y_EN_GPIO_Port GPIOB
#define X_DIR_Pin GPIO_PIN_6
#define X_DIR_GPIO_Port GPIOC
#define X_STEP_Pin GPIO_PIN_8
#define X_STEP_GPIO_Port GPIOC
#define st_ADC_Pin GPIO_PIN_10
#define st_ADC_GPIO_Port GPIOA
#define Y_DIR_Pin GPIO_PIN_11
#define Y_DIR_GPIO_Port GPIOA
#define Y_STEP_Pin GPIO_PIN_12
#define Y_STEP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define st_UART_Pin GPIO_PIN_5
#define st_UART_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

typedef struct {
    float adc_pos[2];
    float pos[2];
    float vel[2];
    float acc[2];
    int   en[2];
} motor_status_struct;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
