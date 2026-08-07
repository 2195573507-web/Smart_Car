/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STM_RX_NO_UART_RX_AF_Pin GPIO_PIN_4
#define STM_RX_NO_UART_RX_AF_GPIO_Port GPIOD
#define RB_INT1_Pin GPIO_PIN_15
#define RB_INT1_GPIO_Port GPIOA
#define STM_TX_NO_UART_TX_AF_Pin GPIO_PIN_3
#define STM_TX_NO_UART_TX_AF_GPIO_Port GPIOD
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define RB_INT2_Pin GPIO_PIN_3
#define RB_INT2_GPIO_Port GPIOB
#define LF_INT2_SWDIO_CONFLICT_Pin GPIO_PIN_13
#define LF_INT2_SWDIO_CONFLICT_GPIO_Port GPIOA
#define LB_INT2_Pin GPIO_PIN_9
#define LB_INT2_GPIO_Port GPIOB
#define LB_INT1_Pin GPIO_PIN_8
#define LB_INT1_GPIO_Port GPIOB
#define LF_INT1_Pin GPIO_PIN_10
#define LF_INT1_GPIO_Port GPIOA
#define RF_INT2_Pin GPIO_PIN_9
#define RF_INT2_GPIO_Port GPIOA
#define AT2_AIN1_Pin GPIO_PIN_8
#define AT2_AIN1_GPIO_Port GPIOC
#define AT2_BIN1_Pin GPIO_PIN_9
#define AT2_BIN1_GPIO_Port GPIOC
#define RF_INT1_Pin GPIO_PIN_8
#define RF_INT1_GPIO_Port GPIOA
#define AT1_BIN1_Pin GPIO_PIN_7
#define AT1_BIN1_GPIO_Port GPIOC
#define AT1_AIN1_Pin GPIO_PIN_6
#define AT1_AIN1_GPIO_Port GPIOC
#define AT1_BIN2_Pin GPIO_PIN_1
#define AT1_BIN2_GPIO_Port GPIOC
#define S3_UART_TX_TO_ESP32_Pin GPIO_PIN_2
#define S3_UART_TX_TO_ESP32_GPIO_Port GPIOA
#define BMI323_INT1_Pin GPIO_PIN_2
#define BMI323_INT1_GPIO_Port GPIOB
#define LSM303_SCL_Pin GPIO_PIN_12
#define LSM303_SCL_GPIO_Port GPIOD
#define LSM303_SDA_Pin GPIO_PIN_13
#define LSM303_SDA_GPIO_Port GPIOD
#define BMI323_CS_Pin GPIO_PIN_4
#define BMI323_CS_GPIO_Port GPIOC
#define AT2_BIN2_Pin GPIO_PIN_15
#define AT2_BIN2_GPIO_Port GPIOB
#define S3_UART_RX_FROM_ESP32_Pin GPIO_PIN_3
#define S3_UART_RX_FROM_ESP32_GPIO_Port GPIOA
#define AT1_AIN2_Pin GPIO_PIN_5
#define AT1_AIN2_GPIO_Port GPIOC
#define AT2_AIN2_Pin GPIO_PIN_14
#define AT2_AIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
