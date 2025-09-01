/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_IT2_Pin GPIO_PIN_2
#define SPI2_IT2_GPIO_Port GPIOE
#define W5500_RST_Pin GPIO_PIN_4
#define W5500_RST_GPIO_Port GPIOE
#define W5500_IT5_Pin GPIO_PIN_5
#define W5500_IT5_GPIO_Port GPIOE
#define W5500_IT5_EXTI_IRQn EXTI9_5_IRQn
#define ENC1_CHA_INCAP_T2C1_Pin GPIO_PIN_0
#define ENC1_CHA_INCAP_T2C1_GPIO_Port GPIOA
#define LED_D2_Pin GPIO_PIN_1
#define LED_D2_GPIO_Port GPIOA
#define DEBUG_TX_U2_Pin GPIO_PIN_2
#define DEBUG_TX_U2_GPIO_Port GPIOA
#define DEBUG_RX_U2_Pin GPIO_PIN_3
#define DEBUG_RX_U2_GPIO_Port GPIOA
#define ENC1_CHA_T3C1_Pin GPIO_PIN_6
#define ENC1_CHA_T3C1_GPIO_Port GPIOA
#define ENC1_CHB_T3C2_Pin GPIO_PIN_7
#define ENC1_CHB_T3C2_GPIO_Port GPIOA
#define PB11_OUT_Pin GPIO_PIN_11
#define PB11_OUT_GPIO_Port GPIOB
#define W5500_SPI2_SCK_Pin GPIO_PIN_13
#define W5500_SPI2_SCK_GPIO_Port GPIOB
#define W5500_SPI2_MISO_Pin GPIO_PIN_14
#define W5500_SPI2_MISO_GPIO_Port GPIOB
#define W5500_SPI2_MOSI_Pin GPIO_PIN_15
#define W5500_SPI2_MOSI_GPIO_Port GPIOB
#define W5500_SPI2_CS_Pin GPIO_PIN_8
#define W5500_SPI2_CS_GPIO_Port GPIOD
#define RS485_DIR_Pin GPIO_PIN_9
#define RS485_DIR_GPIO_Port GPIOD
#define STP_M_LIMIT_SW_IT10_Pin GPIO_PIN_10
#define STP_M_LIMIT_SW_IT10_GPIO_Port GPIOD
#define STP_M_LIMIT_SW_IT10_EXTI_IRQn EXTI15_10_IRQn
#define HBRIDG_LPWM_Pin GPIO_PIN_12
#define HBRIDG_LPWM_GPIO_Port GPIOD
#define HBRIDG_RPWM_Pin GPIO_PIN_13
#define HBRIDG_RPWM_GPIO_Port GPIOD
#define HBRIDG_LEN_Pin GPIO_PIN_14
#define HBRIDG_LEN_GPIO_Port GPIOD
#define ENC1_CHZ_IT15_Pin GPIO_PIN_15
#define ENC1_CHZ_IT15_GPIO_Port GPIOD
#define ENC1_CHZ_IT15_EXTI_IRQn EXTI15_10_IRQn
#define HBRIDG_REN_Pin GPIO_PIN_6
#define HBRIDG_REN_GPIO_Port GPIOC
#define STP_M_EN_Pin GPIO_PIN_7
#define STP_M_EN_GPIO_Port GPIOC
#define STP_M_DIR_Pin GPIO_PIN_9
#define STP_M_DIR_GPIO_Port GPIOC
#define STP_M_PUL_T1C1_Pin GPIO_PIN_8
#define STP_M_PUL_T1C1_GPIO_Port GPIOA
#define RS485_U1_TX_Pin GPIO_PIN_9
#define RS485_U1_TX_GPIO_Port GPIOA
#define RS485_U1_RX_Pin GPIO_PIN_10
#define RS485_U1_RX_GPIO_Port GPIOA
#define FLASH_SPI1_CS_Pin GPIO_PIN_15
#define FLASH_SPI1_CS_GPIO_Port GPIOA
#define FLASH_SPI1_SCK_Pin GPIO_PIN_3
#define FLASH_SPI1_SCK_GPIO_Port GPIOB
#define FLASH_SPI1_MISO_Pin GPIO_PIN_4
#define FLASH_SPI1_MISO_GPIO_Port GPIOB
#define FLASH_SPI1_MOSI_Pin GPIO_PIN_5
#define FLASH_SPI1_MOSI_GPIO_Port GPIOB
#define PWM_TMR10_CH1_Pin GPIO_PIN_8
#define PWM_TMR10_CH1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
