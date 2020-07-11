/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define R5mA_ON_Pin GPIO_PIN_2
#define R5mA_ON_GPIO_Port GPIOE
#define R5mA_OFF_Pin GPIO_PIN_4
#define R5mA_OFF_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define SPI3_CS_Pin GPIO_PIN_6
#define SPI3_CS_GPIO_Port GPIOF
#define SPI3_DRDY_Pin GPIO_PIN_7
#define SPI3_DRDY_GPIO_Port GPIOF
#define SPI3_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define R25A_ON_Pin GPIO_PIN_8
#define R25A_ON_GPIO_Port GPIOF
#define R25A_OFF_Pin GPIO_PIN_9
#define R25A_OFF_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define SPI4_LATCH_Pin GPIO_PIN_10
#define SPI4_LATCH_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_15
#define SPI4_CS_GPIO_Port GPIOE
#define SPI1_SYNC_Pin GPIO_PIN_11
#define SPI1_SYNC_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_12
#define SPI1_CS_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define dummy_SPI1_DRDY_Pin GPIO_PIN_7
#define dummy_SPI1_DRDY_GPIO_Port GPIOC
#define SPI1_DRDY_Pin GPIO_PIN_9
#define SPI1_DRDY_GPIO_Port GPIOA
#define SPI1_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPI3_SCK_Pin GPIO_PIN_10
#define SPI3_SCK_GPIO_Port GPIOC
#define SPI3_MISO_Pin GPIO_PIN_11
#define SPI3_MISO_GPIO_Port GPIOC
#define SPI3_MOSI_Pin GPIO_PIN_12
#define SPI3_MOSI_GPIO_Port GPIOC
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
