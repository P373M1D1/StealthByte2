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
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern UART_HandleTypeDef *MIDI_0;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile uint8_t controllerNumber;
extern volatile uint8_t controllerValue;
extern volatile uint8_t programChangeNumber;
extern volatile uint8_t midi_received_flag;
extern volatile uint8_t sendTapTempoFlag;
extern volatile uint8_t tapTempoPressed;
extern volatile uint8_t syncButtonPressed;
extern volatile uint8_t syncSamples;
extern volatile uint32_t capture;
extern volatile uint8_t tapTempoPressed;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void updateBpm(uint32_t capture);
void calculateTapTempo(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Rotary_SW_Pin GPIO_PIN_4
#define Rotary_SW_GPIO_Port GPIOA
#define Rotary_SW_EXTI_IRQn EXTI4_IRQn
#define TIM2_CH1_Pin GPIO_PIN_5
#define TIM2_CH1_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define TapTempo_Pin GPIO_PIN_12
#define TapTempo_GPIO_Port GPIOF
#define TapTempo_EXTI_IRQn EXTI15_10_IRQn
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define TapTempoLED_Pin GPIO_PIN_6
#define TapTempoLED_GPIO_Port GPIOC
#define Rotary_DT_Pin GPIO_PIN_7
#define Rotary_DT_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MIDI_RX_Pin GPIO_PIN_0
#define MIDI_RX_GPIO_Port GPIOD
#define MIDI_TX_Pin GPIO_PIN_1
#define MIDI_TX_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Rotary_CLK_Pin GPIO_PIN_6
#define Rotary_CLK_GPIO_Port GPIOB
#define Rotary_CLK_EXTI_IRQn EXTI9_5_IRQn
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LCD_1 I2C_LCD_1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
