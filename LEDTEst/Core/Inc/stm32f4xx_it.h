/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
 extern TIM_HandleTypeDef htim2;
 extern TIM_HandleTypeDef htim3;
 extern TIM_HandleTypeDef htim5;
 extern UART_HandleTypeDef huart4;
 extern volatile float currentBPM;
 extern uint8_t buffer[3];
 extern uint8_t statusByteControlChange;
 extern uint8_t statusByteProgramChange;
 extern uint8_t statusControllerNumber;
 extern uint8_t receivedByte;

#define EVENT_MIDI_RECEIVED  		0x01  // MIDI event
#define EVENT_TAP_TEMPO_PRESSED     0x02  // Tap Tempo Pressed
#define EVENT_SYNC_BUTTON_PRESSED	0x04	//
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM5_IRQHandler(void);
void UART4_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
