/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "midiFunctions.h"
#include "math.h"
#include "lcdFunctions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern UART_HandleTypeDef huart4;
extern DAC_HandleTypeDef hdac;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void processReceivedByte(uint8_t byte);
extern void RotaryEncoderTurnedCallback(void);
void configureTimer(TIM_HandleTypeDef *htim, uint32_t bpm);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int16_t encoderPosition = 0;
volatile uint8_t lastEncoderState = 0;
volatile uint8_t tapTempoPressed = 0;
volatile uint8_t syncButtonPressed = 0;
volatile uint8_t syncSamples = 4;
volatile uint8_t midi_event_type = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Rotary_SW_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Rotary_CLK_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(TapTempo_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check if the interrupt is from TIM5
  if (htim->Instance == TIM5)														// used to be TIM3 !!!
  {
         //Send the MIDI Tap Tempo CC message
    synchroniseTempo(&huart4);
        }
  if (htim->Instance == TIM3)
  {
	  HAL_TIM_Base_Stop_IT(&htim3);  // Use HAL_TIM_Base_Stop_IT(&htim3) if interrupts are used

	      // Reset the counter
	      __HAL_TIM_SET_COUNTER(&htim3, 0);
	      standbyScreen();
  }
    }



// Callback function for the rotary encoder
void RotaryEncoderTurnedCallback(void)
{
    // Directly read the rotary encoder pins
    uint8_t clkState = HAL_GPIO_ReadPin(Rotary_CLK_GPIO_Port, Rotary_CLK_Pin);
    uint8_t dtState = HAL_GPIO_ReadPin(Rotary_DT_GPIO_Port, Rotary_DT_Pin);

    // Detect state changes (only on the CLK pin for simplicity)
    if (clkState != lastEncoderState)
    {
        if (clkState == GPIO_PIN_RESET) // CLK transition detected (falling edge)
        {
            if (dtState == GPIO_PIN_SET)
            {
                // Clockwise rotation
                encoderPosition++;
                currentBPM += 1.0;
            }
            else
            {
                // Counter-clockwise rotation
                encoderPosition--;
                currentBPM -= 1.0;
            }

            // Clamp BPM to a reasonable range
            if (currentBPM < 1.0)
                currentBPM = 1.0;
            else if (currentBPM > 240.0)
                currentBPM = 240.0;

            // Update the timer and display with the new BPM
            if (currentBPM > 0.0 && currentBPM < 240.0) { // Set a reasonable upper BPM limit

                        configureTimer(&htim5, currentBPM);						// used to be TIM3
                    }
            updateBpm((uint32_t)currentBPM);
        }

        // Update the last state
        lastEncoderState = clkState;
    }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    if (GPIO_Pin == Rotary_CLK_Pin)
        {
            RotaryEncoderTurnedCallback();
        }

    if (GPIO_Pin == Rotary_SW_Pin)
    	{
    	 	 system_event_flag |= EVENT_SYNC_BUTTON_PRESSED;
    	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        system_event_flag |= EVENT_TAP_TEMPO_PRESSED;
        //tapTempoPressed = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    static uint8_t lastStatusByte = 0;
    static uint8_t expectingValue = 0;

    if (huart->Instance == UART4)
    {
    	instance = 0;
        // Check if this is a new status byte
        if (receivedByte & 0x80)  // Status bytes have MSB set
        {
            lastStatusByte = receivedByte;
            channel = lastStatusByte & 0x0F;
            expectingValue = 0;
        }
        else  // Data byte
        {
            if ((lastStatusByte & 0xF0) == 0xB0)  // Control Change
            {
                if (expectingValue == 0)
                {
                    controllerNumber = receivedByte;  // Store controller number
                    expectingValue = 1;
                }
                else if (expectingValue == 1)
                {
                    controllerValue = receivedByte;  			// Store controller value
                    midi_event_type = 1;         				// Control Change
                    system_event_flag |= EVENT_MIDI_RECEIVED;  	// Set MIDI event
                    expectingValue = 0;
                }
            }
            else if ((lastStatusByte & 0xF0) == 0xC0)  			// Program Change
            {
                programChangeNumber = receivedByte;
                midi_event_type = 2;        				 	// Program Change
                system_event_flag |= EVENT_MIDI_RECEIVED;  		// Set MIDI event
            }
        }
                        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(MIDI_0, &receivedByte, 1);
    }
}



/* USER CODE END 1 */
