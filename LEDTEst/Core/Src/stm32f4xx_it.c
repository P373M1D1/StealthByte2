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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t receivedByte;
extern TIM_HandleTypeDef htim3; // Ensure htim3 is declared globally
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


#define NUM_SAMPLES 4               // Number of samples for averaging durations
#define PERIOD_SAMPLES 4            // Number of samples for averaging periods
#define DEBOUNCE_DELAY_TICKS 250000 // 250ms debounce in timer ticks
#define TIMER_FREQ 1000000          // Timer frequency in Hz (1 MHz)
#define TIMEOUT_PERIOD_TICKS 2000000 // Timeout period in timer ticks (2s)
#define INITIAL_DURATION 50000      // Default initial duration (50ms)

#define DEBOUNCE_DELAY_MS 5



volatile uint32_t durations[NUM_SAMPLES] = {0};   // Circular buffer for durations
volatile uint8_t durationIndex = 0;               // Index in the buffer
volatile uint8_t validSamples = 0;                // Count of valid samples
volatile uint32_t previousCapture = 0;            // Previous capture value
volatile uint32_t lastDebounceCapture = 0;        // Last capture for debounce
volatile float currentBPM = 120.0;                // Default BPM
volatile uint32_t periods[PERIOD_SAMPLES] = {0};  // Circular buffer for periods
volatile uint8_t periodIndex = 0;                 // Index in the buffer
volatile uint8_t validPeriods = 0;                // Count of valid period samples
volatile uint8_t tapTempoPressed = 0;
volatile uint8_t syncButtonPressed = 0;
volatile uint8_t syncSamples = 4;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */
extern uint8_t buffer[3];
extern uint8_t statusByteControlChange;
extern uint8_t statusByteProgramChange;
extern uint8_t statusControllerNumber;

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
  // Check if the interrupt is from TIM3
  if (htim->Instance == TIM3)
  {
         //Send the MIDI Tap Tempo CC message
    sendTapTempo();
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

                        configureTimer(&htim3, currentBPM);
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
    	syncButtonPressed = 1;
    	syncSamples = 5;
    	}
}


void configureTimer(TIM_HandleTypeDef *htim, uint32_t bpm) {
    // Your existing code for configuring the timer
    uint32_t timerClock = 96000000;
    uint32_t prescaler = htim->Init.Prescaler + 1;
    uint32_t period = (timerClock / ((bpm / 60.0) * prescaler)) - 1;

    __HAL_TIM_SET_AUTORELOAD(htim, period);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, period / 8); // Example: 25% duty cycle
    __HAL_TIM_SET_COUNTER(htim, 0);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t duration;
        static uint32_t lastCaptureTime = 0;
        static uint32_t lastValidDuration = INITIAL_DURATION;

        // Handle timer overflow
        if (capture >= previousCapture) {
            duration = capture - previousCapture;
        } else {
            duration = (0xFFFFFFFF - previousCapture) + capture + 1;
        }
        previousCapture = capture;

        // If the time since the last valid tap exceeds TIMEOUT_PERIOD_TICKS, use the last valid duration
        if ((capture - lastCaptureTime) > TIMEOUT_PERIOD_TICKS) {
            duration = lastValidDuration;
        }
        lastCaptureTime = capture;

        // Debounce: Ignore taps too close together
        if ((capture - lastDebounceCapture) < DEBOUNCE_DELAY_TICKS) {
            return;
        }
        lastDebounceCapture = capture;

        // Store the duration in the circular buffer
        durations[durationIndex] = duration;
        durationIndex = (durationIndex + 1) % NUM_SAMPLES;

        // Track valid samples up to NUM_SAMPLES
        if (validSamples < NUM_SAMPLES) {
            validSamples++;
        }

        // Calculate average duration
        uint32_t totalDuration = 0;
        for (uint8_t i = 0; i < validSamples; i++) {
            totalDuration += durations[i];
        }
        float meanDuration = (float)totalDuration / validSamples;

        // Calculate BPM
        float timeInSeconds = meanDuration / TIMER_FREQ;
        float bpm = (timeInSeconds > 0) ? (60.0 / timeInSeconds) : 0;

        // Safety check to avoid invalid period or BPM values
        if (bpm > 0.0 && bpm < 240.0) { // Set a reasonable upper BPM limit
            currentBPM = bpm;
            configureTimer(&htim3, currentBPM);
        }

        // Update the display with the new BPM
        updateBpm((uint32_t)currentBPM);
        tapTempoPressed = 1;
        // Update last known duration
        lastValidDuration = duration;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t lastStatusByte = 0;
    static uint8_t expectingValue = 0;

    if (huart->Instance == UART4)
    {
        // Check if this is a new status byte
        if (receivedByte & 0x80)  // Status bytes have MSB set
        {
            lastStatusByte = receivedByte;
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
                    controllerValue = receivedByte;  // Store controller value
                    midi_received_flag = 1;          // Signal main loop
                    expectingValue = 0;
                }
            }
            else if ((lastStatusByte & 0xF0) == 0xC0)  // Program Change
            {
                programChangeNumber = receivedByte;
                midi_received_flag = 1;  // Signal main loop
            }
        }

        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(huart, &receivedByte, 1);
    }
}



/* USER CODE END 1 */
