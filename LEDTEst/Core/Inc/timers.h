/*
 * timers.h
 *
 *  Created on: Dec 20, 2024
 *      Author: Pete
 */

#ifndef TIMERS_H
#define TIMERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "midiFunctions.h"
#include "stm32f4xx_hal.h"  // Include necessary headers
#include "stm32f4xx_it.h"
#include "lcdFunctions.h"

#define NUM_SAMPLES 4               // Number of samples for averaging durations
#define PERIOD_SAMPLES 4            // Number of samples for averaging periods
#define DEBOUNCE_DELAY_TICKS 250000 // 250ms debounce in timer ticks
#define TIMER_FREQ 1000000          // Timer frequency in Hz (1 MHz)
#define TIMEOUT_PERIOD_TICKS 2000000 // Timeout period in timer ticks (2s)
#define INITIAL_DURATION 500000      // Default initial duration (500ms)
#define DEBOUNCE_DELAY_MS 20

// Function prototypes
void configureTimer(TIM_HandleTypeDef *htim, uint32_t bpm);

// Variable declarations
extern volatile uint32_t durations[NUM_SAMPLES];
extern volatile uint8_t durationIndex;
extern volatile uint32_t previousCapture;
extern volatile uint32_t lastDebounceCapture;
extern volatile float currentBPM;
extern volatile uint32_t capture;
extern volatile uint32_t periods[PERIOD_SAMPLES];
extern volatile uint8_t periodIndex;
extern volatile uint8_t validPeriods;

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H_ */
