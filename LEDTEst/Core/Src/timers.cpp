/*
 * timers.cpp
 *
 * The Timers handle various tasks
 *
 * TIM2 - INPUT_CAPTURE for Tap Tempo Button (32bit timer) set to max
 * TIM3 - Popup Screen timeout counter - set to 1 hz
 * TIM5 - BPM Led PWM Generator 32bit counter set to bpm (0.1hz base rate = 120bpm)
 *
 *  Created on: Dec 20, 2024
 *      Author: Pete
 */

#include "timers.h"
#include "midiFunctions.h"

extern UART_HandleTypeDef *MIDI_0;

volatile uint32_t durations[NUM_SAMPLES] = {0};   // Circular buffer for durations
volatile uint8_t durationIndex = 0;               // Index in the buffer               // Count of valid samples
volatile uint32_t previousCapture = 0;            // Previous capture value
volatile uint32_t lastDebounceCapture = 0;        // Last capture for debounce
volatile float currentBPM = 120.0;                // Default BPM
volatile uint32_t capture = 0;                    // Capture value
volatile uint32_t periods[PERIOD_SAMPLES] = {0};  // Circular buffer for periods
volatile uint8_t periodIndex = 0;                 // Index in the buffer
volatile uint8_t validPeriods = 0;                // Count of valid period samples
double correctionValue = 1.0399;
uint8_t validSamples = 0;
uint8_t lastValidDuration= 0;


void restart_TIM3(void)
{
    // Stop the timer
    HAL_TIM_Base_Stop_IT(&htim3);  // Use HAL_TIM_Base_Stop_IT(&htim3) if interrupts are used

    // Reset the counter
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // Start the timer
    HAL_TIM_Base_Start_IT(&htim3);  // Use HAL_TIM_Base_Start_IT(&htim3) if interrupts are used
}

void configureTimer(TIM_HandleTypeDef *htim, uint32_t bpm) {
    // Timer clock frequency
    uint32_t timerClock = 96000000; // 96 MHz
    uint32_t prescaler = htim->Init.Prescaler + 1;

    // Calculate the period based on the desired BPM
    uint32_t period = (timerClock / ((bpm / 60.0) * prescaler)) - 1;

    // Calculate the pulse width for a 50ms duty cycle
    uint32_t pulseWidth = (50 * (timerClock / prescaler)) / 1000; // 50ms in timer ticks

    // Update the timer with the new period and pulse width
    __HAL_TIM_SET_AUTORELOAD(htim, period);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulseWidth);
    __HAL_TIM_SET_COUNTER(htim, 0);
}

void calculateTapTempo()
{
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
	            currentBPM = bpm / correctionValue;

	            configureTimer(&htim5, currentBPM);
	        }
	        // Update last known duration
	        lastValidDuration = duration;
	    }

