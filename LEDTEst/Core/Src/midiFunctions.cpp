#include "midiFunctions.h"
#include "stm32f4xx_hal.h"
#include "main.h"

// Define the MIDI channel and controller number
#define MIDI_CHANNEL 0x00 // MIDI Channel 1 (0-15 corresponds to 1-16)
#define TAP_TEMPO_CC 0x40 // Example Controller Number for Tap Tempo
#define TIMEOUT 100
extern UART_HandleTypeDef huart4; // Assuming UART2 is used for MIDI communication

void synchroniseTempo(void)
{
    uint8_t midiMessage[3];

    // Construct the MIDI CC message
    midiMessage[0] = 0xB0 | (MIDI_CHANNEL & 0x0F); // Ensure MIDI_CHANNEL is masked to 4 bits (0-15)
    midiMessage[1] = TAP_TEMPO_CC;                // Controller Number (0-127)
    midiMessage[2] = 0x7F;                        // Value (127 for max press)

    if (syncButtonPressed == 1 && syncSamples != 0)
    {
    	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        HAL_UART_Transmit(&huart4, midiMessage, sizeof(midiMessage), TIMEOUT);
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        syncSamples--;

        if (syncSamples == 0)
        {
            syncButtonPressed = 0;
        }
    }
}

void sendTapTempo(void){
	uint8_t midiMessage[3];
    midiMessage[0] = 0xB0 | (MIDI_CHANNEL & 0x0F); // Ensure MIDI_CHANNEL is masked to 4 bits (0-15)
    midiMessage[1] = TAP_TEMPO_CC;                // Controller Number (0-127)
    midiMessage[2] = 0x7F;                        // Value (127 for max press)

	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	    	    HAL_UART_Transmit(&huart4, midiMessage, sizeof(midiMessage), TIMEOUT);
	    	    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}
