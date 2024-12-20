#include "midiFunctions.h"
#include "stm32f4xx_hal.h"
#include "main.h"

// Define the MIDI channel and controller number
#define MIDI_CHANNEL 0x00 // MIDI Channel 1 (0-15 corresponds to 1-16)
#define TAP_TEMPO_CC 0x40 // Example Controller Number for Tap Tempo
#define TIMEOUT 100
extern UART_HandleTypeDef huart4; // Assuming UART2 is used for MIDI communication

uint8_t midiMessage1[3] = {
    0xB0 | (0 & 0x0F), // CC message on MIDI channel 1
    0x5D,              // Controller Number 81
    0x7F               // Value 127
};

uint8_t midiMessage2[3] = {
    0xB0 | (0 & 0x0F), // CC message on MIDI channel 1
    0x5D,              // Controller Number 81
    0x00               // Value 0
};



void synchroniseTempo(void)
{
    uint8_t midiMessage[3];

    if (syncButtonPressed == 1 && syncSamples != 0)
    {
    	sendTapTempo();
        syncSamples--;

        if (syncSamples == 0)
        {
            syncButtonPressed = 0;
        }
    }
}

void sendTapTempo(void){
	uint8_t midiMessage[3];

	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	    	    HAL_UART_Transmit(&huart4, midiMessage1, sizeof(midiMessage), TIMEOUT);


	    	    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	    	    //HAL_UART_Transmit(&huart4, midiMessage2, sizeof(midiMessage), TIMEOUT);
}
