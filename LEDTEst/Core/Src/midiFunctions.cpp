#include "midiFunctions.h"
#include "main.h"

#define TIMEOUT 100
extern


void sendProgramChange(UART_HandleTypeDef *midiInstance, uint8_t midiChannel, uint8_t programNumber) {
	uint8_t statusByte = MIDI_PROGRAM_CHANGE | (midiChannel & 0x0F);
	uint8_t midiMessage [2] = {statusByte, programNumber};
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    HAL_UART_Transmit(midiInstance, midiMessage, sizeof(midiMessage), TIMEOUT);
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}

void sendControlChange(UART_HandleTypeDef *midiInstance, uint8_t midiChannel, uint8_t controllerNumber, uint8_t value) {
	uint8_t statusByte = MIDI_CONTROL_CHANGE | (midiChannel & 0x0F);
	uint8_t midiMessage[3] = {statusByte, controllerNumber, value};
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    HAL_UART_Transmit(midiInstance, midiMessage, sizeof(midiMessage), TIMEOUT);
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}

void synchroniseTempo(UART_HandleTypeDef *midiInstance) {
    if (syncButtonPressed == 1 && syncSamples != 0) {
        // send midi tap tempo message
        syncSamples--;

        if (syncSamples == 0) {
            syncButtonPressed = 0;
        }
    }
}
