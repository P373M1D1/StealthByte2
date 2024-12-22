#ifndef MIDIFUNCTIONS_H
#define MIDIFUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"

#define MIDI_CONTROL_CHANGE 0xB0
#define MIDI_PROGRAM_CHANGE 0xC0

extern UART_HandleTypeDef *MIDI_0;


void sendProgramChange(UART_HandleTypeDef *midiInstance, uint8_t midiChannel, uint8_t programNumber);
void sendControlChange(UART_HandleTypeDef *midiInstance, uint8_t midiChannel, uint8_t controllerNumber, uint8_t value);
void synchroniseTempo(UART_HandleTypeDef *midiInstance);

#ifdef __cplusplus
}
#endif

#endif // MIDIFUNCTIONS_H
