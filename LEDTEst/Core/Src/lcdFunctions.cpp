#include "lcdFunctions.h"
#include "stm32f4xx_hal.h"
#include "../../I2C_LCD/I2C_LCD.h"
#include "main.h"
#include "midiFunctions.h"
#include "timers.h"

const char* empty = "";
// External variables used in the function

// Buffer to hold the previous content of the display
char previousDisplay[LCD_HEIGHT][LCD_WIDTH + 1] = {0}; // +1 for null terminator

void updateLCDLine(int line, const char* newContent) {
    char paddedContent[LCD_WIDTH + 1]; // +1 for null terminator

    // Copy the new content into the padded content buffer
    strncpy(paddedContent, newContent, LCD_WIDTH);

    // Ensure the remaining part of the line is filled with spaces
    for (int i = strlen(newContent); i < LCD_WIDTH; i++) {
        paddedContent[i] = ' ';
    }
    paddedContent[LCD_WIDTH] = '\0'; // Ensure null termination

    // Check if the new content is different from the previous content
    if (strcmp(previousDisplay[line], paddedContent) != 0) {
        // Update the previous display content
        strcpy(previousDisplay[line], paddedContent);

        // Set the cursor and write the new content to the display
        I2C_LCD_SetCursor(LCD_1, 0, line);
        I2C_LCD_WriteString(LCD_1, paddedContent);
    }
}

void updateBpm(uint32_t bpm) {
    char buffer[LCD_WIDTH + 1]; // Buffer to store the converted string +1 for null terminator

    // Convert the integer to a string with padding to ensure it always occupies three digits
    snprintf(buffer, sizeof(buffer), "     BPM: %3lu", bpm); // %3lu ensures the BPM value is right-aligned in a field of width 3

    // Ensure the remaining part of the line is filled with spaces
    for (int i = strlen(buffer); i < LCD_WIDTH; i++) {
        buffer[i] = ' ';
    }
    buffer[LCD_WIDTH] = '\0'; // Ensure null termination
    // Update the line on the LCD
    updateLCDLine(0, buffer);
    updateLCDLine(1, empty);
    updateLCDLine(2, empty);
    updateLCDLine(3, empty);
    restart_TIM3();
}

void standbyScreen(void) {
    // Define the new content for each line

    const char* text = "  Stealth Byte 2.0";

    // Update only the necessary lines
    updateLCDLine(0, empty);
    updateLCDLine(1, text);
    updateLCDLine(2, empty);
    updateLCDLine(3, empty);
}

void displaySyncMessage(void){
	const char *text = "  synchronising...";
		    updateLCDLine(0, empty);
		    updateLCDLine(1, empty);
		    updateLCDLine(2, text);
		    updateLCDLine(3, empty);
		    restart_TIM3();
}

void displayPopup(MessageType messageType) {

    char midi_value_str[32];
    	snprintf(midi_value_str, sizeof(midi_value_str), "INST:%2d", instance); // Add 1 to channel to display 1-16
        updateLCDLine(0, midi_value_str);
    	snprintf(midi_value_str, sizeof(midi_value_str), "CH: %3d", channel + 1); // Add 1 to channel to display 1-16
        updateLCDLine(1, midi_value_str);

    switch (messageType) {

        case CONTROL_CHANGE:

            snprintf(midi_value_str, sizeof(midi_value_str), "CTL:%3d", controllerNumber); // %3d ensures the value is right-aligned in a field width of 3
            updateLCDLine(2, midi_value_str);
            snprintf(midi_value_str, sizeof(midi_value_str), "VAL:%3d", controllerValue); // %3d ensures the value is right-aligned in a field width of 3
            updateLCDLine(3, midi_value_str);
            sendControlChange(MIDI_0, 0, controllerNumber, controllerValue);
            break;

        case PROGRAM_CHANGE:
            snprintf(midi_value_str, sizeof(midi_value_str), "PGM:%3d", programChangeNumber); // %3d ensures the value is right-aligned in a field width of 3
            updateLCDLine(2, midi_value_str);
            sendProgramChange(MIDI_0, 0, programChangeNumber);
            break;

        default:
            // Handle unknown message type
            break;
    }
    restart_TIM3();
}


