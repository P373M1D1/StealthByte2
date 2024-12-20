#include "lcdFunctions.h"
#include "stm32f4xx_hal.h"
#include "../../I2C_LCD/I2C_LCD.h"
#include "main.h"

// External variables used in the function


void updateBpm(uint32_t bpm) {
    char buffer[16]; // Buffer to store the converted string

    // Set the cursor on the LCD and clear the previous BPM value
    I2C_LCD_SetCursor(LCD_1, 0, 0);

    // Convert the integer to a string with padding to ensure it always occupies three digits
    sprintf(buffer, "BPM: %3lu", bpm); // %3lu ensures the BPM value is right-aligned in a field of width 3

    // Write the string to the LCD
    I2C_LCD_WriteString(LCD_1, buffer);
}

void updateDisplay(void)
{
    static uint8_t previousControllerNumber = 0xFF;
    static uint8_t previousControllerValue = 0xFF;
    static uint8_t previousProgramChangeNumber = 0xFF;
    char midi_value_str[32];

    if (controllerNumber != previousControllerNumber)
    {
        I2C_LCD_SetCursor(LCD_1, 0, 1);
        snprintf(midi_value_str, sizeof(midi_value_str), "Ctl: %3d", controllerNumber); // %3d ensures the value is right-aligned in a field width of 3
        I2C_LCD_WriteString(LCD_1, midi_value_str);
        previousControllerNumber = controllerNumber;
    }

    if (controllerValue != previousControllerValue)
    {
        I2C_LCD_SetCursor(LCD_1, 0, 2);
        snprintf(midi_value_str, sizeof(midi_value_str), "Val: %3d", controllerValue); // %3d ensures the value is right-aligned in a field width of 3
        I2C_LCD_WriteString(LCD_1, midi_value_str);
        previousControllerValue = controllerValue;
    }

    if (programChangeNumber != previousProgramChangeNumber)
    {
        I2C_LCD_SetCursor(LCD_1, 0, 3);
        snprintf(midi_value_str, sizeof(midi_value_str), "Pgm: %3d", programChangeNumber); // %3d ensures the value is right-aligned in a field width of 3
        I2C_LCD_WriteString(LCD_1, midi_value_str);
        previousProgramChangeNumber = programChangeNumber;
    }
}

