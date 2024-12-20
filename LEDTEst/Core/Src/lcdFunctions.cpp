#include "lcdFunctions.h"
#include "stm32f4xx_hal.h"
#include "../../I2C_LCD/I2C_LCD.h"
#include "main.h"

// External variables used in the function

void updateBpm(uint32_t bpm) {
    char buffer[16]; // Buffer to store the converted string
    I2C_LCD_SetCursor(LCD_1, 5, 0);
    I2C_LCD_WriteString(LCD_1, "               ");
    I2C_LCD_SetCursor(LCD_1, 0, 0); // Set the cursor on the LCD
    sprintf(buffer, "BPM: %lu", bpm); // Convert the integer to a string
    I2C_LCD_WriteString(LCD_1, buffer); // Write the string to the LCD
}

void updateDisplay(void)
{
    static uint8_t previousControllerNumber = 0xFF;
    static uint8_t previousControllerValue = 0xFF;
    static uint8_t previousProgramChangeNumber = 0xFF;
    char midi_value_str[32];

    if (controllerNumber != previousControllerNumber)
    {
        I2C_LCD_SetCursor(LCD_1, 12, 1);
        I2C_LCD_WriteString(LCD_1, "   ");
        I2C_LCD_SetCursor(LCD_1, 0, 1);
        snprintf(midi_value_str, sizeof(midi_value_str), "Controller: %d", controllerNumber);
        I2C_LCD_WriteString(LCD_1, midi_value_str);
        previousControllerNumber = controllerNumber;
    }

    if (controllerValue != previousControllerValue)
    {
        I2C_LCD_SetCursor(LCD_1, 7, 2);
        I2C_LCD_WriteString(LCD_1, "   ");
        I2C_LCD_SetCursor(LCD_1, 0, 2);
        snprintf(midi_value_str, sizeof(midi_value_str), "Value: %d", controllerValue);
        I2C_LCD_WriteString(LCD_1, midi_value_str);
        previousControllerValue = controllerValue;
    }

    if (programChangeNumber != previousProgramChangeNumber)
    {
        I2C_LCD_SetCursor(LCD_1, 9, 3);
        I2C_LCD_WriteString(LCD_1, "   ");
        I2C_LCD_SetCursor(LCD_1, 0, 3);
        snprintf(midi_value_str, sizeof(midi_value_str), "Program: %d", programChangeNumber);
        I2C_LCD_WriteString(LCD_1, midi_value_str);
        previousProgramChangeNumber = programChangeNumber;
    }
}

