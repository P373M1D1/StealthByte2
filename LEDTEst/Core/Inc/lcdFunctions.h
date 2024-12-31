#ifndef LCDFUNCTIONS_H
#define LCDFUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <string.h>

#define LCD_WIDTH 20
#define LCD_HEIGHT 4

typedef enum {
    CONTROL_CHANGE,
    PROGRAM_CHANGE,
    // Add other message types here
} MessageType;

void displayPopup(MessageType messageType);
void updateBpm(uint32_t bpm);
void standbyScreen(void);
void displaySyncMessage(void);

#ifdef __cplusplus
}
#endif

#endif // LCDFUNCTIONS_H
