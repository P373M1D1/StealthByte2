#ifndef LCDFUNCTIONS_H
#define LCDFUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>


void updateDisplay(void);
void updateBpm(uint32_t bpm);

#ifdef __cplusplus
}
#endif

#endif // LCDFUNCTIONS_H
