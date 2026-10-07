#ifndef SHIFT_REGISTER_H_
#define SHIFT_REGISTER_H_

#include "main.h"

#define USE_SR
// #define SR_DEBUG_PRINT
#define R2D_DEBUG   /* Skip SDC/brake/speed checks for R2D activation */

#define SR_DEBOUNCE_SAMPLES 5
#define SR_READ_INTERVAL_MS 10 // Task runs at 20ms, so this can be 0 or 10

typedef struct {
    uint8_t currentValue;
    uint8_t lastRawValue;
    uint8_t debounceCounter;
    uint32_t lastReadTime;
    uint8_t isValid;
} SR_State_t;

void SR_Init(void);
void SR_ProcessPeriodic(void);
uint8_t SR_HasChanged(void);
int8_t SR_GetStablePosition(void);
uint8_t SR_GetRawValue(void);   // Returns last raw byte read (before debounce)



#endif /* SHIFT_REGISTER_H_ */