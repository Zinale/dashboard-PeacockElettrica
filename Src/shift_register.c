#include "shift_register.h"

// File-global state (hidden outside)
static SR_State_t sr_state = {
    .currentValue = 0,
    .lastRawValue = 0xFF,
    .debounceCounter = 0,
    .lastReadTime = 0,
    .isValid = 0
};

// ====================================================================
// PRIVATE FUNCTIONS
// ====================================================================

// Raw bit-banging read (active-low PL)
static uint8_t SR_ReadRaw(void) {
    uint8_t data = 0;
    
    // Pulse latch to load parallel data (active-low PL)
    HAL_GPIO_WritePin(SR_latch_GPIO_Port, SR_latch_Pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 72; i++) __NOP(); // approx ~1us @ 72MHz
    HAL_GPIO_WritePin(SR_latch_GPIO_Port, SR_latch_Pin, GPIO_PIN_SET);
    
    for (volatile int i = 0; i < 10; i++) __NOP(); // Safety delay
    
    // Read data (MSB first: Q7 -> Q0)
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        
        if (HAL_GPIO_ReadPin(SR_data_GPIO_Port, SR_data_Pin) == GPIO_PIN_SET) {
            data |= 1;
        }
        
        HAL_GPIO_WritePin(SR_clock_GPIO_Port, SR_clock_Pin, GPIO_PIN_SET);
        for (volatile int j = 0; j < 5; j++) __NOP(); // Hold time
        HAL_GPIO_WritePin(SR_clock_GPIO_Port, SR_clock_Pin, GPIO_PIN_RESET);
        
        if (i < 7) {
            for (volatile int j = 0; j < 5; j++) __NOP(); 
        }
    }
    
    return data;
}

// Count set bits
static uint8_t countBits(uint8_t value) {
    uint8_t count = 0;
    while (value) {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

// Convert byte to selector physical position (1-8)
static int8_t byteToPosition(uint8_t data) {
    uint8_t bitCount = countBits(data);
    
    if (bitCount == 0) return 0; // No position active
    if (bitCount != 1) return -1; // Error: short circuit between positions
    
    for (int i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            return i + 1; 
        }
    }
    return -1; 
}


// ====================================================================
// PUBLIC FUNCTIONS (called by Tasks.c)
// ====================================================================

void SR_Init(void) {
    HAL_GPIO_WritePin(SR_clock_GPIO_Port, SR_clock_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SR_latch_GPIO_Port, SR_latch_Pin, GPIO_PIN_SET);
    
    uint8_t initialValue = SR_ReadRaw();
    sr_state.currentValue = initialValue;
    sr_state.lastRawValue = initialValue;
    sr_state.debounceCounter = SR_DEBOUNCE_SAMPLES; 
    sr_state.lastReadTime = HAL_GetTick();
    sr_state.isValid = 0;
}

void SR_ProcessPeriodic(void) {
    uint32_t currentTime = HAL_GetTick();
    
    if ((currentTime - sr_state.lastReadTime) < SR_READ_INTERVAL_MS) {
        return;
    }
    sr_state.lastReadTime = currentTime;
    
    uint8_t rawValue = SR_ReadRaw();
    
    // Debouncing logic
    if (rawValue == sr_state.lastRawValue) {
        if (sr_state.debounceCounter < SR_DEBOUNCE_SAMPLES) {
            sr_state.debounceCounter++;
        }
        
        if (sr_state.debounceCounter >= SR_DEBOUNCE_SAMPLES) {
            if (rawValue != sr_state.currentValue) {
                sr_state.currentValue = rawValue;
                sr_state.isValid = 1; // Set flag for Tasks.c
            }
        }
    } else {
        sr_state.lastRawValue = rawValue;
        sr_state.debounceCounter = 1; // Reset counter if fluctuating
    }
}

uint8_t SR_HasChanged(void) {
    if (sr_state.isValid) {
        sr_state.isValid = 0; // Consume event (flag reset)
        return 1;
    }
    return 0;
}

int8_t SR_GetStablePosition(void) {
    return byteToPosition(sr_state.currentValue);
}

uint8_t SR_GetRawValue(void) {
    return sr_state.lastRawValue;
}
