/* Tasks.c  -  Initialization and system reset. */
/* Periodic tasks location: Core/Src/Tasks/ */

#include "Tasks.h"
#include "Communication/serial.h"
#include "shift_register.h"

CarData_t car_state = {0};

void TaskInit(void)
{
    CanInit();
    SR_Init();
    car_state.val_regen    = 1;
    car_state.val_src      = 1;
    car_state.val_tvc      = 1;
    car_state.val_event    = 1;
    int8_t pos = SR_GetStablePosition();

    if (pos >= 1 && pos <= 3) {
        car_state.SR_map       = (uint8_t)pos;
        car_state.current_page = 1;
        Nextion_Cmd("page page1");
    } else if (pos >= 4 && pos <= 7) {
        car_state.SR_map          = 1; /* ECO default: no map position selected */
        car_state.selected_setting = pos - 3;
        car_state.current_page    = 4;
        Nextion_Cmd("page page4");
    } else if (pos == 8) {
        car_state.SR_map       = 1; /* ECO default */
        car_state.current_page = 5;
        Nextion_Cmd("page page5");
    } else {
        /* pos == 0 (neutral) or -1 (hardware error): safe default */
        car_state.current_page = 1;
        Nextion_Cmd("page page1");
    }
}

void system_reset(void)
{
    Nextion_Cmd("rest");
    HAL_Delay(100);
    __disable_irq();
    NVIC_SystemReset();
}
