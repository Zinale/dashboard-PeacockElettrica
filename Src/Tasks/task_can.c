/*
 * task_can.c
 *  Task_SendCAN - 150 ms
 *  Trasmette verso la macchina: R2D request, mappa motore, parametri dinamici.
 *
 *  Frame 0x106 - 3 byte:
 *    Byte 0 -> R2D REQUEST (0/1) - toggle del pulsante. Lo stato R2D vero lo
 *              gestisce la MCU e lo rimanda su 0x104 (byte 6/7).
 *    Byte 1 -> Mappa motore (1=ECO, 2=NORM, 3=GAS)
 *    Byte 2 -> Parametri [EVENT(7:6) | TVC(5:4) | SRC(3:2) | REGEN(1:0)]  (2 bit ciascuno, valore 0-3)
 */

#include "Tasks.h"
#include "main.h"


/* =========================================================================
 * Task_SendCAN  [150 ms]
 * ========================================================================= */
void Task_SendCAN(void)
{
    uint8_t tx_data[3] = {0};

    tx_data[0] = car_state.r2d_request ? 1 : 0;
    tx_data[1] = (uint8_t)car_state.SR_map;

    uint8_t p1 = (car_state.val_regen - 1) & 0x03;
    uint8_t p2 = (car_state.val_src   - 1) & 0x03;
    uint8_t p3 = (car_state.val_tvc    - 1) & 0x03;
    uint8_t p4 = (car_state.val_event - 1) & 0x03;
    tx_data[2] = (p4 << 6) | (p3 << 4) | (p2 << 2) | p1;

    Transmit_CAN_Message(&hcan, CAR_DASH_R2D_MAP_PARAMS, 3, tx_data);
}
