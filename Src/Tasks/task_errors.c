/*
 * task_errors.c
 *  Task_CheckCANTimeouts - called by Task_Check_Errors
 *  Task_Check_Errors     - called by Task_UpdateDisplay every 100 ms
 *
 *  Error priority (highest first):
 *    0. Local hardware (SR, CAN controller)
 *    1. CAN Timeout (board not responding for > CAN_TIMEOUT_MS)
 *    2. Network errors: BMS -> MCU/Inverter -> PDM1 -> PDM2
 */

#include "Tasks.h"

/* Macro: set error and return */
#define SET_ERR_RET(e)  do { car_state.error_flag = true; car_state.error = (e); return; } while(0)
/* Macro: set error, return true (used in CheckCANTimeouts) */
#define TIMEOUT_ERR(e)  do { car_state.error_flag = true; car_state.error = (e); return true; } while(0)

/* =========================================================================
 * Task_CheckCANTimeouts
 * Returns true if at least one board hasn't transmitted for > CAN_TIMEOUT_MS.
 * ========================================================================= */
bool Task_CheckCANTimeouts(void)
{
    uint32_t now = HAL_GetTick();

#ifdef CAN_ERROR_RX_OTHER_BOARDS
    if ((now - car_state.bms.last_can_message_received)      > CAN_TIMEOUT_MS) TIMEOUT_ERR(ERR_TIMEOUT_BMS);
    if ((now - car_state.mcu.last_can_message_received)      > CAN_TIMEOUT_MS) TIMEOUT_ERR(ERR_TIMEOUT_MCU);
    if ((now - car_state.pdm_vcu1.last_can_message_received) > CAN_TIMEOUT_MS) TIMEOUT_ERR(ERR_TIMEOUT_PDM_VCU1);
    if ((now - car_state.pdm_vcu2.last_can_message_received) > CAN_TIMEOUT_MS) TIMEOUT_ERR(ERR_TIMEOUT_PDM_VCU2);

#endif
    return false;
}

/* =========================================================================
 * Task_Check_Errors
 * Scans all errors in priority order and writes car_state.error.
 * If no errors are present, resets error_flag and error to ERR_NONE.
 * ========================================================================= */
void Task_Check_Errors(void)
{
    /* --- Level 0: Local hardware errors (persistent) --- */
    if (car_state.error == ERR_SR_HARDWARE ||
        car_state.error == ERR_CAN_FILTER_INIT ||
        car_state.error == ERR_CAN_TX ||
        car_state.error == ERR_CAN_RX) {
        car_state.error_flag = true;
        return;
    }

    /* --- Livello 1: timeout CAN --- */
    if (Task_CheckCANTimeouts()) return;

    /* --- Livello 2: errori BMS pack --- */
    if (car_state.bms.pack_overvoltage_error)               SET_ERR_RET(ERR_BMS_PACK_OVERVOLTAGE);
    if (car_state.bms.pack_overcurrent_error)               SET_ERR_RET(ERR_BMS_PACK_OVERCURRENT);
    if (car_state.bms.cell_openwire_error)                  SET_ERR_RET(ERR_BMS_CELL_OPENWIRE);
    if (car_state.bms.temp_openwire_error)                  SET_ERR_RET(ERR_BMS_TEMP_OPENWIRE);
    if (car_state.bms.current_sensor_disconnected_error)    SET_ERR_RET(ERR_BMS_CURR_SENSOR_DISC);
    if (car_state.bms.slave_sensor_disconnected_error)      SET_ERR_RET(ERR_BMS_SLAVE_DISC);

    /* Errori BMS per modulo */
    for (uint8_t i = 0; i < BMS_NUM_MODULES; i++) {
        if (car_state.bms.modules[i].over_voltage_error)    SET_ERR_RET(ERR_BMS_BASE_OVERVOLTAGE  + i);
        if (car_state.bms.modules[i].under_voltage_error)   SET_ERR_RET(ERR_BMS_BASE_UNDERVOLTAGE + i);
        if (car_state.bms.modules[i].over_temperature_error)SET_ERR_RET(ERR_BMS_BASE_OVERTEMP     + i);
    }

    /* --- Livello 3: errori Inverter / MCU --- */
    if (car_state.mcu.inv104_status_word & INV_STAT_ERROR)  SET_ERR_RET(ERR_MCU_INV1_FAULT);
    if (car_state.mcu.inv204_status_word & INV_STAT_ERROR)  SET_ERR_RET(ERR_MCU_INV2_FAULT);
    if (car_state.mcu.inv104_derating)                      SET_ERR_RET(ERR_MCU_INV1_DERATING);
    if (car_state.mcu.inv204_derating)                      SET_ERR_RET(ERR_MCU_INV2_DERATING);
    if (car_state.mcu.APPS_implausibility)                  SET_ERR_RET(ERR_MCU_APPS_IMPLAUS);
    if (car_state.mcu.APP1_out_of_range)                    SET_ERR_RET(ERR_MCU_APP1_RANGE);
    if (car_state.mcu.APP2_out_of_range)                    SET_ERR_RET(ERR_MCU_APP2_RANGE);
    if (car_state.mcu.SAS_out_of_range)                     SET_ERR_RET(ERR_MCU_SAS_RANGE);

    /* --- Livello 4: errori PDM VCU1 --- */
    if (car_state.pdm_vcu1.overcurrent_R1)                  SET_ERR_RET(ERR_PDM1_OVERCURRENT_R1);
    if (car_state.pdm_vcu1.overcurrent_R2)                  SET_ERR_RET(ERR_PDM1_OVERCURRENT_R2);
    if (car_state.pdm_vcu1.overcurrent_R3)                  SET_ERR_RET(ERR_PDM1_OVERCURRENT_R3);
    if (car_state.pdm_vcu1.overcurrent_R4)                  SET_ERR_RET(ERR_PDM1_OVERCURRENT_R4);
    if (car_state.pdm_vcu1.negative_current_error)          SET_ERR_RET(ERR_PDM1_NEG_CURRENT);
    if (car_state.pdm_vcu1.range_24_error)                  SET_ERR_RET(ERR_PDM1_24V_RANGE);
    if (car_state.pdm_vcu1.low_voltage_error)               SET_ERR_RET(ERR_PDM1_LOW_BATT);
    if (car_state.pdm_vcu1.dc_temp_error)                   SET_ERR_RET(ERR_PDM1_DC_TEMP);
    if (car_state.pdm_vcu1.OC_24_voltage_error)             SET_ERR_RET(ERR_PDM1_OC_24V_RAIL);
    if (car_state.pdm_vcu1.OC_TSAC_fans_error)              SET_ERR_RET(ERR_PDM1_OC_TSAC_FANS);
    if (car_state.pdm_vcu1.OC_RAD_fans_L_error)             SET_ERR_RET(ERR_PDM1_OC_RAD_FANS_L);
    if (car_state.pdm_vcu1.OC_RAD_fans_R_error)             SET_ERR_RET(ERR_PDM1_OC_RAD_FANS_R);
    if (car_state.pdm_vcu1.OC_PUMP_error)                   SET_ERR_RET(ERR_PDM1_OC_PUMP);

    /* --- Livello 5: errori PDM VCU2 --- */
    if (car_state.pdm_vcu2.rear_brake_error)                SET_ERR_RET(ERR_PDM2_REAR_BRAKE);
    if (car_state.pdm_vcu2.rear_pushrod_L_error)            SET_ERR_RET(ERR_PDM2_REAR_PUSHRO_L);
    if (car_state.pdm_vcu2.rear_pushrod_R_error)            SET_ERR_RET(ERR_PDM2_REAR_PUSHRO_R);
    if (car_state.pdm_vcu2.rear_susp_L_error)               SET_ERR_RET(ERR_PDM2_REAR_SUSP_L);
    if (car_state.pdm_vcu2.rear_susp_R_error)               SET_ERR_RET(ERR_PDM2_REAR_SUSP_R);
    if (car_state.pdm_vcu2.coolant_temp_L_error)            SET_ERR_RET(ERR_PDM2_COOLANT_TEMP_L);
    if (car_state.pdm_vcu2.coolant_temp_R_error)            SET_ERR_RET(ERR_PDM2_COOLANT_TEMP_R);

    /* --- Nessun errore --- */
    car_state.error_flag = false;
    car_state.error      = ERR_NONE;
}
