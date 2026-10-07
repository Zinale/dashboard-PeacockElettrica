#include "Communication/can.h"
#include "Tasks.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;

static void Transmit_CAN_Message_MCP2551(CAN_HandleTypeDef *hcan, uint32_t StdId,
                                          uint32_t DLC, uint8_t *TxData);
static void Receive_CAN_Message_MCP2551(CAN_HandleTypeDef *hcan);

/* =========================================================================
 *  CAN INIT
 * ========================================================================= */
void CanInit(void)
{
#ifdef CAN_ENABLE
    CAN_FilterTypeDef sFilterConfig;

    /* Pass-all filter: accepts all standard IDs */
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        if (!car_state.error_flag)
        {
            car_state.error_flag = true;
            car_state.error      = ERR_CAN_FILTER_INIT;
        }
    }

    HAL_CAN_Start(&hcan);

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        if (!car_state.error_flag)
        {
            car_state.error_flag = true;
            car_state.error      = ERR_CAN_FILTER_INIT;
        }
    }

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
#endif
}

/* =========================================================================
 *  TRANSMISSION
 * ========================================================================= */
void Transmit_CAN_Message(CAN_HandleTypeDef *hcan, uint32_t StdId,
                           uint32_t DLC, uint8_t *TxData)
{
#ifdef USE_MCP2551
    Transmit_CAN_Message_MCP2551(hcan, StdId, DLC, TxData);
#endif
}

static void Transmit_CAN_Message_MCP2551(CAN_HandleTypeDef *hcan, uint32_t StdId,
                                          uint32_t DLC, uint8_t *TxData)
{
    CAN_TxHeaderTypeDef TxHeader;

    if (StdId <= 0x7FFu)
    {
        TxHeader.StdId = StdId;
        TxHeader.IDE   = CAN_ID_STD;
    }
    else
    {
        TxHeader.ExtId = StdId;
        TxHeader.IDE   = CAN_ID_EXT;
    }

    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = (DLC < 8u) ? DLC : 8u;

    uint32_t TxMailbox = CAN_TX_MAILBOX0;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        uint32_t hal_err = HAL_CAN_GetError(hcan);
        if ((hal_err & HAL_CAN_ERROR_ACK) ||
            (hal_err & HAL_CAN_ERROR_TX_TERR0) ||
            (hal_err & HAL_CAN_ERROR_BOF))
        {
            if (!car_state.error_flag)
            {
                car_state.error_flag = true;
                car_state.error      = ERR_CAN_TX;
            }
        }
    }
}

/* =========================================================================
 *  RECEPTION 
 * ========================================================================= */
void Receive_CAN_Message(CAN_HandleTypeDef *hcan)
{
#ifdef USE_MCP2551
    if (hcan != NULL)
        Receive_CAN_Message_MCP2551(hcan);
#endif
}

/* =========================================================================
 *  RECEPTION – MCP2551 implementation
 *  Called by callback HAL_CAN_RxFifo0MsgPendingCallback in main.c
 * ========================================================================= */
static void Receive_CAN_Message_MCP2551(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        uint32_t id = (RxHeader.IDE == CAN_ID_STD)
                      ? (uint32_t)RxHeader.StdId
                      : (uint32_t)RxHeader.ExtId;

        CAN_ParseFrame(id, RxData, (uint8_t)RxHeader.DLC);
    }
    else
    {
        if (!car_state.error_flag)
        {
            car_state.error_flag = true;
            car_state.error      = ERR_CAN_RX;
        }
    }
}

/* =========================================================================
 *  CAN_ParseFrame
 *  Decodifica il frame CAN e aggiorna car_state.
 * ========================================================================= */
void CAN_ParseFrame(uint32_t id, uint8_t *data, uint8_t dlc)
{
    uint32_t now = HAL_GetTick();

    switch (id)
    {
        /* -----------------------------------------------------------------
         * 0x001 – BMS_ERRORS  (DLC=4, BMS -> DASH)
         *
         * Byte 0: OverVoltage Error   - bit i = modulo i in overVoltage
         * Byte 1: UnderVoltage Error  - bit i = modulo i in underVoltage
         * Byte 2: OverTemp Error      - bit i = modulo i in overTemperature
         * Byte 3: FLAGS pack
         *   b5=Pack Overcurrent  b4=Pack OverVoltage  b3=Cell OpenWire
         *   b2=Temp OpenWire  b1=Curr Sensor Disc.  b0=Slave Sensor Disc.
         * ----------------------------------------------------------------- */
        case CAR_BMS_ERRORS:
        {
            if (dlc < 4u) break;

            for (uint8_t i = 0u; i < BMS_NUM_MODULES; i++)
            {
                car_state.bms.modules[i].over_voltage_error     = (data[0] >> i) & 0x01u;
                car_state.bms.modules[i].under_voltage_error    = (data[1] >> i) & 0x01u;
                car_state.bms.modules[i].over_temperature_error = (data[2] >> i) & 0x01u;
            }

            car_state.bms.pack_overcurrent_error             = (data[3] >> 5) & 0x01u;
            car_state.bms.pack_overvoltage_error             = (data[3] >> 4) & 0x01u;
            car_state.bms.cell_openwire_error                = (data[3] >> 3) & 0x01u;
            car_state.bms.temp_openwire_error                = (data[3] >> 2) & 0x01u;
            car_state.bms.current_sensor_disconnected_error  = (data[3] >> 1) & 0x01u;
            car_state.bms.slave_sensor_disconnected_error    = (data[3] >> 0) & 0x01u;

            car_state.bms.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x002 – INV1_ERRORS  (DLC=6, MCU -> DASH, little-endian)
         *
         * Byte 0-1: INV1_ERROR_NUM  (U16)
         * ----------------------------------------------------------------- */
        case CAR_INV1_ERRORS:
        {
            if (dlc < 6u) break;

            car_state.mcu.inv1_error_num = (uint16_t)(data[0] | (data[1] << 8));
            car_state.mcu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x003 – INV2_ERRORS  (DLC=6, MCU -> DASH, little-endian)
         *
         * Byte 0-1: INV2_ERROR_NUM  (U16)
         * ----------------------------------------------------------------- */
        case CAR_INV2_ERRORS:
        {
            if (dlc < 6u) break;

            car_state.mcu.inv2_error_num = (uint16_t)(data[0] | (data[1] << 8));

            car_state.mcu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x004 – PDM_VCU2_ERRORS  (DLC=1, PDM VCU2 -> DASH/DATALOGGER)
         *
         * Byte 0: FLAGS
         *   b7=Rear Brake  b6=PushRod L  b5=PushRod R
         *   b4=Rear Susp L  b3=Rear Susp R
         *   b2=Coolant Temp L  b1=Coolant Temp R  b0=Error present
         * ----------------------------------------------------------------- */
        case CAR_PDM_VCU2_ERRORS:
        {
            if (dlc < 1u) break;

            car_state.pdm_vcu2.rear_brake_error     = (data[0] >> 7) & 0x01u;
            car_state.pdm_vcu2.rear_pushrod_L_error = (data[0] >> 6) & 0x01u;
            car_state.pdm_vcu2.rear_pushrod_R_error = (data[0] >> 5) & 0x01u;
            car_state.pdm_vcu2.rear_susp_L_error    = (data[0] >> 4) & 0x01u;
            car_state.pdm_vcu2.rear_susp_R_error    = (data[0] >> 3) & 0x01u;
            car_state.pdm_vcu2.coolant_temp_L_error = (data[0] >> 2) & 0x01u;
            car_state.pdm_vcu2.coolant_temp_R_error = (data[0] >> 1) & 0x01u;
            car_state.pdm_vcu2.error_present        = (data[0] >> 0) & 0x01u;

            car_state.pdm_vcu2.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x005 – PDM_VCU1_ERRORS  (DLC=2, PDM VCU1 -> DASH/DATALOGGER)
         *
         * Byte 0: FLAGS (Errors Power)
         *   b7=OC R4  b6=OC R3  b5=OC R2  b4=OC R1
         *   b3=Neg Current  b2=24V Range  b1=Low Batt  b0=Error present
         * Byte 1: FLAGS (Errors Output)
         *   b7=DC Temp 24V  b6=DC Temp 5V  b5=OC 24V Rail  b4=OC TSAC Fans
         *   b3=OC Rad Fan R  b2=OC Rad Fan L  b1=OC Pump  b0=Error present
         * ----------------------------------------------------------------- */
        case CAR_PDM_VCU1_ERRORS:
        {
            if (dlc < 2u) break;

            car_state.pdm_vcu1.overcurrent_R4         = (data[0] >> 7) & 0x01u;
            car_state.pdm_vcu1.overcurrent_R3         = (data[0] >> 6) & 0x01u;
            car_state.pdm_vcu1.overcurrent_R2         = (data[0] >> 5) & 0x01u;
            car_state.pdm_vcu1.overcurrent_R1         = (data[0] >> 4) & 0x01u;
            car_state.pdm_vcu1.negative_current_error = (data[0] >> 3) & 0x01u;
            car_state.pdm_vcu1.range_24_error         = (data[0] >> 2) & 0x01u;
            car_state.pdm_vcu1.low_voltage_error      = (data[0] >> 1) & 0x01u;
            car_state.pdm_vcu1.error_present_1        = (data[0] >> 0) & 0x01u;

            car_state.pdm_vcu1.dc_temp_error          = (data[1] >> 7) & 0x01u;
            car_state.pdm_vcu1.OC_24_voltage_error    = (data[1] >> 5) & 0x01u;
            car_state.pdm_vcu1.OC_TSAC_fans_error     = (data[1] >> 4) & 0x01u;
            car_state.pdm_vcu1.OC_RAD_fans_R_error    = (data[1] >> 3) & 0x01u;
            car_state.pdm_vcu1.OC_RAD_fans_L_error    = (data[1] >> 2) & 0x01u;
            car_state.pdm_vcu1.OC_PUMP_error          = (data[1] >> 1) & 0x01u;
            car_state.pdm_vcu1.error_present_2        = (data[1] >> 0) & 0x01u;

            car_state.pdm_vcu1.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x006 – MCU_FLAGS  (DLC=1, MCU -> DASH)
         *
         * Byte 0:
         *   b0=INV1 Derating  b1=INV2 Derating  b2=APPs Implaus.
         *   b3=Regen Active   b4=APP1 OOR        b5=APP2 OOR  b6=SAS OOR
         * ----------------------------------------------------------------- */
        case CAR_MCU_FLAGS:
        {
            if (dlc < 1u) break;

            car_state.mcu.inv104_derating     = (data[0] >> 0) & 0x01u;
            car_state.mcu.inv204_derating     = (data[0] >> 1) & 0x01u;
            car_state.mcu.APPS_implausibility = (data[0] >> 2) & 0x01u;
            car_state.mcu.regen_active        = (data[0] >> 3) & 0x01u;
            car_state.mcu.APP1_out_of_range   = (data[0] >> 4) & 0x01u;
            car_state.mcu.APP2_out_of_range   = (data[0] >> 5) & 0x01u;
            car_state.mcu.SAS_out_of_range    = (data[0] >> 6) & 0x01u;

            car_state.mcu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x103 – BMS_STATE  (BMS -> DASH)
         *
         * Byte 0: SoC [%]         Byte 1: SoH [%]
         * Byte 2: Max disch. [A]  Byte 3: Max regen [A]
         * ----------------------------------------------------------------- */
        case CAR_BMS_STATE:
        {
            if (dlc < 6u) break;

            car_state.bms.SoC_percent           = data[0];
            car_state.bms.SoH_percent           = data[1];

            //big-endian in CAN
            car_state.bms.max_discharge_current = (uint8_t)(((data[2] << 8) | data[3]) * 0.01f); //Ampere
            car_state.bms.max_regen_current     = (uint8_t)(((data[4] << 8) | data[5]) * 0.01f); //Ampere

            car_state.bms.last_can_message_received = now;
            break;
        }

        /*-----------------------------------------------------------------
         * 0x104
         *
         -----------------------------------------------------------------*/
        case CAR_MCU_FW:
        {
            if (dlc < 8u) break;

            //big-endian, la MCU invia km/h * 10 (factor 0.1)
            car_state.mcu.wheel_speed_FR = (int16_t)(((int16_t)((data[0] << 8) | data[1])) / 10);   //km/h
            car_state.mcu.wheel_speed_FL = (int16_t)(((int16_t)((data[2] << 8) | data[3])) / 10);   //km/h
            car_state.mcu.car_speed      = (int16_t)(((int16_t)((data[4] << 8) | data[5])) / 10);   //km/h

            car_state.mcu.fsm_state  = (McuFsmState_t)data[6];         // stato FSM completo
            car_state.mcu.r2d_armed  = (data[7] & 0x01u) != 0u;       // armato (HV in salita o attivo)
            car_state.mcu.r2d_active = (data[7] & 0x02u) != 0u;       // R2D EFFETTIVO (coppia abilitata)

            car_state.mcu.last_can_message_received = now;
            break;
        }

        case CAR_MCU_BRAKE_PRES:
        {
            if (dlc < 2u) break;

            car_state.mcu.brake_pressure_front = (uint16_t)(data[0]);   //bar

            car_state.mcu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x107 – BMS_SDC_STATE  (BMS -> DASH)
         *
         * Byte 0: b0 = SDC chiuso (1 = ok)
         * ----------------------------------------------------------------- */
        case CAR_BMS_SDC_STATE:
        {
            if (dlc < 1u) break;

            car_state.bms.sdc_state = (data[0]) & 0x01u;

            car_state.bms.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x108 – MCU_APPS_SAS  (MCU -> DASH)
         *
         * Byte 0: APPS [%]   Byte 1: APP1 raw [%]
         * Byte 2: APP2 raw   Byte 3: SAS [%] (con segno)
         * ----------------------------------------------------------------- */
        case CAR_MCU_APPS_SAS:
        {
            if (dlc < 4u) break;

            car_state.mcu.apps_percent   = data[0];
            car_state.mcu.app1_raw       = data[1];
            car_state.mcu.app2_raw       = data[2];
            car_state.mcu.sas_percentage = (int8_t)data[3];

            car_state.mcu.last_can_message_received = now;
            break;
        }


        /*-----------------------------------------------------------------
          * 0x200 – BMS_V_MODS  (BMS -> DASH)
          *
          * Byte 0-5: Moduli 1-6 tensione [V]
         ----------------------------------------------------------------- */
        case CAR_BMS_V_MODS:
        {
            if (dlc < BMS_NUM_MODULES) break;

            for (uint8_t i = 0u; i < BMS_NUM_MODULES; i++)
            {
                car_state.bms.modules[i].voltage = (uint8_t)(data[i]);
            }

            car_state.bms.last_can_message_received = now;
            break;
        }

        /*-----------------------------------------------------------------
          * 0x201 – BMS_T_MODS  (BMS -> DASH)
          *
          * Byte 0-5: Moduli 1-6 temperatura [°C]
         ----------------------------------------------------------------- */
        case CAR_BMS_T_MODS:
        {
            if (dlc < BMS_NUM_MODULES) break;

            for (uint8_t i = 0u; i < BMS_NUM_MODULES; i++)
            {
                car_state.bms.modules[i].temperature = (uint8_t)(data[i]);
            }

            car_state.bms.last_can_message_received = now;
            break;
        }

        /*-----------------------------------------------------------------
          * 0x202 – BMS_FAN_SPEED  (BMS -> DASH)
          *
          * Byte 0-5: Percentuale velocità ventola moduli [%]
         ----------------------------------------------------------------- */
        case CAR_BMS_FAN_SPEED:
        {
            if (dlc < 1u) break;

            car_state.bms.battery_fan_percent = data[0];

            car_state.bms.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x250 – INV1_STATE  (MCU -> DASH, little-endian S16)
         *
         * Byte 2-3: motor1 speed [rpm]
         * Byte 4-5: motor1 temp  [°C]
         * Byte 6-7: inv1   temp  [°C]
         * ----------------------------------------------------------------- */
        case CAR_INV1_STATE:
        {
            if (dlc < 8u) break;
            
            car_state.mcu.inv104_status_word = (uint16_t)(data[0] | (data[1] << 8));
            car_state.mcu.motor104_speed = (int16_t)(data[2] | (data[3] << 8));
            car_state.mcu.motor104_temp  = (int16_t)(data[4] | (data[5] << 8));
            car_state.mcu.inv104_temp    = (int16_t)(data[6] | (data[7] << 8));

            car_state.mcu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x251 – INV2_STATE  (MCU -> DASH, little-endian S16)
         *
         * Byte 2-3: motor2 speed [rpm]
         * Byte 4-5: motor2 temp  [°C]
         * Byte 6-7: inv2   temp  [°C]
         * ----------------------------------------------------------------- */
        case CAR_INV2_STATE:
        {
            if (dlc < 8u) break;

            car_state.mcu.inv204_status_word = (uint16_t)(data[0] | (data[1] << 8));
            car_state.mcu.motor204_speed = (int16_t)(data[2] | (data[3] << 8));
            car_state.mcu.motor204_temp  = (int16_t)(data[4] | (data[5] << 8));
            car_state.mcu.inv204_temp    = (int16_t)(data[6] | (data[7] << 8));

            car_state.mcu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x300 – SOSP_FRONT  (-> DASH)
         *
         * Byte 0-1: susp. ant. SX [mm, con segno]
         * Byte 2-3: susp. ant. DX [mm, con segno]
         * ----------------------------------------------------------------- */
        case CAR_SOSP_FRONT:
        {
            if (dlc < 8u) break;

            car_state.ecu.front_suspension_L = (int8_t)((data[0] << 8) | data[1]);
            car_state.ecu.front_suspension_R = (int8_t)((data[2] << 8) | data[3]);

            car_state.ecu.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x301 – COOLANT_T  (PDM VCU2 -> DASH)
         *
         * Byte 0-1: temperatura radiatore sinistro [°C, con segno]
         * Byte 2-3: temperatura radiatore destro [°C, con segno]
         * ----------------------------------------------------------------- */
        case CAR_COOLANT_T:
        {
            if (dlc < 4u) break;

            float coolant_temp_left  = ((data[0] << 8) | data[1]) * 0.01f; //°C
            float coolant_temp_right = ((data[2] << 8) | data[3]) * 0.01f; //°C
            car_state.pdm_vcu2.coolant_temp = (int8_t)((coolant_temp_left + coolant_temp_right) / 2.0f);

            car_state.pdm_vcu2.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x303 – SOSP_REAR  (PDM VCU2 -> DASH)
         *
         * Byte 4-5: susp. post. SX [mm, con segno]
         * Byte 6-7: susp. post. DX [mm, con segno]
         * ----------------------------------------------------------------- */
        case CAR_SOSP_REAR:
        {
            if (dlc < 8u) break;

            car_state.pdm_vcu2.rear_suspension_L = (int8_t)((data[4] << 8) | data[5]);
            car_state.pdm_vcu2.rear_suspension_R = (int8_t)((data[6] << 8) | data[7]);

            car_state.pdm_vcu2.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x3C2 – CURRENT_BMS  (LEM -> DASH, big-endian S16)
         *
         * Byte 0-3: corrente LEM [A]
         * Extract signed 32-bit current value with offset
         * CAB500-C uses BIG-ENDIAN byte order (Motorola)
         * Formato: 0x80000000 = 0 mA, 0x7FFFFFFF = -1 mA, 0x80000001 = +1 mA
         * ----------------------------------------------------------------- */
        case CAR_CURRENT_BMS:
        {
            if (dlc < 4u) break;

            int32_t raw_be;
            raw_be = (int32_t) ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
            int32_t current_mA = raw_be - (int32_t) 0x80000000;
            float current_A = (float) current_mA * 0.001f;  // Convert mA to A
            car_state.lem_cab.LEM_current = (int16_t)current_A;
            car_state.lem_cab.last_can_message_received = now;
            break;
        }

        /* -----------------------------------------------------------------
         * 0x401 – LV_BATT  (PDM VCU1 -> DASH, little-endian U16)
         *
         * Byte 0-1: tensione LV [V * 100]  es. 1200 = 12.00 V
         * Byte 2-3: ampere LV [A * 100] es. 2400 = 24.00 V
         * ----------------------------------------------------------------- */
        case CAR_LV_BATT:
        {
            if (dlc < 4u) break;

            uint16_t raw;
            raw = (uint16_t)((data[0] << 8) | data[1]); //big-endian
            car_state.pdm_vcu1.lv_battery_voltage = (float)raw / 100.0f;

            car_state.pdm_vcu1.last_can_message_received = now;
            break;
        }

        /*Parsing pacchetti dati specifici dei moduli BMS x T MAX celle
        *  Byte 6-7: T MAX modulo i [°C * 10] es. 250 = 25.0 °C
        */
        case CAR_BMS_MODULE_1_DATA:
        case CAR_BMS_MODULE_2_DATA:
        case CAR_BMS_MODULE_3_DATA:
        case CAR_BMS_MODULE_4_DATA:
        case CAR_BMS_MODULE_5_DATA:
        case CAR_BMS_MODULE_6_DATA:
        {
            if (dlc < 8u) break;

            uint8_t module_index = BMS_NUM_MODULES - (id - CAR_BMS_MODULE_1_DATA); //0-based index: 0=modulo1, 1=modulo2, ..., 5=modulo6
            if (module_index < BMS_NUM_MODULES)
            {
                //big-endian
                int16_t t_max_raw = (int16_t)((data[6] << 8) | data[7]);
                car_state.bms.modules[module_index].t_max = (int16_t)(t_max_raw * 0.1f); //°C
            }

            car_state.bms.last_can_message_received = now;
            break;
        }


        default:
            /* Frame non gestito – ignorato */
            break;
    }
}



void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t errorcode = HAL_CAN_GetError(hcan);

    if (errorcode != HAL_CAN_ERROR_NONE)
    {
        if (!car_state.error_flag)
        {
            car_state.error_flag = true;
            car_state.error = ERR_CAN_BUSOFF; 
        }
    }
}
