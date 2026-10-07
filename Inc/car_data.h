/*
 * car_data.h
 *
 *  Created on: 19 giu 2026
 *      Author: zinga
 */

#ifndef INC_CAR_DATA_H_
#define INC_CAR_DATA_H_

#include "stdint.h"
#include "stdbool.h"

#define BTN_COOLDOWN_MS   100
#define CAN_TIMEOUT_MS    3000
#define INV_STAT_ERROR          (1U << 9)   /**< bit 9: Error active */


/* =========================================================================
 *  DASHBOARD ERROR CODES
 *  Codes 1-99: dashboard system errors
 *  Codes 100+: CAN errors (frame specific)
 * ========================================================================= */
typedef enum {
    ERR_NONE                  = 0,

    /* CAN timeout errors (board not responding for > CAN_TIMEOUT_MS) */
    ERR_TIMEOUT_BMS           = 1,
    ERR_TIMEOUT_MCU           = 2,
    ERR_TIMEOUT_PDM_VCU1      = 3,
    ERR_TIMEOUT_PDM_VCU2      = 4,

    /* DASHBOARD CAN errors */
    ERR_SR_HARDWARE           = 99,
    ERR_CAN_FILTER_INIT       = 100,
    ERR_CAN_TX                = 101,
    ERR_CAN_RX                = 102,
    ERR_CAN_BUSOFF            = 103,

    /* BMS errors (0x001) */
    ERR_BMS_BASE_OVERVOLTAGE  = 201,  // real error = ERR_BMS_PACK_OVERVOLTAGE + module index
    ERR_BMS_BASE_UNDERVOLTAGE = 211,  // real error = ERR_BMS_PACK_UNDERVOLTAGE + module index
    ERR_BMS_BASE_OVERTEMP     = 221,  // real error = ERR_BMS_PACK_OVERTEMP + module index
    ERR_BMS_PACK_OVERCURRENT  = 207,
    ERR_BMS_PACK_OVERVOLTAGE  = 208,
    ERR_BMS_CELL_OPENWIRE     = 209,
    ERR_BMS_TEMP_OPENWIRE     = 210,
    ERR_BMS_CURR_SENSOR_DISC  = 217,
    ERR_BMS_SLAVE_DISC        = 218,

    /* Errori Inverter (0x002 - 0x003)*/
    ERR_MCU_INV1_FAULT        = 310,
    ERR_MCU_INV2_FAULT        = 311,


    /* Errori MCU/Inverter (0x006) */
    ERR_MCU_INV1_DERATING     = 300,
    ERR_MCU_INV2_DERATING     = 301,
    ERR_MCU_APPS_IMPLAUS      = 302,
    ERR_MCU_APP1_RANGE        = 303,
    ERR_MCU_APP2_RANGE        = 304,
    ERR_MCU_SAS_RANGE         = 305,
    

    /* Errori PDM VCU1 (0x005) */
    ERR_PDM1_OVERCURRENT_R1   = 400,
    ERR_PDM1_OVERCURRENT_R2   = 401,
    ERR_PDM1_OVERCURRENT_R3   = 402,
    ERR_PDM1_OVERCURRENT_R4   = 403,
    ERR_PDM1_NEG_CURRENT      = 404,
    ERR_PDM1_24V_RANGE        = 405,
    ERR_PDM1_LOW_BATT         = 406,
    ERR_PDM1_DC_TEMP          = 407,
    ERR_PDM1_OC_24V_RAIL      = 408,
    ERR_PDM1_OC_TSAC_FANS     = 409,
    ERR_PDM1_OC_RAD_FANS_L    = 410,
    ERR_PDM1_OC_RAD_FANS_R    = 411,
    ERR_PDM1_OC_PUMP          = 412,

    /* PDM VCU2 errors (0x004) */
    ERR_PDM2_REAR_BRAKE       = 500,
    ERR_PDM2_REAR_PUSHRO_R    = 501,
    ERR_PDM2_REAR_PUSHRO_L    = 502,
    ERR_PDM2_REAR_SUSP_L      = 503,
    ERR_PDM2_REAR_SUSP_R      = 504,
    ERR_PDM2_COOLANT_TEMP_L   = 505,
    ERR_PDM2_COOLANT_TEMP_R   = 506,
} DashError_t;

typedef struct{
    bool pressed;
    bool just_pressed;          // true ONLY for one cycle
    uint32_t last_pressed_time;
} button_t;

/* Stato FSM della MCU (frame 0x104 byte 6). DEVE combaciare con VehicleState_t
 * lato MCU (shared_data.h). */
typedef enum {
    MCU_FSM_INIT           = 0,
    MCU_FSM_READY          = 1,
    MCU_FSM_RUNNING        = 2,   // R2D attivo, coppia abilitata
    MCU_FSM_IMPLAUSIBILITY = 3,
    MCU_FSM_CAR_ERROR      = 4,
    MCU_FSM_ERROR          = 5,
} McuFsmState_t;

#define BMS_NUM_MODULES  6

typedef struct{
    //0x001
    bool over_voltage_error;
    bool under_voltage_error;
    bool over_temperature_error;

    //0x200
    uint8_t voltage;
    uint8_t temperature;
    int16_t t_max;
} BMS_Module_t;

typedef struct{
    BMS_Module_t modules[BMS_NUM_MODULES];

    //0x001
    bool pack_overcurrent_error;
    bool pack_overvoltage_error;
    bool cell_openwire_error;
    bool temp_openwire_error;
    bool current_sensor_disconnected_error;
    bool slave_sensor_disconnected_error;

    //0x103
    uint8_t SoC_percent;
    uint8_t SoH_percent;
    uint8_t max_discharge_current;
    uint8_t max_regen_current;

    //0x107
    bool sdc_state;

    //0x202
    uint8_t battery_fan_percent;

    uint32_t last_can_message_received;
} BMS_t;


typedef struct{
    //0x002 - INV1 errors 
    uint16_t inv1_error_num;
    //0x003 - INV2 errors 
    uint16_t inv2_error_num;
    //0x006
    bool inv104_derating;
    bool inv204_derating;
    bool APPS_implausibility;
    bool regen_active;
    bool APP1_out_of_range;
    bool APP2_out_of_range;
    bool SAS_out_of_range;

    //0x108
    uint8_t apps_percent;
    uint8_t app1_raw;
    uint8_t app2_raw;
    int8_t sas_percentage;

    //0x250
    uint16_t inv104_status_word;
    int16_t motor104_speed; //rpm
    int16_t motor104_temp; //°C
    int16_t inv104_temp; //°C

    //0x251
    uint16_t inv204_status_word;
    int16_t motor204_speed; //rpm
    int16_t motor204_temp; //°C
    int16_t inv204_temp; //°C

    //0x104
    int16_t wheel_speed_FL; //km/h
    int16_t wheel_speed_FR; //km/h
    int16_t car_speed; //km/h
    McuFsmState_t fsm_state;   // byte 6 - stato FSM completo (per error display / logica pagine)
    bool     r2d_armed;        // byte 7 bit0 - il pilota ha armato (HV in salita o attivo)
    bool     r2d_active;       // byte 7 bit1 - R2D EFFETTIVO (coppia abilitata) = quello che accende buzzer/LED

    uint8_t brake_pressure_front; //bar

    uint32_t last_can_message_received;
} MCU_t;

typedef struct{
    //0x100, 0x101, 0x102
    uint32_t last_can_message_received;
} IMU_t;

typedef struct{
    //0x3C2
    int16_t LEM_current; //A
    uint32_t last_can_message_received;
} LEM_CAB_t;

typedef struct{
    //0x005
    bool overcurrent_R4, overcurrent_R3, overcurrent_R2, overcurrent_R1, negative_current_error, range_24_error, low_voltage_error, error_present_1;

    bool dc_temp_error, OC_24_voltage_error, OC_TSAC_fans_error, OC_RAD_fans_L_error, OC_RAD_fans_R_error, OC_PUMP_error, error_present_2;

    //0x401
    float lv_battery_voltage; //V

    uint32_t last_can_message_received;
} PDM_VCU1_t;

typedef struct{
    //0x004
    bool rear_brake_error, rear_pushrod_L_error, rear_pushrod_R_error, rear_susp_L_error, rear_susp_R_error, coolant_temp_L_error, coolant_temp_R_error, error_present;

    //0x301
    int8_t coolant_temp; //avg °C

    //0x303
    int8_t rear_suspension_L; //mm
    int8_t rear_suspension_R; //mm

    uint32_t last_can_message_received;
} PDM_VCU2_t;

typedef struct{
    //0x300
    int8_t front_suspension_L; //mm
    int8_t front_suspension_R; //mm

    uint32_t last_can_message_received;
} ECU_t;

typedef struct {
    button_t btn_1;            
    button_t btn_2;           
    bool     btn_combo;        
    
    uint8_t  SR_map;            // 1=ECO, 2=NORM, 3=GAS
    uint8_t  selected_setting;  // 0=Nessuno, 1=REGEN, 2=SRC, 3=OS, 4=TVC
    bool 	 r2d_request;       // RICHIESTA R2D dal pilota (toggle pulsante) - lo stato vero lo tiene la MCU (mcu.fsm_state)
    uint8_t  SR_raw_position;  // Valore raw da shift register (0-8, -1=errore hardware)

    // Valori dei parametri di dinamica modificabili dal pilota
    uint8_t  val_regen;
    uint8_t  val_src;
    uint8_t  val_tvc;
    uint8_t  val_event;

    uint32_t last_can_message_received;
    bool        error_flag;
    DashError_t error;
    DashError_t last_error;
    uint8_t    current_page;        // 1=Main, 2=Sensori, 3=Powertrain, 4=Setup

    BMS_t bms;
    MCU_t mcu;
    IMU_t imu;
    LEM_CAB_t lem_cab;
    PDM_VCU1_t pdm_vcu1;
    PDM_VCU2_t pdm_vcu2;
    ECU_t ecu;
} CarData_t;

extern CarData_t car_state;


#endif /* INC_CAR_DATA_H_ */
