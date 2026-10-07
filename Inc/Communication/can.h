/*
 * can.h
 *
 *  Created on: 22 gen 2026
 *      Author: zinga
 */

#ifndef INC_COMMUNICATION_CAN_H_
#define INC_COMMUNICATION_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "car_data.h"

/* =========================================================================
 *  CAN HARDWARE CONFIGURATION
 * ========================================================================= */
#define CAN_ENABLE       /* Enable CAN interface */
#define USE_MCP2551      /* MCP2551 Transceiver  */

/* =========================================================================
 *  CAN MESSAGE IDs  (format: CAR_<SOURCE>_<CONTENT>)
 * ========================================================================= */

/* --- ERRORS --- */
#define CAR_BMS_ERRORS          0x001
#define CAR_INV1_ERRORS         0x002
#define CAR_INV2_ERRORS         0x003
#define CAR_PDM_VCU2_ERRORS     0x004
#define CAR_PDM_VCU1_ERRORS     0x005
#define CAR_MCU_FLAGS           0x006

/* --- STATO BMS --- */
#define CAR_BMS_STATE           0x103
#define CAR_BMS_SDC_STATE       0x107

/* --- MCU / INVERTER --- */
#define CAR_MCU_FW              0x104
#define CAR_MCU_BRAKE_PRES      0x105
#define CAR_MCU_APPS_SAS        0x108
#define CAR_INV1_STATE          0x250
#define CAR_INV2_STATE          0x251

/* --- SOSPENSIONI / COOLING --- */
#define CAR_SOSP_FRONT          0x300
#define CAR_COOLANT_T           0x301
#define CAR_SOSP_REAR           0x303
#define CAR_RADIATOR_FAN_PERC   0x304

/* --- BMS MODULI --- */
#define CAR_BMS_V_MODS          0x200
#define CAR_BMS_T_MODS          0x201
#define CAR_BMS_FAN_SPEED       0x202

/* --- CORRENTE / LV --- */
#define CAR_CURRENT_BMS         0x3C2
#define CAR_LV_BATT             0x401

/* --- Dati Specifici di ogni Modulo --- */
#define CAR_BMS_MODULE_1_DATA       0x510
#define CAR_BMS_MODULE_2_DATA       0x511
#define CAR_BMS_MODULE_3_DATA       0x512
#define CAR_BMS_MODULE_4_DATA       0x513
#define CAR_BMS_MODULE_5_DATA       0x514
#define CAR_BMS_MODULE_6_DATA       0x515

/* --- PARAMETRI DASH (inviati dalla dash) --- */
#define CAR_DASH_R2D_MAP_PARAMS 0x106

/* =========================================================================
 *  STRUTTURA MESSAGGIO CAN GREZZO
 * ========================================================================= */
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} Can_Message_t;

/* =========================================================================
 *  PROTOTIPI FUNZIONI PUBBLICHE
 * ========================================================================= */

/** Inizializza il CAN, configura i filtri e abilita le notifiche RX */
void CanInit(void);

/** Trasmette un messaggio CAN (usa il transceiver configurato) */
void Transmit_CAN_Message(CAN_HandleTypeDef *hcan, uint32_t StdId,
                          uint32_t DLC, uint8_t *TxData);

/** Callback RX: decodifica il frame e aggiorna car_state */
void Receive_CAN_Message(CAN_HandleTypeDef *hcan);

/**
 * @brief Decodifica un frame CAN e aggiorna la struttura car_state.
 *        Da chiamare dalla callback RX con id, dati e DLC del frame ricevuto.
 */
void CAN_ParseFrame(uint32_t id, uint8_t *data, uint8_t dlc);

#endif /* INC_COMMUNICATION_CAN_H_ */

