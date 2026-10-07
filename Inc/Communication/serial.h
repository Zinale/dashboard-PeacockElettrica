
/*
 * serial.h
 *
 *  Created on: 22 gen 2026
 *      Author: zinga
 */

#ifndef INC_COMMUNICATION_SERIAL_H_
#define INC_COMMUNICATION_SERIAL_H_

#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "string.h"
#define UART2_TX_DMA_BUFFER_SIZE 256


extern uint8_t cmd_end[3];

extern UART_HandleTypeDef huart2;
extern char uart2_tx_dma_buffer[UART2_TX_DMA_BUFFER_SIZE];
extern volatile uint8_t uart2_tx_busy;


// Serial communication function
void Display_Message(UART_HandleTypeDef *huart, const char *format, ...);

// Nextion display: send command + terminator 0xFF 0xFF 0xFF in a single DMA transfer
void Nextion_Cmd(const char *fmt, ...);


#endif /* INC_COMMUNICATION_SERIAL_H_ */
