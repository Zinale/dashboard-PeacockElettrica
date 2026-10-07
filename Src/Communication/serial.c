/*
 * serial.c
 *
 *  Created on: 22 gen 2026
 *      Author: zinga
 */
#include "serial.h"
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdarg.h"
#include "argz.h"

char uart2_tx_dma_buffer[UART2_TX_DMA_BUFFER_SIZE];
volatile uint8_t uart2_tx_busy = 0;



uint8_t cmd_end[3] = {0xFF,0xFF,0xFF};


/**
 * @brief Trasmette un messaggio formattato sull'interfaccia UART specificata.
 *
 * Questa funzione formatta una stringa utilizzando una lista di argomenti variabili
 * e la trasmette tramite l'interfaccia UART fornita.
 *
 * @param huart Puntatore alla struttura `UART_HandleTypeDef` che rappresenta l'interfaccia UART.
 * @param format Stringa di formato, simile a quella usata in `printf`.
 * @param ... Argomenti variabili da formattare nella stringa.
 */
void Display_Message(UART_HandleTypeDef *huart, const char *format, ...)
{
	if (huart==&huart2){
	    va_list args;
	    va_start(args, format);
		// Per UART2 usa DMA (non bloccante)
		// Aspetta che la trasmissione precedente sia completata
		while (uart2_tx_busy) { __NOP(); }

		int length = vsnprintf(uart2_tx_dma_buffer, UART2_TX_DMA_BUFFER_SIZE, format, args);
		va_end(args);

		if (length > 0 && length < UART2_TX_DMA_BUFFER_SIZE)
		{
			uart2_tx_busy = 1;
			HAL_UART_Transmit_DMA(huart, (uint8_t *)uart2_tx_dma_buffer, length);
		}
	}
}

void Nextion_Cmd(const char *fmt, ...)
{
	while (uart2_tx_busy) { __NOP(); }

	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(uart2_tx_dma_buffer,
	                    UART2_TX_DMA_BUFFER_SIZE - 3,
	                    fmt, args);
	va_end(args);

	if (len > 0 && len <= UART2_TX_DMA_BUFFER_SIZE - 3)
	{
		uart2_tx_dma_buffer[len]     = (char)0xFF;
		uart2_tx_dma_buffer[len + 1] = (char)0xFF;
		uart2_tx_dma_buffer[len + 2] = (char)0xFF;
		uart2_tx_busy = 1;
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart2_tx_dma_buffer, len + 3);
	}
}



