/*
 * Scheduler.c
 *
 *  Created on: 19 giu 2026
 *      Author: zinga
 */

#include "Scheduler.h"
#include "Tasks.h" // File declaring Task_ReadInputs(), Task_SendCAN(), etc.

volatile uint32_t tickCounter = 0;

volatile uint8_t flag_task_read_inputs = 0;
volatile uint8_t flag_task_send_can = 0;
volatile uint8_t flag_task_update_disp = 0;

void SchedulerInit(void){
	// SysTick configured at 1 millisecond (HCLK / 1000)
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	tickCounter = 0;
	flag_task_read_inputs = 0;
	flag_task_send_can = 0;
	flag_task_update_disp = 0;

	TaskInit();
}

void SchedulerTimerInterruptCallBack(void){
	tickCounter++;

	// Evaluate flags based on intervals defined in header
	if(tickCounter % INTERVAL_READ_INPUTS == 0){
		flag_task_read_inputs = 1;
	}
	if(tickCounter % INTERVAL_SEND_CAN == 0){
		flag_task_send_can = 1;
	}
	if(tickCounter % INTERVAL_UPDATE_DISP == 0){
		flag_task_update_disp = 1;
	}

	// Counter reset:
	if(tickCounter >= 1000){
		tickCounter = 0;
	}
}

void SchedulerManagementFunction(void){

	// 1. Data Acquisition Task
	if(flag_task_read_inputs){
		flag_task_read_inputs = 0;
		Task_ReadInputs();
	}

	// 2. Transmission Task
	if(flag_task_send_can){
		flag_task_send_can = 0;
		Task_SendCAN();
	}

	// 3. Interface Task
	if(flag_task_update_disp){
		flag_task_update_disp = 0;
		Task_UpdateDisplay();
	}

}
