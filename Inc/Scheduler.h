/*
 * Scheduler.h
 *
 *  Created on: 19 giu 2026
 *      Author: zinga
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "main.h"

/* ==========================================
 * TASK INTERVAL CONFIGURATION (in ms)
 * ========================================== */
#define INTERVAL_READ_INPUTS    20   // Read buttons, SR and rotary switch
#define INTERVAL_SEND_CAN       150  // Send parameters and engine map via CAN
#define INTERVAL_UPDATE_DISP    70  // Display data refresh

/* ========================================== */

void SchedulerInit(void);
void SchedulerTimerInterruptCallBack(void);
void SchedulerManagementFunction(void);


#endif /* INC_SCHEDULER_H_ */
