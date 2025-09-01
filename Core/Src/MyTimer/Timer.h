/*
 * Timer.h
 *
 *  Created on: Mar 22, 2025
 *      Author: wsrra
 */

#ifndef TIMER_H_
#define TIMER_H_
#include <stdint.h>

typedef struct Timer{
	uint8_t enable;
	uint64_t time;
	uint32_t timeout;
	uint32_t isTimeRst;
	uint64_t (*getTimeUs)(void);
}Timer_ts;

void Timer_Init(Timer_ts *timer);
void Timer_Start(Timer_ts *timer);
void Timer_Stop(Timer_ts *timer);
void Timer_Reset(Timer_ts *timer);
void Timer_SetTimeout(Timer_ts *timer, uint32_t timeout);
uint8_t Timer_IsRunning(Timer_ts *timer);
uint8_t Timer_IsTimeout(Timer_ts *timer);

void Timer_AttachTimeSource(Timer_ts *timer, uint64_t (*timeUs)(void));

#endif /* TIMER_H_ */
