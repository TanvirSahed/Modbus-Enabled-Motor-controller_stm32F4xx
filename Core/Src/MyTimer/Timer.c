/*
 * Timer.c
 *
 *  Created on: Mar 22, 2025
 *      Author: wsrra
 */

#ifndef TIMER_C_
#define TIMER_C_
#include "Timer.h"



void Timer_Start(Timer_ts *timer){
	if(timer->timeout==0){
		timer->enable = 0;
		return;
	}
	timer->time = timer->getTimeUs();
	timer->enable = 1;
}

void Timer_Stop(Timer_ts *timer){
	timer->enable = 0;
	timer->isTimeRst = 0;
}

void Timer_Reset(Timer_ts *timer){
	timer->time = timer->getTimeUs();
	timer->isTimeRst = 1;
}

void Timer_SetTimeout(Timer_ts *timer, uint32_t timeout){
	timer->timeout = timeout;
}

uint8_t Timer_IsRunning(Timer_ts *timer){
	return timer->enable;
}

uint8_t Timer_IsTimeout(Timer_ts *timer){
	if(timer->enable == 0) return 0;
	uint64_t currentTIme = timer->getTimeUs();
	if((uint32_t)(currentTIme - timer->time) >= (uint32_t)timer->timeout){
//		timer->time = TS_GetUS(&timStamp);
		timer->isTimeRst = 0;
		return 1;
	}
	return 0;
}

void Timer_AttachTimeSource(Timer_ts *timer, uint64_t (*timeUs)(void)){
	timer->getTimeUs = timeUs;
}

#endif /* TIMER_C_ */
