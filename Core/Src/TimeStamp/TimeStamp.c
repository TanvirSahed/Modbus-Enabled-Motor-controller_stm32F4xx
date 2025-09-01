/*
 * MH_Timer.c
 *
 *  Created on: Mar 28, 2023
 *      Author: Mahmudul Hasan (Russell)
 *      
 */

#include "../TimeStamp/TimeStamp.h"

#include "debug.h"

#define TIMR_BUS_CLK()		(HAL_RCC_GetPCLK1Freq()*2)
#define htim 				(&htim14)

extern TIM_HandleTypeDef htim14;




typedef struct TimeStamp{
	uint64_t currentTime;
	uint32_t ovfCount;
	TS_Format tf;
}TimeStamp;

TimeStamp tm = {0};


static uint32_t mcuSysClk;
static double mcuSysClkFacktor;
static uint64_t lastCount;		// total count from the last power up

void TS_Init(void){
	mcuSysClk = TIMR_BUS_CLK();//HAL_RCC_GetSysClockFreq();
	mcuSysClkFacktor = (double)(((htim->Init.Prescaler+1.0f)*1000000.0f)/mcuSysClk);
	lastCount = 0;
//	dbg_print("htim:%p \r\n",htim);
//	dbg_print("htim_Prescaler:%ld \r\n",htim->Init.Prescaler);
//	dbg_print("mcuSysClk:%ld, mcuSysClk:%f, Prescaler:%ld\r\n",
//			mcuSysClk, mcuSysClkFacktor, htim->Init.Prescaler);
	TS_Start();
}
/* This function start the timer
 * */
void TS_Start(void){
	HAL_TIM_Base_Start_IT(htim);
}

/* This function start the timer
 * */
void TS_Stop(void){
	HAL_TIM_Base_Stop_IT(htim);
}


/* This function returns total microsecond
 * */
uint64_t TS_GetUS(void){
	tm.currentTime = (uint64_t)((htim->Instance->CNT + lastCount)*mcuSysClkFacktor);
	return tm.currentTime;
}

/* This function returns total millisecond
 * */
uint64_t TS_GetMS(void){
	return (TS_GetUS()/1000);

}

TS_Format TS_GetTime(void){
	TS_GetUS();
	tm.tf.sec = tm.currentTime/1000000;
	tm.tf.ms = (tm.currentTime - ((uint64_t)tm.tf.sec*1000000))/1000;
	tm.tf.us = (tm.currentTime)%1000;
	tm.tf.min = tm.tf.sec/60;
	tm.tf.hr = tm.tf.min/60;
	tm.tf.min = tm.tf.min%60;
	tm.tf.sec = tm.tf.sec%60;
	return tm.tf;
}



void TS_CatchOVF(void){
	tm.ovfCount++;
	lastCount =  (uint64_t)((uint64_t)htim->Init.Period * (uint64_t)tm.ovfCount);
}
