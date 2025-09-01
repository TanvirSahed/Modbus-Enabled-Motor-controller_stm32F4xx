/*
 * DM542T_Interface.c
 *
 *  Created on: Mar 12, 2025
 *      Author: wsrra
 */

#include "DM542T_Interface.h"

#include "main.h"
#include "debug.h"

extern TIM_HandleTypeDef htim1;
#define DM542T_TIMER				(&htim1)
#define DM542T_TIMER_PWM_CHANNEL	TIM_CHANNEL_1
#define DM542T_EN_PORT 				STP_M_EN_GPIO_Port
#define DM542T_EN_PIN 				STP_M_EN_Pin
#define DM542T_DIR_PORT 			STP_M_DIR_GPIO_Port
#define DM542T_DIR_PIN 				STP_M_DIR_Pin



DM542T_Error_te dm542t_init_if(struct DM542T *drv);
void dm542t_enPin_if(uint8_t status);
void dm542t_dirPin_if(DM542T_Dir_te dir);
DM542T_Error_te dm542t_timerInit_if(struct DM542T *drv);
DM542T_Error_te dm542t_startPWM_if(void);
DM542T_Error_te dm542t_stopPWM_if(void);
DM542T_Error_te dm542t_setPeriod_if(struct DM542T *drv, uint32_t period);
uint32_t dm542t_getPrescaler_if(void);
uint32_t dm542t_getSysFreq_if(void);
void dm542t_delayUS_if(uint32_t us);



void DM542T_InitInterface(DM542T_ts *dm542t){
	dm542t->init = dm542t_init_if;
	dm542t->enPin = dm542t_enPin_if;
	dm542t->dirPin = dm542t_dirPin_if;
	dm542t->timerInit = dm542t_timerInit_if;
	dm542t->startPWM = dm542t_startPWM_if;
	dm542t->stopPWM = dm542t_stopPWM_if;
	dm542t->setTimerPeriod	= dm542t_setPeriod_if;
	dm542t->getTimerPrescaler = dm542t_getPrescaler_if;
	dm542t->getSysFreq = dm542t_getSysFreq_if;
	dm542t->delay_us = dm542t_delayUS_if;
}


DM542T_Error_te dm542t_init_if(struct DM542T *drv){
	return DM542T_ERR_OK;
}

void dm542t_enPin_if(uint8_t status){

	HAL_GPIO_WritePin(DM542T_EN_PORT, DM542T_EN_PIN,
			(status? GPIO_PIN_RESET : GPIO_PIN_SET));
//		dbg_print("DM542T: %s\r\n",(status? "Disable" : "Enabled"));
}

void dm542t_dirPin_if(DM542T_Dir_te dir){

	HAL_GPIO_WritePin(DM542T_DIR_PORT, DM542T_DIR_PIN,
			(dir == DM542T_DIR_CW ? GPIO_PIN_RESET : GPIO_PIN_SET));
//		dbg_print("DM542T: Direction %s\r\n",(dir == DM542T_DIR_CW? "CW" : "CCW"));
}

DM542T_Error_te dm542t_timerInit_if(struct DM542T *drv){
	//TODO: to implement later
	return DM542T_ERR_OK;
}

DM542T_Error_te dm542t_startPWM_if(void){

	if(HAL_TIM_PWM_Start(DM542T_TIMER, DM542T_TIMER_PWM_CHANNEL) != HAL_OK){
		dbg_print("DM542T Error: Start PWM Failed\r\n");
		return DM542T_ERR_PWM_START_FAILED;
	}
//		dbg_print("DM542T: Start PWM\r\n");
	return DM542T_ERR_OK;
}

DM542T_Error_te dm542t_stopPWM_if(void){

	if(HAL_TIM_PWM_Stop(DM542T_TIMER, DM542T_TIMER_PWM_CHANNEL) != HAL_OK){
		dbg_print("DM542T Error: Stop PWM Failed\r\n");
		return DM542T_ERR_PWM_STOP_FAILED;
	}
//		dbg_print("DM542T: Stop PWM\r\n");
	return DM542T_ERR_OK;
}

DM542T_Error_te dm542t_setPeriod_if(struct DM542T *drv, uint32_t period){

	__HAL_TIM_SET_AUTORELOAD(DM542T_TIMER, period);
	__HAL_TIM_SET_COUNTER(DM542T_TIMER, period/2);
//		dbg_print("DM542T: set period: %u\r\n", period);
	return DM542T_ERR_OK;
}

uint32_t dm542t_getPrescaler_if(void){

	return DM542T_TIMER->Init.Prescaler;
}

uint32_t dm542t_getSysFreq_if(void){
	return HAL_RCC_GetSysClockFreq();
}

void dm542t_delayUS_if(uint32_t us){
	/*It will make delay close to 5us*/
	for(uint32_t i = 0; i <= 70; i++){
		__NOP();
	}
}


