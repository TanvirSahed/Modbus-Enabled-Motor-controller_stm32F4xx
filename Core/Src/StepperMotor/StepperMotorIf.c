/*
 * StepperMotorIf.c
 *
 *  Created on: Dec 18, 2024
 *      Author: RusselPC
 */


#include "StepperMotor.h"
#include "main.h"

uint8_t SMD_ReadDownSen(void){
	return HAL_GPIO_ReadPin(ENC1_CHZ_IT15_GPIO_Port, ENC1_CHZ_IT15_Pin);
}


uint8_t SMD_ReadUpSen(void){
	return HAL_GPIO_ReadPin(STP_M_LIMIT_SW_IT10_GPIO_Port, STP_M_LIMIT_SW_IT10_Pin);
}
