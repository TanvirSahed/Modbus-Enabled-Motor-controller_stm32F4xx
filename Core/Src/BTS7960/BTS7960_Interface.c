/*
 * BTS7960_Interface.c
 *
 *  Created on: Feb 8, 2025
 *      Author: RusselPC
 */
#include "BTS7960_Interface.h"
#include "debug.h"
#include "main.h"

/* L_EN and R_EN pin configuration
 * Note: This pins are overlapped with the
 * stepper motor enable and direction pins respectively
 * */
#define BTS7960_L_EN_PORT		HBRIDG_LEN_GPIO_Port
#define BTS7960_L_EN_PIN		HBRIDG_LEN_Pin
#define BTS7960_R_EN_PORT		HBRIDG_REN_GPIO_Port
#define BTS7960_R_EN_PIN		HBRIDG_REN_Pin
#define BTS7960_R_PWM_CH		TIM_CHANNEL_2
#define BTS7960_L_PWM_CH		TIM_CHANNEL_1
#define BTS7960_TIMER			htim4


extern TIM_HandleTypeDef BTS7960_TIMER;
TIM_OC_InitTypeDef sConfigOC = {0};


void BTS7960_LeftPin_If(uint8_t status);
void BTS7960_RightPin_If(uint8_t status);
BTS7960_Error_te BTS7960_StartPWM_If(BTS7960_Direction_te dir);
BTS7960_Error_te BTS7960_StopPWM_If(BTS7960_Direction_te dir);
BTS7960_Error_te BTS7960_SetDuty_If(struct BTS7960 *drv, float duty);
void BTS7960_DelayUS_If(uint32_t us);


BTS7960_Error_te BTS7860_InitTimer(void);


void BTS7960_Init_If(BTS7960_ts *drv){
	/*Check the parameters validity*/
	if(drv == NULL){
		dbg_print("BTS7960 ERROR: Driver null pointer!");
	}

	drv->leftPin = BTS7960_LeftPin_If;
	drv->rightPin = BTS7960_RightPin_If;
	drv->startPWM = BTS7960_StartPWM_If;
	drv->stopPWM = BTS7960_StopPWM_If;
	drv->setDuty = BTS7960_SetDuty_If;
	drv->delay_us = BTS7960_DelayUS_If;

//	BTS7860_InitTimer();
//	__HAL_TIM_SET_AUTORELOAD(BTS7960_TIMER, drv->period);
//	__HAL_TIM_SET_COMPARE(&BTS7960_TIMER,BTS7960_R_PWM_CH, 45);
}


void BTS7960_LeftPin_If(uint8_t status){
//	dbg_print("BTS7960: Left %s!\r\n",(status ? "Enabled" : "Disabled"));
	HAL_GPIO_WritePin(BTS7960_L_EN_PORT, BTS7960_L_EN_PIN,
			(status ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void BTS7960_RightPin_If(uint8_t status){
//	dbg_print("BTS7960: Right %s!\r\n",(status ? "Enabled" : "Disabled"));
	HAL_GPIO_WritePin(BTS7960_R_EN_PORT, BTS7960_R_EN_PIN,
			(status ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

BTS7960_Error_te BTS7960_StartPWM_If(BTS7960_Direction_te dir){
//	dbg_print("BTS7960: Start PWM [Dir:%d]\r\n", dir);
	switch (dir) {
		case BTS7960_DIR_RIGHT:
			HAL_TIM_PWM_Start(&BTS7960_TIMER, BTS7960_R_PWM_CH);
			break;
		case BTS7960_DIR_LEFT:
			HAL_TIM_PWM_Start(&BTS7960_TIMER, BTS7960_L_PWM_CH);
			break;
		default:
			return BTS7960_ERR_INVLD_DIR;
			break;
	}

	return BTS7960_ERR_OK;
}


BTS7960_Error_te BTS7960_StopPWM_If(BTS7960_Direction_te dir){
//	dbg_print("BTS7960: Stop PWM [Dir:%d]\r\n", dir);
	switch (dir) {
		case BTS7960_DIR_RIGHT:
			HAL_TIM_PWM_Stop(&BTS7960_TIMER, BTS7960_R_PWM_CH);
			break;
		case BTS7960_DIR_LEFT:
			HAL_TIM_PWM_Stop(&BTS7960_TIMER, BTS7960_L_PWM_CH);
			break;
		default:
			return BTS7960_ERR_INVLD_DIR;
			break;
	}
	return BTS7960_ERR_OK;
}

BTS7960_Error_te BTS7960_SetDuty_If(struct BTS7960 *drv, float duty){
	/*TODO: duty +=20;*/
//	duty +=20;

//	dbg_print("BTS7960: Set Duty %u, Period: %u\r\n",duty, BTS7960_TIMER.Init.Period);
	if(drv->start) {
		HAL_TIM_PWM_Stop(&BTS7960_TIMER, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&BTS7960_TIMER, TIM_CHANNEL_2);
	}

	duty = duty*((BTS7960_TIMER.Init.Period+1)/BTS7960_PWM_RESOLUTION);
//	dbg_print("BTS7960_If: Duty %0.2f, Period: %u\r\n",duty, BTS7960_TIMER.Init.Period);
	if(duty > BTS7960_TIMER.Init.Period){
		dbg_print("BTS7960 ERROR: Invalid Duty %u, Period: %u\r\n",duty,BTS7960_TIMER.Init.Period);
		return BTS7960_ERR_INVLD_VALUE;
	}
	HAL_StatusTypeDef ret = HAL_OK;
	sConfigOC.Pulse = duty;
	switch (drv->dir) {
		case BTS7960_DIR_RIGHT:
//			ret = HAL_TIM_PWM_ConfigChannel(&BTS7960_TIMER, &sConfigOC, BTS7960_R_PWM_CH);
			__HAL_TIM_SET_COMPARE(&BTS7960_TIMER,BTS7960_R_PWM_CH, duty);
			break;
		case BTS7960_DIR_LEFT:
//			ret = HAL_TIM_PWM_ConfigChannel(&BTS7960_TIMER, &sConfigOC,  BTS7960_L_PWM_CH);
			__HAL_TIM_SET_COMPARE(&BTS7960_TIMER,BTS7960_L_PWM_CH, duty);
			break;
		default:
			dbg_print("BTS7960 ERROR: Invalid Direction!\r\n");
			return BTS7960_ERR_INVLD_DIR;
			break;
	}

	if ( ret != HAL_OK)
	{
	  dbg_print("BTS7960 ERROR: Timer channel 1 config failed!\r\n");
	  return BTS7960_ERR_PWM_CH_CONFIG_FAILED;
	}

	if(drv->start) {
		if (BTS7960_StartPWM_If(drv->dir) != BTS7960_ERR_OK){
			dbg_print("BTS7960 ERROR: PWM Start failed!\r\n");
			return	BTS7960_ERR_START_PWM;
		}
	}
	return BTS7960_ERR_OK;
}

void BTS7960_DelayUS_If(uint32_t us){
	HAL_Delay(1);
}



BTS7960_Error_te BTS7860_InitTimer(void){

//	__HAL_TIM_SET_AUTORELOAD(drv->config.tim,drv->config.tim->Init.Period);
//	__HAL_TIM_SET_COMPARE(&BTS7960_TIMER,BTS7960_R_PWM_CH, 45);

//	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	  sConfigOC.Pulse = 0;//BTS7960_TIMER.Init.Period/2;
//	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
//	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//	  if (HAL_TIM_PWM_ConfigChannel(&BTS7960_TIMER, &sConfigOC, BTS7960_R_PWM_CH) != HAL_OK)
//	  {
//		  dbg_print("BTS7960 ERROR: Timer channel 1 config failed!");
//		  return BTS7960_ERR_PWM_CH_CONFIG_FAILED;
//	  }
//	  if (HAL_TIM_PWM_ConfigChannel(&BTS7960_TIMER, &sConfigOC, BTS7960_L_PWM_CH) != HAL_OK)
//	  {
//		  dbg_print("BTS7960 ERROR: Timer channel 2 config failed!");
//		  return BTS7960_ERR_PWM_CH_CONFIG_FAILED;
//	  }
	  return BTS7960_ERR_OK;
}

