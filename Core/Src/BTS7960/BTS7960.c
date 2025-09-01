/*
 * BTS7960.c
 *
 *  Created on: Feb 8, 2025
 *      Author: RusselPC
 */

#include "BTS7960.h"
#include <stdio.h>

/* @Brief initialize BTS driver
 *
 * @param drv: pointer to the BTS7960_Error_te driver structure
 * @retval error status
 * */
BTS7960_Error_te BTS7960_Init(BTS7960_ts *drv){
	/*Check the valid parameters*/

	drv->enable = 0;
	drv->start = 0;
	drv->dir = BTS7960_DIR_NONE;
	drv->duty = 0;
	drv->error = BTS7960_ERR_OK;


	return BTS7960_ERR_OK;
}

/*@brief enable*/
BTS7960_Error_te BTS7960_Enable(BTS7960_ts *drv){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	drv->enable = 1;
	return BTS7960_ERR_OK;
}

/*@brief disable*/
BTS7960_Error_te BTS7960_Disable(BTS7960_ts *drv){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	drv->enable = 0;
	return BTS7960_ERR_OK;
}

/*@brief enable*/
BTS7960_Error_te BTS7960_Start(BTS7960_ts *drv){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}



	if(drv->dir != BTS7960_DIR_RIGHT && drv->dir != BTS7960_DIR_LEFT){
		return BTS7960_ERR_INVLD_DIR;
	}

	/* Turn off left and write enable
	 * Turn off left and write pwm
	 * */
	if(drv->start){
		return BTS7960_ERR_OK;
	}
//	BTS7960_Stop(drv);
	drv->rightPin(BTS7960_HIGH);
	drv->leftPin(BTS7960_HIGH);
	/*Delay to stable the MOSFET*/
	drv->delay_us(2);

	/*CHeck the direction*/
	switch (drv->dir) {
		case BTS7960_DIR_RIGHT:
			/* Enable right pin
			 * Start right PWM
			 * */
			if(drv->startPWM(BTS7960_DIR_RIGHT) == BTS7960_ERR_OK){
//				drv->rightPin(BTS7960_HIGH);
				drv->start = 1;
			}else{
				return BTS7960_ERR_START_PWM;
			}
			break;
		case BTS7960_DIR_LEFT:
			/* Enable left pin
			 * Start left PWM
			 * */
			if(drv->startPWM(BTS7960_DIR_LEFT) == BTS7960_ERR_OK){
//				drv->leftPin(BTS7960_HIGH);
				drv->start = 1;
			}else{
				return BTS7960_ERR_START_PWM;
			}
			break;
		default:
			return BTS7960_ERR_INVLD_DIR;
			break;
	}

	/*TODO: Write code later*/
	return BTS7960_ERR_OK;
}

BTS7960_Error_te BTS7960_Stop(BTS7960_ts *drv){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Turn off left and write enable*/
	drv->rightPin(BTS7960_LOW);
	drv->leftPin(BTS7960_LOW);

	/*Turn off left and write pwm*/
	BTS7960_Error_te error;
	error = drv->stopPWM(BTS7960_DIR_RIGHT);
	if(error == BTS7960_ERR_OK){
		error = drv->stopPWM(BTS7960_DIR_LEFT);
		drv->start = 0;
	}
	return error;
}


BTS7960_Error_te BTS7960_SetDir(BTS7960_ts *drv, BTS7960_Direction_te dir){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}if(dir != BTS7960_DIR_RIGHT && dir != BTS7960_DIR_LEFT){
		return BTS7960_ERR_INVLD_DIR;
	}

	/*Make sure the the BTS7960 driver is not running*/
	if(!drv->start){
		drv->dir = dir;
	}
	return BTS7960_ERR_OK;
}



BTS7960_Error_te BTS7960_SetDuty(BTS7960_ts *drv, float duty){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}if(!(duty >=0 && duty <= BTS7960_PWM_RESOLUTION)){
		return BTS7960_ERR_INVLD_SPEED;
	}
	drv->duty = duty;
	float dutyCycle = (duty*(BTS7960_PWM_RESOLUTION-drv->dutyOffset)/100.0)+drv->dutyOffset;
	drv->setDuty(drv, dutyCycle);

	return BTS7960_ERR_OK;
}


BTS7960_Error_te BTS7960_CalcPWM(BTS7960_ts *drv, float duty){
	/*Check the valid parameters*/
	if(drv == NULL){
		return BTS7960_ERR_NULL_PTR;
	}if(!(duty >0.0 && duty <= BTS7960_PWM_RESOLUTION)){
		return BTS7960_ERR_INVLD_DIR;
	}

	/* 1. duty <= period
	 *
	 * */
	return BTS7960_ERR_OK;

}




/*@brief Attach CW pin to the driver*/
BTS7960_Error_te BTS7960_AttachCWPin(BTS7960_ts *drv, void (*leftPin)(uint8_t status)){
	/*Check the valid parameters*/
	if(drv == NULL || leftPin == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Attach the functions*/
	drv->leftPin = leftPin;

	return BTS7960_ERR_OK;
}


/*@brief Attach CCW pin to the driver*/
BTS7960_Error_te BTS7960_AttachCCWPin(BTS7960_ts *drv, void (*rightPin)(uint8_t status)){
	/*Check the valid parameters*/
	if(drv == NULL || rightPin == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Attach the functions*/
	drv->rightPin = rightPin;

	return BTS7960_ERR_OK;
}


/*@brief Attach timer start to the driver*/
BTS7960_Error_te BTS7960_AttachTimerStartPWM(BTS7960_ts *drv, BTS7960_Error_te (*startPWM)(BTS7960_Direction_te dir)){
	/*Check the valid parameters*/
	if(drv == NULL || startPWM == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Attach the functions*/
	drv->startPWM = startPWM;

	return BTS7960_ERR_OK;
}


/*@brief Attach timer stop to the driver*/
BTS7960_Error_te BTS7960_AttachTimerStopPWM(BTS7960_ts *drv, BTS7960_Error_te (*stopPWM)(BTS7960_Direction_te dir)){
	/*Check the valid parameters*/
	if(drv == NULL || stopPWM == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Attach the functions*/
	drv->stopPWM = stopPWM;

	return BTS7960_ERR_OK;
}


/*@brief Attach timer set duty to the driver*/
BTS7960_Error_te BTS7960_AttachTimerSetDuty(BTS7960_ts *drv, BTS7960_Error_te (*setDuty)(struct BTS7960 *drv, float duty)){
	/*Check the valid parameters*/
	if(drv == NULL || setDuty == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Attach the functions*/
	drv->setDuty = setDuty;

	return BTS7960_ERR_OK;
}

/*@brief Attach timer set duty to the driver*/
BTS7960_Error_te BTS7960_AttachDelay(BTS7960_ts *drv, void (*delay_us)(uint32_t us)){
	/*Check the valid parameters*/
	if(drv == NULL || delay_us == NULL){
		return BTS7960_ERR_NULL_PTR;
	}
	/*Attach the functions*/
	drv->delay_us = delay_us;

	return BTS7960_ERR_OK;
}

