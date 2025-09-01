/*
 * LimitSensorIf.c
 *
 *  Created on: Mar 24, 2025
 *      Author: wsrra
 */

#include "MotorController.h"
#include "main.h"

#define DOWN_SEN_PORT		ENC1_CHZ_IT15_GPIO_Port
#define DOWN_SEN_PIN		ENC1_CHZ_IT15_Pin
#define UP_SEN_PORT			STP_M_LIMIT_SW_IT10_GPIO_Port
#define UP_SEN_PIN			STP_M_LIMIT_SW_IT10_Pin

MotorCtrl_Error_te LimitSenIf_ReadUpSen(MotorCtrl_Sensor_ts *sen);
MotorCtrl_Error_te LimitSenIf_ReadDownSen(MotorCtrl_Sensor_ts *sen);


MotorCtrl_LimitSenInterface_ts	limitSenInterface ={
		LimitSenIf_ReadUpSen,
		LimitSenIf_ReadDownSen
};




MotorCtrl_Error_te LimitSenIf_ReadDownSen(MotorCtrl_Sensor_ts *sen){

	sen->state = HAL_GPIO_ReadPin(DOWN_SEN_PORT, DOWN_SEN_PIN);
	 return MOTOR_CTRL_ERR_OK;
}


MotorCtrl_Error_te LimitSenIf_ReadUpSen(MotorCtrl_Sensor_ts *sen){
	sen->state = HAL_GPIO_ReadPin(UP_SEN_PORT, UP_SEN_PIN);
	return MOTOR_CTRL_ERR_OK;
}
