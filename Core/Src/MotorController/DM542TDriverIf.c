/*
 * DM542TDriverIf.c
 *
 *  Created on: Mar 23, 2025
 *      Author: wsrra
 */
#include "DM542TDriverIf.h"
#include "MotorController.h"
#include "DM542T.h"

DM542T_ts dm542t;

MotorCtrl_Error_te dm542t_enable(void);
MotorCtrl_Error_te dm542t_disable(void);
MotorCtrl_Error_te dm542t_start(void);
MotorCtrl_Error_te dm542t_stop(void);
MotorCtrl_Error_te dm542t_setDir(MotorCtrl_Dir_te dir);
MotorCtrl_Error_te dm542t_setSpeed(float speed);

MotorCtrl_DriverInterface_ts dm542tDrvIf = {
		dm542t_enable,
		dm542t_disable,
		dm542t_start,
		dm542t_stop,
		dm542t_setDir,
		dm542t_setSpeed
};

void DM452TDrvIf_Init(void){
	dm542t.enable = 0;
	dm542t.start = 0;
	dm542t.dir = DM542T_DIR_NONE;
	dm542t.posAngle = 0;
	dm542t.isChanged = 0;
	dm542t.isChanged = 0;
	dm542t.fault = 0;
	dm542t.faultClear = 0;
//	DM542T_InitInterface(&dm542t);
}


MotorCtrl_Error_te dm542t_enable(void){

	if( DM542T_Enable(&dm542t) != DM542T_ERR_OK){
		return MOTOR_CTRL_ERR;
	}
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te dm542t_disable(void){
	if( DM542T_Disable(&dm542t) != DM542T_ERR_OK){
		return MOTOR_CTRL_ERR;
	}
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te dm542t_start(void){
	if( DM542T_StartPulse(&dm542t) != DM542T_ERR_OK){
		return MOTOR_CTRL_ERR;
	}
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te dm542t_stop(void){
	if( DM542T_StopPulse(&dm542t) != DM542T_ERR_OK){
		return MOTOR_CTRL_ERR;
	}
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te dm542t_setDir(MotorCtrl_Dir_te dir){
//	if(dir == MOTOR_CTRL_DIR_UP) dir = MOTOR_CTRL_DIR_DOWN;
//	else if(dir == MOTOR_CTRL_DIR_DOWN) dir = MOTOR_CTRL_DIR_UP;


	if( DM542T_SetDir(&dm542t, dir) != DM542T_ERR_OK){
		return MOTOR_CTRL_ERR;
	}
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te dm542t_setSpeed(float speed){
	if( DM542T_SetRPM(&dm542t, speed) != DM542T_ERR_OK){
		return MOTOR_CTRL_ERR;
	}
	return MOTOR_CTRL_ERR_OK;
}
