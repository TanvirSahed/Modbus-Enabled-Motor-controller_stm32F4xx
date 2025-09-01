/*
 * BTS7960DriverIf.c
 *
 *  Created on: Jun 19, 2025
 *      Author: wsrra
 */

#include "BTS7960DriverIf.h"
#include "BTS7960.h"
#include "MotorController.h"
#include "debug.h"



BTS7960_ts bts7960;

MotorCtrl_Error_te bts7960_enable(void);
MotorCtrl_Error_te bts7960_disable(void);
MotorCtrl_Error_te bts7960_start(void);
MotorCtrl_Error_te bts7960_stop(void);
MotorCtrl_Error_te bts7960_setDir(MotorCtrl_Dir_te dir);
MotorCtrl_Error_te bts7960_setSpeed(float speed);

MotorCtrl_DriverInterface_ts bts7960DrvIf = {
		bts7960_enable,
		bts7960_disable,
		bts7960_start,
		bts7960_stop,
		bts7960_setDir,
		bts7960_setSpeed
};

void BTS7960DrvIf_Init(void){
	bts7960.enable = 0;
	bts7960.start = 0;
	bts7960.dir = BTS7960_DIR_NONE;
	bts7960.rpm = 0;
	bts7960.duty = 0;
	bts7960.period = 0;
	bts7960.error = BTS7960_ERR_OK;

//	BTS7960_InitInterface(&bts7960);
}


MotorCtrl_Error_te bts7960_enable(void){

	if( BTS7960_Enable(&bts7960) != BTS7960_ERR_OK){
		dbg_print("BTS7960 Error: Enable Failed!");
		return MOTOR_CTRL_ERR;
	}
//	dbg_print("BTS7960: Enabled!");
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te bts7960_disable(void){
	if( BTS7960_Disable(&bts7960) != BTS7960_ERR_OK){
		dbg_print("BTS7960 Error: Disable Failed!\r\n");
		return MOTOR_CTRL_ERR;
	}
//	dbg_print("BTS7960: Disabled!\r\n");
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te bts7960_start(void){
	if( BTS7960_Start(&bts7960) != BTS7960_ERR_OK){
		dbg_print("BTS7960 Error: Start Failed!\r\n");
		return MOTOR_CTRL_ERR;
	}
//	dbg_print("BTS7960: Started!\r\n");
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te bts7960_stop(void){
	if( BTS7960_Stop(&bts7960) != BTS7960_ERR_OK){
		dbg_print("BTS7960 Error: Stop Failed!\r\n");
		return MOTOR_CTRL_ERR;
	}
//	dbg_print("BTS7960: Stoped!\r\n");
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te bts7960_setDir(MotorCtrl_Dir_te dir){
	if( BTS7960_SetDir(&bts7960, dir) != BTS7960_ERR_OK){
		dbg_print("BTS7960 Error: Set Direction Failed!\r\n");
		return MOTOR_CTRL_ERR;
	}
//	dbg_print("BTS7960: Direction Set Successfully!\r\n");
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te bts7960_setSpeed(float speed){
	/*TODO: speed*10*/
	BTS7960_Error_te ret = BTS7960_SetDuty(&bts7960, speed);
	if( ret != BTS7960_ERR_OK){
		dbg_print("BTS7960 Error: Set Speed Failed (%d), speed:%0.2f\r\n",ret,speed);
		return MOTOR_CTRL_ERR;
	}
//	dbg_print("BTS7960: Speed Set Successfully!\r\n");
	return MOTOR_CTRL_ERR_OK;
}
