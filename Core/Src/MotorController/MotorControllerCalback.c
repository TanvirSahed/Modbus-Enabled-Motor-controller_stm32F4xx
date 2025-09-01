/*
 * MotorControllerCalback.c
 *
 *  Created on: Mar 26, 2025
 *      Author: wsrra
 */


#include "MotorControllerIf.h"
#include "Encoder.h"
#include "app_main.h"
#include "debug.h"

extern GlobalVar_ts gv;

void downPosRechdCallback(void *ctx);
void upPosRechdCallback(void *ctx);
void midPosRechdCallback(void *ctx);

MotorCtrl_Callback_ts motoCtrlCallback = {
		downPosRechdCallback,
		upPosRechdCallback,
		midPosRechdCallback
};


void downPosRechdCallback(void *ctx){
	dbg_print("MC: down pos callback\r\n");
//	Encoder_SetCount(&gv.enc1, 0);
//	gv.enc1.posAngle = 0;
}

void upPosRechdCallback(void *ctx){
	dbg_print("MC: up pos callback\r\n");
}

void midPosRechdCallback(void *ctx){
	dbg_print("MC: mid pos callback\r\n");
}
