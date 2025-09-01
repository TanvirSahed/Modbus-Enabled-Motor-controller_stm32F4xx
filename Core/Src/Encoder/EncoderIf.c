/*
 * EncoderIf.c
 *
 *  Created on: Apr 15, 2025
 *      Author: wsrra
 */

#include "EncoderMain.h"
#include "app_main.h"
#include "MotorController.h"
#include "DM542T.h"

extern GlobalVar_ts gv;

Encoder_Error_te ENC0_StartLearningCB(void *ctx);
Encoder_Error_te ENC0_StopLearningCB(void *ctx);
Encoder_Error_te ENC0_CheckConfigCB(void *ctx);
Encoder_Error_te ENC0_ReadyToMeasureCB(void *ctx);
Encoder_Error_te ENC0_MeasureCpltCB(void *ctx);


Encoder_Error_te ENC1_StartLearningCB(void *ctx);
Encoder_Error_te ENC1_StopLearningCB(void *ctx);
Encoder_Error_te ENC1_CheckConfigCB(void *ctx);
Encoder_Error_te ENC1_ReadyToMeasureCB(void *ctx);
Encoder_Error_te ENC1_MeasureCpltCB(void *ctx);


void Encoder_Init_If(void){
//	gv.encHandler0.startLearningCB = ENC0_StartLearningCB;
//	gv.encHandler0.startLearningCB = ENC0_StartLearningCB;
//	gv.encHandler0.checkConfigCB = ENC0_CheckConfigCB;
//	gv.encHandler0.ReadyToMeasureCB = ENC0_ReadyToMeasureCB;
//	gv.encHandler0.MeasureCpltCB = ENC0_MeasureCpltCB;
//
//	gv.encHandler1.startLearningCB = ENC1_StartLearningCB;
//	gv.encHandler1.startLearningCB = ENC1_StartLearningCB;
//	gv.encHandler1.checkConfigCB = ENC1_CheckConfigCB;
//	gv.encHandler1.ReadyToMeasureCB = ENC1_ReadyToMeasureCB;
//	gv.encHandler1.MeasureCpltCB = ENC1_MeasureCpltCB;
}


Encoder_Error_te ENC0_StartLearningCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO:Write code here
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC0_StopLearningCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO:Write code here
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC0_CheckConfigCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO:Write code here
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC0_ReadyToMeasureCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

	//TODO:Write code herereturn ENCODER_ERR_NONE;
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC0_MeasureCpltCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

	//TODO:Write code herereturn ENCODER_ERR_NONE;
	return ENCODER_ERR_NONE;
}



Encoder_Error_te ENC1_StartLearningCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

	/* Stop Motor controller
	 * */
	gv.motoCtrlStpr.enable = 0;

	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC1_StopLearningCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO:Write code here
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC1_CheckConfigCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO:Write code here
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC1_ReadyToMeasureCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO:Write code here
	/* Enable the motor driver attached with the encoder
	 * Set speed
	 * Set dir
	 * Start the motor
	 * */
	DM542T_Enable(gv.dm542tDrv);
	DM542T_SetDir(gv.dm542tDrv, DM542T_DIR_CCW);
//	DM542T_SetRPM(gv.dm542tDrv, gv.encHandler1.speed);
	DM542T_StartPulse(gv.dm542tDrv);
	return ENCODER_ERR_NONE;
}

Encoder_Error_te ENC1_MeasureCpltCB(void *ctx){
	if(ctx == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
//	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

	//TODO:Write code here
	/* disable the motor driver attached with the encoder
	 * Stop the motor
	 * */
	DM542T_Disable(gv.dm542tDrv);
	DM542T_StopPulse(gv.dm542tDrv);
	return ENCODER_ERR_NONE;
}
