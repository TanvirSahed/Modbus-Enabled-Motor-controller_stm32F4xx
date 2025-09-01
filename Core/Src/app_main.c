/*
 * app_main.c
 *
 *  Created on: Dec 5, 2024
 *      Author: RusselPC
 */
#include "app_main.h"
#include "main.h"
#include "Encoder.h"
#include "debug.h"
#include "Settings.h"
#include "Config.h"

#include "stm32f4xx_hal.h"
#include "string.h"

#include "BTS7960.h"
#include "BTS7960_Interface.h"
#include "DM542T_Interface.h"
#include "DM542TDriverIf.h"
#include "MotorController.h"
#include "DM542T.h"
#include "Timer.h"
#include "TimeStamp.h"
#include "w25qxx_flash.h"

#include "flash_addrs.h"



extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim10;

GlobalVar_ts gv;

extern BTS7960_ts bts7960;
extern DM542T_ts dm542t;
extern MotorCtrl_DriverInterface_ts dm542tDrvIf;
extern MotorCtrl_LimitSenInterface_ts	limitSenInterface;
extern MotorCtrl_Callback_ts motoCtrlCallback;

void Log_Priodic(void);
void APP_RestartDevice(void);

void APP_Init(void){

	Settings_PreInit();
	Settings_Update();
	Settings_Init();





//htim1.Instance->CCR1 = 10;

//	gv.dm542tDrv = &dm542t;
//
//	DM542T_InitInterface(gv.dm542tDrv);
//	DM542T_Init(gv.dm542tDrv);

//	DM452TDrvIf_Init();
//	MotorCtrl_SetMotorDriverInterface(&gv.motoCtrlStpr, &dm542tDrvIf);
//	MotorCtrl_SetLimitSensorInterface(&gv.motoCtrlStpr, &limitSenInterface);
//	MotorCtrl_Init(&gv.motoCtrlStpr);
//	MotorCtrl_AttachCallback(&gv.motoCtrlStpr, &motoCtrlCallback);
//	gv.motoCtrlStpr.enable = 1;
//	gv.motoCtrlStpr.maxPos = ANGLE_MAX;	//TODO: for testing
//	gv.motoCtrlStpr.setPos = 0;
//	gv.motoCtrlStpr.setPosLast = 0;
//	gv.motoCtrlStpr.maxSpeed = 300;
////	DM542T_SetMotorStepAngle(gv.dm542tDrv, 0.9);
////	DM542T_SetDriverMicrostep(gv.dm542tDrv, 16);
////	DM542T_SetRPM(gv.dm542tDrv, 0);
////	Timer_Init(&gv.motoCtrlStpr.timer);
//
//	Timer_AttachTimeSource(&gv.motoCtrlStpr.timer, TS_GetUS);
//	Timer_SetTimeout(&gv.motoCtrlStpr.timer, 2000);
//
//
//	SMD_Init(&gv.smd);
//	gv.smd.enable = 1;

//	gv.smd.dir = SMD_DIR_DOWN;
//	gv.smd.inputRpm = 10;





//	HAL_GPIO_WritePin(STP_M_EN_GPIO_Port, STP_M_EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(STP_M_DIR_GPIO_Port, STP_M_DIR_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	HAL_Delay(1);
//	DM542T_SetRPM(&gv.smd.drv, 10);
//	__HAL_TIM_SET_AUTORELOAD(gv.stpMotDrv.config.tim, 840);
//	__HAL_TIM_SET_COUNTER(gv.stpMotDrv.config.tim, 840/2);


	/*BTS7960 HBridge Motor driver init*/
//	dbg_print("BTS7960: Init\r\n");
//	BTS7960_Init(&gv.bts7960);
//	BTS7960_Init_If(&gv.bts7960);
//	BTS7960_Enable(&gv.bts7960);
//	gv.start = 0;
//	gv.dir = 0;
//	gv.speed = 0;
	//flash_cleanup();

	/*Debug*/
	memset(gv.dbgRxBuf, 0, DBG_RX_BUFF_SIZE);
	gv.dbgRxLen = 0;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, gv.dbgRxBuf, DBG_RX_BUFF_SIZE);



//	gv.pid1.enable = 1;
//	gv.pid1.setpoint = 0;
//	gv.pid1.kp = 0.05;
//	gv.pid1.ki = 0.01;
//	gv.pid1.kd = 0.01;
//	gv.pid1.output_i_max = 1.0;
//

	/*Encoder 1*/
//	gv.enc1.enable = 1;
//	gv.encHandler1.startLearning = 1;

}

void APP_Main(void){



	/*Encoder------------------------------------*/
	Encoder_Update(&gv.enc1);


	/*PID------------------------------------*/
//	if(gv.encHandler1.mode == ENC_SM_MODE_OPERATE){
		float pos = (gv.motoCtrlStpr.path==MOTOR_CTRL_PATH_NORMAL)?gv.enc1.posAngle: (360 - gv.enc1.posAngle); // gv.enc1.posAngle;//
		gv.pid1.measurement = pos;//gv.enc1.posAngle;
		//

		PID_Controller(&gv.pid1);

		/*Map pid with motor controller*/
		if(gv.motoCtrlStpr.isStartUp == 0){
			gv.motoCtrlStpr.speed = (gv.pid1.output_pid*300/100);

			gv.motoCtrlStpr.pos =  pos;//gv.enc1.posAngle;
			MotorCtrl_MoveToPos(&gv.motoCtrlStpr,
					//gv.enc1.posAngle,
					pos,
					gv.motoCtrlStpr.speed,
					1);
		}

		/*Motor angular position learning'
		 * Active only in learning stage
		 * */
		if(gv.motoCtrlStpr.isLearnOpenPos && gv.motoCtrlStpr.sm.state == MOTOR_CTRL_SM_STATE_UP_POS_5){
			gv.motoCtrlStpr.maxPos = pos;//gv.enc1.posAngle;
			dbg_print("maxPos:%0.2f\r\n",gv.motoCtrlStpr.maxPos);
		}

		/*Need to clear glitch of encoder readings in the down position*/
		if(gv.motoCtrlStpr.sm.state == MOTOR_CTRL_SM_STATE_DOWN_POS_3 &&
				gv.enc1.count > 0)
		{
			if(gv.motoCtrlStpr.path == MOTOR_CTRL_PATH_NORMAL)
				Encoder_SetCount(&gv.enc1, 0);
			else{
				Encoder_SetCount(&gv.enc1, gv.enc1.ppr); // added
			}

		}
//	}

		/*DM542T Stepper motor driver--------------*/
	if(gv.motoCtrlStpr.type == MOTOR_CTRL_TYPE_STEPPER){
		DM542T_Update(gv.dm542tDrv);
	}

	/*Motro controller-----------------------------*/
	MotorCtrl_Run(&gv.motoCtrlStpr);



	/*Restart device--------------------------------*/
	APP_RestartDevice();


	/*Update flash storage on change event--------------*/
	if(flash_flushOnChange() == 0){
		dbg_print("Flash: Update Success!\r\n");
	}

	/*General purpose loop for testing purpose------------*/
	static uint32_t tick = 0;
	if((HAL_GetTick() - tick) >= 5000){
		tick = HAL_GetTick();
		HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin);

		static uint8_t dir = 1;
		if(dir == 1){
			dir = 2;
		}else{
			dir = 1;
		}
//		BTS7960_Stop(&bts7960);
//		HAL_Delay(500);
//		BTS7960_SetDir(&bts7960, dir);
//		BTS7960_SetDuty(&bts7960, 3);
//		BTS7960_Start(&bts7960);


//		DM542T_StopPulse(&dm542t);
//		DM542T_SetDir(&dm542t, dir);
//		DM542T_SetRPM(&dm542t, 150);
//		DM542T_StartPulse(&dm542t);

	}


	poll_periodic();
	Log_Priodic();
}



void Log_Priodic(void){
	static uint32_t tick = 0;
	if((HAL_GetTick() - tick) >= 5000){
		tick = HAL_GetTick();
		dbg_print("\r\n");
		dbg_print("RPM:%0.3f, rpm(z):%0.3f\r\n",gv.enc1.rpm, gv.enc1.rpmFromZ);

			dbg_print("sysClk:%ld\r\n",HAL_RCC_GetSysClockFreq());
//			dbg_print("ENCH: {EN:%d, st:%d, evt:%d, startLrn:%d, mode:%d, resTlrnc:%d,"
//					"isRdyeMsur:%d, smplCnt:%d, smplMax:%d,rtry:%d, rtryMax:%d, isFlt:%d, fltClr:%d}\r\n",
//					gv.encHandler1.enable, gv.encHandler1.sm.state, gv.encHandler1.sm.event,
//					gv.encHandler1.startLearning, gv.encHandler1.mode, gv.encHandler1.resuTolrnc,
//					gv.encHandler1.isReadyToMeasure,gv.encHandler1.sampleCount, gv.encHandler1.sampleMax,
//					gv.encHandler1.retry, gv.encHandler1.retryMax, gv.encHandler1.isFault, gv.encHandler1.clearFault
//					);
			dbg_print("ENC1: {count:%ld, lastCount:%ld, countZ:%ld, dir:%d, freq:%0.3f, "
					"rpm:%0.3f, rpm(Z):%0.3f, resolution:%d, ppr:%d, posAngle:%0.3f, "
					"rpmSet:%0.3f, refClk:%0.3f, diff:%d}\r\n",
					gv.enc1.count, gv.enc1.lastCount, gv.enc1.countZ, gv.enc1.dir, gv.enc1.chAInCap.freq,
					gv.enc1.rpm, gv.enc1.rpmFromZ, gv.enc1.resolution, gv.enc1.ppr, gv.enc1.posAngle,
					gv.enc1.rpmSet,	gv.enc1.chAInCap.refClk,gv.enc1.chAInCap.diff
					);
		dbg_print("PID_1: { MOD: %d, SP: %0.3f, FB: %0.3f, Err: %0.3f, Kp: %0.3f, ki: %0.3f, kd: %0.3f, "
				"p-out: %0.3f, i-out: %0.3f, d-out: %0.3f, out_i_Max: %0.3f, PID: %0.3f }\r\n",
				gv.pid1.outputMode, gv.pid1.setpoint, gv.pid1.measurement, gv.pid1.error,
				gv.pid1.kp,  gv.pid1.ki, gv.pid1.kd, gv.pid1.output_p, gv.pid1.output_i,
				gv.pid1.output_d,gv.pid1.output_i_max, gv.pid1.output_pid);
		//			dbg_print("PID-%d: { En: %d, Src: %d, outMod: %d, hyst: %d, "
		//						"Set: %0.2f, MaxSet: %0.2f, Mesure't: %0.2f, Kp: %0.2f, Ki: %0.2f, Kd: %0.2f, "
		//						"OutMax: %d, OutMin: %d, "
		//						"P-out: %0.2f, I-out: %0.2f, D-out: %0.2f, PID-out: %d }\r\n",
		//						gv.pid1.id, gv.pid1.enable, gv.pid1.fbSource, gv.pid1.outputMode, gv.pid1.hystValue,
		//						gv.pid1.setpoint,gv.pid1.maxSetpoint, gv.pid1.measurement,  gv.pid1.kp, gv.pid1.ki,	gv.pid1.kd,
		//						 gv.pid1.maxOutput, gv.pid1.minOutput,
		//						gv.pid1.output_p, gv.pid1.output_i, gv.pid1.output_d, gv.pid1.output_pid);

		dbg_print("DM542T: {EN:%d, Start:%d, Dir:%d, RPM:%d, rpmRef:%d, uSteps:%d, ppr:%d, "
				"stepAngle:%0.2f, stpePerRev:%d}\r\n",
				gv.dm542tDrv->enable, gv.dm542tDrv->start, gv.dm542tDrv->dir,
				gv.dm542tDrv->rpm, gv.dm542tDrv->refRpm,
				gv.dm542tDrv->drvConfig.microsteps, gv.dm542tDrv->drvConfig.ppr,
				gv.dm542tDrv->motorConfig.stepAngle, gv.dm542tDrv->motorConfig.stepsPerRev);

		dbg_print("MC:: {ST:%d, startUp:%d, EN:%d, Start:%d, "
				"Dir:%d, speed:%0.2f, startUpSpeed:%0.2f, pos:%0.2f, setPos:%0.2f, maxPos:%0.2f, Fault: %d, "
				"downSen:{state:%d, isTrigd:%d}, upSen:{state:%d, isTrigd:%d}}\r\n",
				gv.motoCtrlStpr.sm.state, gv.motoCtrlStpr.isStartUp, gv.motoCtrlStpr.enable, gv.motoCtrlStpr.start,
				gv.motoCtrlStpr.dir, gv.motoCtrlStpr.speed, gv.motoCtrlStpr.learningSpeed,
				gv.motoCtrlStpr.pos, gv.motoCtrlStpr.setPos, gv.motoCtrlStpr.maxPos, gv.motoCtrlStpr.fault,
				gv.motoCtrlStpr.downSen.state, gv.motoCtrlStpr.downSen.isTriggered,
				gv.motoCtrlStpr.upSen.state, gv.motoCtrlStpr.upSen.isTriggered,
				gv.motoCtrlStpr.fault, gv.motoCtrlStpr.faultClear);
		dbg_print("MC2: {TimeOut: %u}\n\r",gv.motoCtrlStpr.timer.timeout);

//		dbg_print("DM542T: {ST:%d, EN:%d, Start:%d, Dir:%d, setRPM:%d, Fault: %d, Period:%d, Prescaler:%d}\r\n",
//				gv.smd.drv.state, gv.smd.drv.enable, gv.smd.drv.start, gv.smd.drv.dir, gv.smd.drv.rpm, gv.smd.drv.fault,
//				gv.smd.drv.timerConfig.period, gv.smd.drv.timerConfig.prescaler);
//		dbg_print("StpMD: {ST:%d, startUp:%d, EN:%d, Start:%d, Dir:%d, Trig:%d, "
//				"pos:%d, setPos:%d, refRpm:%d, inRpm: %d, "
//				"downSen:%d, downTrig:%d, UpSen:%d upTrig:%d}\r\n",
//				gv.smd.sm.state, gv.smd.isStartingUp, gv.smd.enable, gv.smd.start, gv.smd.dir, gv.smd.sm.trigger,
//				gv.smd.pos, gv.smd.setPos, gv.smd.refRpm, gv.smd.inputRpm,
//				gv.smd.downSenState,gv.smd.downSenTrig, gv.smd.upSenState,gv.smd.upSenTrig);
	}
}


void APP_RestartDevice(void){
	if(gv.restart){
		gv.restart = 0;
		NVIC_SystemReset();
	}

}

void poll_periodic(void){


	if(gv.stateTracker[MOTOR_START_STOP]!= gv.motoCtrlStpr.start){

		gv.stateTracker[MOTOR_START_STOP]= gv.motoCtrlStpr.start;

		if(gv.motoCtrlStpr.start)
			PWM_SetDutyCycle(&gv.safety.pwm, gv.safety.runningDuty);
		else{

			gv.safety.timer.startCount = 1;
			gv.safety.timer.currentTime = HAL_GetTick();
			//PWM_SetDutyCycle(&gv.safety.pwm, gv.safety.holdingDuty);
		}
	}

	if(gv.safety.timer.startCount){

		if( (HAL_GetTick() - gv.safety.timer.currentTime)
				>= gv.safety.timer.waitingTime){

			gv.safety.timer.startCount = 0;
			gv.safety.timer.currentTime = HAL_GetTick();
			PWM_SetDutyCycle(&gv.safety.pwm, gv.safety.holdingDuty);
		}

	}


	if(gv.mbtcps.applyChange){
		modbus_tcp_update_netIfs();
		gv.mbtcps.applyChange = 0;
		flash_write(FLS_ADDR_MB_TCPS_APPLY_CHANGE, gv.mbtcps.applyChange);
	}

	if(gv.mbrtu.applyChange){
		modbus_rtu_update_serialConfig();
		gv.mbrtu.applyChange = 0;
		flash_write(FLS_ADDR_MB_RTU_APPLY_CHANGE, gv.mbrtu.applyChange);
	}

	if(gv.safety.applyChange){
		if(gv.motoCtrlStpr.start)
		PWM_SetDutyCycle(&gv.safety.pwm, gv.safety.runningDuty);
		else
		PWM_SetDutyCycle(&gv.safety.pwm, gv.safety.holdingDuty);
	}
}

