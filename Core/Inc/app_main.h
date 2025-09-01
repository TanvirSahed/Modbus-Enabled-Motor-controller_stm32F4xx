/*
 * app_main.h
 *
 *  Created on: Dec 5, 2024
 *      Author: RusselPC
 */

#ifndef INC_APP_MAIN_H_
#define INC_APP_MAIN_H_

#include "Encoder.h"
#include "DM542T.h"
#include "StepperMotor.h"
#include "PID_Controller.h"
#include "BTS7960.h"
#include "MotorController.h"
#include "EncoderMain.h"
#include "Registers.h"
#include "wizchip_conf.h"
#include "modbus_uart_manager.h"

#include "../Src/MotorSafety/pwmIF.h"


#define DBG_RX_BUFF_SIZE 100

typedef struct {
	uint16_t port;
	wiz_NetInfo netif;
	uint8_t applyChange;
}Modbus_TCPNetInfo_t;

typedef struct{
	SerialCommParams serial;
	uint8_t applyChange;
}Modbus_RTUSerialInfo_t;


typedef struct{
	uint32_t currentTime;
	uint32_t waitingTime;
	//uint8_t IsTimerElasped;
	uint8_t startCount;
}safetyTimer_t;

typedef struct{
	PWM_t pwm;
	uint32_t holdingDuty;
	uint32_t runningDuty;
	uint8_t applyChange;
	safetyTimer_t timer;
}MotorSafety_t;

typedef enum{
	MOTOR_START_STOP,
	STATE_MAX
}StateTracker_t;

typedef struct {
	uint32_t sysClk;
	uint8_t restart;

	/*Modbus TCP Server Net Info---*/
	Modbus_TCPNetInfo_t mbtcps;

	/*Modbus RTU Serial config*/
	Modbus_RTUSerialInfo_t mbrtu;

	modbus_reg_ts mbReg;

	Encoder_ts enc0;
	Encoder_ts enc1;
//	Encoder_Handler_ts encHandler0;
//	Encoder_Handler_ts encHandler1;

	/*DM542T Stepper Motor Driver*/
//	DM542TStpDrv_ts stpMotDrv;
	DM542T_ts *dm542tDrv;
	BTS7960_ts *bts7960Drv;

	MotorController_ts motoCtrlStpr; //TODO: for stepper

	MotorController_ts motoCtrlHB;

	StpMotDrv_ts smd;

	/*BTS7960-----*/

	uint8_t start;
	uint8_t dir;
	uint16_t speed;
	/*PID*/
	PID pid0;
	PID pid1;

	/*PWM generation*/
	MotorSafety_t safety;

	uint8_t stateTracker[STATE_MAX];


	/*For debug only*/
	uint8_t debugEnabe;
	uint8_t dbgRxBuf[DBG_RX_BUFF_SIZE];
	uint8_t dbgRxLen;
}GlobalVar_ts;




void APP_Init(void);
void APP_Main(void);
void poll_periodic(void);

#endif /* INC_APP_MAIN_H_ */
