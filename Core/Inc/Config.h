/*
 * Config.h
 *
 *  Created on: Dec 9, 2024
 *      Author: RusselPC
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
#include "Define.h"
/*-----------------------------------
  |		Default Configuration		|
  -----------------------------------*/
/*Utilities--------------------------------------*/
#define CONF_DEBUG_ENABLE							1

/*Encoder 1--------------------------------------*/
#define CONF_ENC1_RESOLUTION							2000//ENC_RESOLUTION //



/*Stepper motor driver---------------------------*/
#define CONF_DEF_STEPPER_MOTOR_STEPANGLE			0.9F // degree per step
#define CONF_DEF_STEPPER_MOTOR_MAX_RPM					300

/*DM542T stepper motor driver-----------------------*/
#define CONF_DEF_DM542T_MICROSTEP					16

/*HBridge motor---------------------------*/
#define CONF_DEF_BTS79600_DUTY_OFFSET				7.0F


/*Motor controller----------------------------------*/
#define MOTR_CTRL_PATH_CW							2
#define MOTR_CTRL_PATH_CCW							1

/*Stepper Motor controller----------------------------------*/
#define CONF_DEF_STPRCTRL_ENABLE					1
#define CONF_DEF_STPRCTRL_MAX_ANGLE					180.00F	// degree
#define CONF_DEF_STPRCTRL_MAX_SPEED					300.00F // rpm
#define CONF_DEF_STPRCTRL_CALIBRATION_SPEED			5.0f//1.5f // in rpm
#define CONF_DEF_STPRCTRL_WAITING_TIMEOUT			2000 // in us
#define CONF_DEF_STPRCTRL_WAITING_TIMEOUT_MIN		100 // in us
#define CONF_DEF_STPRCTRL_WAITING_TIMEOUT_MAX		100000000 // in us
#define CONF_DEF_STPRCTRL_PATH						MOTR_CTRL_PATH_CW
//#define CONF_DEF_MOTOCTRL_TYPE						MOTOR_CTRL_TYPE_STEPPER

/*HB Motor Controller*/
#define CONF_DEF_HBCTRL_ENABLE					1
#define CONF_DEF_HBCTRL_MAX_ANGLE					180.00F	// degree
#define CONF_DEF_HBCTRL_MAX_SPEED					300.00F // rpm
#define CONF_DEF_HBCTRL_CALIBRATION_SPEED			5.0f//1.5f // in rpm
#define CONF_DEF_HBCTRL_WAITING_TIMEOUT				2000 // in us
#define CONF_DEF_HBCTRL_WAITING_TIMEOUT_MIN			100 // in us
#define CONF_DEF_HBCTRL_WAITING_TIMEOUT_MAX			100000000 // in us
#define CONF_DEF_HBCTRL_PATH						MOTR_CTRL_PATH_CW

/*HB motor pid------------------------------*/
#define CONF_DEF_PID0_ENABLE	  					1
#define CONF_DEF_PID0_SETPOINT	  					0
#define CONF_DEF_PID0_OUT_MODE  					PID_OM_PERCENT
#define CONF_DEF_PID0_KP							0.05F
#define CONF_DEF_PID0_KI							0.01F
#define CONF_DEF_PID0_KD							0.01F
#define CONF_DEF_PID0_I_OUT_MAX						1.0F

/*Stepper motor pid------------------------------*/
#define CONF_DEF_PID1_ENABLE	  					1
#define CONF_DEF_PID1_SETPOINT	  					0
#define CONF_DEF_PID1_OUT_MODE  					PID_OM_PERCENT
#define CONF_DEF_PID1_KP							0.05F
#define CONF_DEF_PID1_KI							0.01F
#define CONF_DEF_PID1_KD							0.01F
#define CONF_DEF_PID1_I_OUT_MAX						1.0F


/*Modbus TCP Server Net Info----------------------*/
#define CONF_DEF_MB_TCPS_PORT						502
#define CONF_DEF_MB_TCPS_MAC						{0x80, 0x81, 0x82, 0x83, 0x84, 0x85}	// default mac address
#define CONF_DEF_MB_TCPS_IP							{192, 168, 0, 14}	//{192U, 168U, 0U, 110U}			// default server ip address
#define CONF_DEF_MB_TCPS_SN							{255U, 255U, 255U, 0U}			// default subnet mask
#define CONF_DEF_MB_TCPS_GW							{192, 168, 0, 1 }			// default gateway
#define CONF_DEF_MB_TCPS_DNS						{8U, 8U, 8U, 8U}				// default DNS
#define CONF_DEF_MB_TCPS_DHCP						NETINFO_STATIC
#define CONF_DEF_MB_TCPS_APPLY_CHANGE				0
#define CONF_DEF_IP_OCTATE 							4

/*Modbus RTU Serial Communication DEF Params----------------------*/
#define CONF_DEF_MB_RTU_BAUD						115200
#define CONF_DEF_MB_RTU_DATA_BIT					8
#define CONF_DEF_MB_RTU_STOP_BIT					1
#define CONF_DEF_MB_RTU_PARITY						'N'
#define CONF_DEF_MB_RTU_APPLY_CHANGE				0
#define CONF_DEF_MB_RTU_UART_IDX					0

/* PWM DEF Params----------------------*/
#define CONF_DEF_PWM_FREQ							20000
#define CONF_DEF_PWM_INITIAL_DUTY					50
#define CONF_DEF_PWM_PORT							GPIOB
#define CONF_DEF_PWM_PIN							GPIO_PIN_8
#define CONF_DEF_MOTOSFTY_HOLDING_DUTY				1000
#define CONF_DEF_MOTOSFTY_RUNNING_DUTY				8000
#define CONF_DEF_MOTOSFTY_WAITING_TIME_MS			100

//.timer = &htim10,
//	        .channel = TIM_CHANNEL_1,
//	        .port = GPIOB,
//	        .pin = GPIO_PIN_8,
//	        .alternate_func = GPIO_AF3_TIM9

#endif /* INC_CONFIG_H_ */


