/*
 * MotorController.h
 *
 *  Created on: Mar 4, 2025
 *      Author: wsrra
 */

#ifndef SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_
#define SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_
#include <stdint.h>
#include "Timer.h"


#define MC_LEARNING_SPEED			5		//rpm
#define MC_LEARNING_SPEED_MIN		0.1F		//rpm
#define MC_LEARNING_SPEED_MAX		10.0F		//rpm

/**
 * @enum MotorCtrl_Error_te
 * @brief Error codes for the stepper driver.
 */
typedef enum {
	MOTOR_CTRL_ERR_OK 					= 0,
	MOTOR_CTRL_ERR 						= -1,
	MOTOR_CTRL_ERR_NULL_PTR 			= -2,
	MOTOR_CTRL_ERR_INVALID_VALUE 		= -3,
	MOTOR_CTRL_ERR_INVALID_DIR	 		= -4,

}MotorCtrl_Error_te;


/**
 * @enum MotorCtrl_Error_te
 * @brief Direction of stepper motor movement.
 */
typedef enum MotorCtrl_Dir{
	MOTOR_CTRL_DIR_NONE = 0,
	MOTOR_CTRL_DIR_DOWN,
	MOTOR_CTRL_DIR_UP,

}MotorCtrl_Dir_te;

typedef enum MotorCtrl_Path{
	MOTOR_CTRL_PATH_NONE = 0,
	MOTOR_CTRL_PATH_NORMAL,
	MOTOR_CTRL_PATH_REVERSE,


}MotorCtrl_Path_te;


typedef enum MotorCtrl_Type{
	MOTOR_CTRL_TYPE_STEPPER,
	MOTOR_CTRL_TYPE_HBRIDGE
}MotorCtrl_Type_te;

typedef enum MotorCtrl_Fault{
	MOTOR_CTRL_FAULT_OK = 0,
	MOTOR_CTRL_FAULT_ON_ENABLE,
	MOTOR_CTRL_FAULT_ON_DISABLE,
	MOTOR_CTRL_FAULT_ON_START,
	MOTOR_CTRL_FAULT_ON_STOP,
	MOTOR_CTRL_FAULT_ON_READ_DOWN_SEN,
	MOTOR_CTRL_FAULT_ON_READ_UP_SEN,
	MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR,
	MOTOR_CTRL_FAULT_ON_SET_UP_DIR,
	MOTOR_CTRL_FAULT_ON_SET_SPEED,
}MotorCtrl_Fault_te;


//typedef enum MotorCtrl_ChangeFlag{
//	MOTOR_CTRL_CHANGE_NONE = 0,
//	MOTOR_CTRL_CHANGE_
//}MotorCtrl_ChangeFlag_te;

/*Motor Controller State Machine---------------*/
/*States for the State Machine*/
typedef enum MotorCtrl_SMState{
	MOTOR_CTRL_SM_STATE_INACTIVE_0 = 0,
	MOTOR_CTRL_SM_STATE_START_UP_1,
	MOTOR_CTRL_SM_STATE_GOING_DOWN_2,
	MOTOR_CTRL_SM_STATE_DOWN_POS_3,
	MOTOR_CTRL_SM_STATE_GOING_UP_4,
	MOTOR_CTRL_SM_STATE_UP_POS_5,
	MOTOR_CTRL_SM_STATE_WAITING_6,
	MOTOR_CTRL_SM_STATE_FAULT_7,
	MOTOR_CTRL_SM_STATE_MID_POS_8,
	MOTOR_CTRL_SM_STATE_MAX
}MotorCtrl_State_te;

/*Events for the State Machine*/
typedef enum MotorCtrl_SMEvent{
	MOTOR_CTRL_SM_EVENT_NONE = 0,
	MOTOR_CTRL_SM_EVENT_ENABLE,
	MOTOR_CTRL_SM_EVENT_DISABLE,
	MOTOR_CTRL_SM_EVENT_START,
	MOTOR_CTRL_SM_EVENT_STOP,
	MOTOR_CTRL_SM_EVENT_ABSOLUTE_POS,
	MOTOR_CTRL_SM_EVENT_MOTOR_SPEED,
	MOTOR_CTRL_SM_EVENT_DOWN_SEN,

}MotorCtrl_SMEvent_te;

typedef struct MotorCtrl_Sensor{
	uint8_t state;
	uint8_t isTriggered;
}MotorCtrl_Sensor_ts;

/*State Machine Handler*/
typedef struct MotorCtrl_StateMachine{
	MotorCtrl_State_te state;
	MotorCtrl_SMEvent_te event;
}MotorCtrl_StateMachine_ts;




/*Motor Interface*/
typedef struct MotorCtrl_DriverInterface{
	MotorCtrl_Error_te (*enable)(void);
	MotorCtrl_Error_te (*disable)(void);
	MotorCtrl_Error_te (*start)(void);
	MotorCtrl_Error_te (*stop)(void);
	MotorCtrl_Error_te (*setDir)(MotorCtrl_Dir_te dir);//--------------------------------
	MotorCtrl_Error_te (*setSpeed)(float speed);
}MotorCtrl_DriverInterface_ts;


typedef struct MotorCtrl_LimitSenInterface{
	MotorCtrl_Error_te (*readUpSen)(MotorCtrl_Sensor_ts *sen);
	MotorCtrl_Error_te (*readDownSen)(MotorCtrl_Sensor_ts *sen);
}MotorCtrl_LimitSenInterface_ts;

typedef struct MotorCtrl_Callback{
	void (*downPosRechdCB)(void *ctx);
	void (*upPosRechdCB)(void *ctx);
	void (*midPosRechdCB)(void *ctx);
}MotorCtrl_Callback_ts;

/*Main motor controller*/
typedef struct MotorController{
	/*State Machine structure*/
	MotorCtrl_StateMachine_ts sm;

	uint8_t isStartUp;
	uint8_t isLearnOpenPos;
	uint8_t enable;
	uint8_t start; // motor start stop command
	MotorCtrl_Dir_te dir;
	MotorCtrl_Path_te path;
	float speed;
	float lastSpeed;
	float maxSpeed;
	float learningSpeed;
	float pos;
	float setPos;
	float setPosLast;
	float maxPos; // limit learned from learnng stage


	MotorCtrl_Type_te type;
	MotorCtrl_Sensor_ts upSen;
	MotorCtrl_Sensor_ts downSen;
	MotorCtrl_Fault_te fault;
	uint8_t faultClear;

	Timer_ts timer;
	MotorCtrl_DriverInterface_ts *motorIf;
	MotorCtrl_LimitSenInterface_ts *limitSenIf;

	MotorCtrl_Callback_ts callback;
}MotorController_ts;





MotorCtrl_Error_te MotorCtrl_Init(MotorController_ts *mc);
MotorCtrl_Error_te MotorCtrl_Run(MotorController_ts *mc);

MotorCtrl_Error_te MotorCtrl_MoveToPos(MotorController_ts *mc, uint16_t pos,
		float speed, uint8_t start);

MotorCtrl_Error_te MotorCtrl_SetMotorDriverInterface(MotorController_ts *mc,
		MotorCtrl_DriverInterface_ts *drv);
MotorCtrl_Error_te MotorCtrl_SetLimitSensorInterface(MotorController_ts *mc,
		MotorCtrl_LimitSenInterface_ts *sen);

MotorCtrl_Error_te MotorCtrl_AttachCallback(MotorController_ts *mc,
		MotorCtrl_Callback_ts *callback);

#endif /* SRC_MOTORCONTROLLER_MOTORCONTROLLER_H_ */
