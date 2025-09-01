/*
 * StepperMotor.h
 *
 *  Created on: Dec 17, 2024
 *      Author: RusselPC
 */

#ifndef SRC_STEPPERMOTOR_STEPPERMOTOR_H_
#define SRC_STEPPERMOTOR_STEPPERMOTOR_H_
#include <stdint.h>
#include <DM542T.h>

#define DM542T_DIR_DOWN   	 DM542T_DIR_CW
#define DM542T_DIR_UP	     DM542T_DIR_CCW

#define SMD_MAX_POS_ANGLE	100 // degree

/*State Machine----------------*/
/**
 * @enum SMD_SMState_te
 * @brief State machine states for the stepper motor.
 */
typedef enum SMD_SMState{
	SMD_SM_STAT_NONE = 0,
	SMD_SM_STAT_INACTIVE,
	SMD_SM_STAT_STARTUP,
	SMD_SM_STAT_GOING_DOWN,
	SMD_SM_STAT_DOWN_POS,
	SMD_SM_STAT_GOING_UP,
	SMD_SM_STAT_UP_POS,
	SMD_SM_STAT_FAULT,
	SMD_SM_STAT_MAX,
}SMD_SMState_te;

/**
 * @enum SMD_SMEvent_te
 * @brief Events triggering state transitions.
 */
typedef enum SMD_SMEvent{
	SMD_SM_EVENT_NONE = 0,
	SMD_SM_EVENT_ENABLE,
	SMD_SM_EVENT_DISABLE,
	SMD_SM_EVENT_POS,
	SMD_SM_EVENT_SPEED,
	SMD_SM_EVENT_DIR,
	SMD_SM_EVENT_DOWN_SEN,
	SMD_SM_EVENT_UP_SEN,
	SMD_SM_EVENT_FAULT,
	SMD_SM_EVENT_FAULT_CLEAR,
	SMD_SM_EVENT_MAX
}SMD_SMEvent_te;


/**
 * @enum SMD_SMTrigger_te
 * @brief Triggers for state transitions.
 */
typedef enum SMD_SMTrigger{
	SMD_SM_TRIG_NONE = 0,
	SMD_SM_TRIG_ENABLE,
	SMD_SM_TRIG_DISABLE,
	SMD_SM_TRIG_DIR_UP,
	SMD_SM_TRIG_DIR_DOWN,
	SMD_SM_TRIG_DOWN_SEN_TRUE,
	SMD_SM_TRIG_DOWN_SEN_FALSE,
	SMD_SM_TRIG_DOWN_POS_MATCHED,
	SMD_SM_TRIG_UP_SEN_TRUE,
	SMD_SM_TRIG_UP_SEN_FALSE,
	SMD_SM_TRIG_UP_POS_MATCHED,
	SMD_SM_TRIG_FAULT_TRUE,
	SMD_SM_TRIG_FAULT_FALSE,
	SMD_SM_TRIG_FAULT_CLR_TRUE,
	SMD_SM_TRIG_FAULT_CLR_FALSE,
	SMD_SM_TRIG_MAX
}SMD_SMTrigger_te;

/*Stepper Motor ---------------------*/
/**
 * @enum SMD_Dir_te
 * @brief Direction of stepper motor movement.
 */
typedef enum SMD_Dir{
	SMD_DIR_DOWN = 0,
	SMD_DIR_UP,
	SMD_DIR_UNKNOWN
}SMD_Dir_te;


/*SM Structures-------------*/
/**
 * @struct SMD_SMTrans_ts
 * @brief Represents a single state machine transition.
 */
typedef struct SMD_SMTrans{
	SMD_SMState_te fromState;
	SMD_SMTrigger_te trigger;
	void (*action)(void *arg);
	SMD_SMState_te toState;
}SMD_SMTrans_ts;

/**
 * @struct SMD_SM_ts
 * @brief State machine management structure.
 */
typedef struct SMD_SM{
	uint8_t state;
	uint8_t maxTrans;
	SMD_SMEvent_te event;
	SMD_SMTrigger_te trigger;
	const SMD_SMTrans_ts *trans;
}SMD_SM_ts;

/*Servo Motor Driver structure----------*/
/**
 * @struct StpMotDrv_ts
 * @brief Represents the stepper motor driver, including state machine and control.
 */
typedef struct StpMotDrv{
	/*Other Info*/
	uint8_t id;

	/*Driver Input*/
	uint8_t enable;
	uint8_t start;
	SMD_Dir_te dir;
	uint16_t setPos;
	uint16_t setPosLast;
	uint16_t pos;
	uint16_t maxPos;
	uint8_t isPosMatched;
	float refRpm;
	float inputRpm;
	uint8_t isStartingUp;
	uint8_t isFault;
	uint8_t isFaultClear;
	uint8_t downSenState;
	uint8_t downSenTrig;
	uint8_t upSenState;
	uint8_t upSenTrig;
	uint8_t (*downSen)(void);
	uint8_t (*upSen)(void);

	/*Driver Output*/
	DM542T_ts drv;

	/*State Machine*/
	SMD_SM_ts sm;
}StpMotDrv_ts;

void SMD_Init(StpMotDrv_ts *smd);
void SMD_Run(StpMotDrv_ts *smd);

void SMD_Enable(StpMotDrv_ts *smd);
void SMD_Disable(StpMotDrv_ts *smd);
void SMD_Start(StpMotDrv_ts *smd);
void SMD_Stop(StpMotDrv_ts *smd);
void SMD_SetDir(StpMotDrv_ts *smd, SMD_Dir_te dir);
void SMD_SetSpeed(StpMotDrv_ts *smd, float rpm);
void SMD_SetPos(StpMotDrv_ts *smd, uint16_t pos);
void SMD_MoveToPos(StpMotDrv_ts *smd);
uint8_t SMD_ReadDownSen(void);
uint8_t SMD_ReadUpSen(void);
#endif /* SRC_STEPPERMOTOR_STEPPERMOTOR_H_ */
