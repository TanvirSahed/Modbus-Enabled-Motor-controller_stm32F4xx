/*
 * DM542T.h
 *
 *  Created on: Dec 11, 2024
 *      Author: RusselPC
 */

#ifndef SRC_DM542T_DM542T_H_
#define SRC_DM542T_DM542T_H_
#include <stdint.h>
#include "stm32f4xx_hal.h"


#define ANGLE_MAX	360

/* Stepper Motor Default Configuration*/
#define DM542T_DEFAULT_MOTOR_RPM_MIN		0
#define DM542T_DEFAULT_MOTOR_RPM_MAX		300
#define DM542T_DEFAULT_MOTOR_STEP_ANGLE		0.9f


/*Stepper driver DM542T Default Configuration*/
#define DM542T_DEFAULT_DRIVER_PPS_MIN		0
#define DM542T_DEFAULT_DRIVER_PPS_MAX		2000
#define DM542T_DEFAULT_DRIVER_PPR			6400 // pulse/rev

#define DM542T_DELAY_US				5 //5 us delay is needed according to the dm542t manual



/**
 * @enum DM542T_SMStates_te
 * @brief State machine states for the stepper driver.
 */
typedef enum DM542T_SMStates{
	DM542T_SM_STATE_NONE_0 = 0,
	DM542T_SM_STATE_INITIAL_1,
	DM542T_SM_STATE_IDLE_2,
	DM542T_SM_STATE_MOVE_CW_3,
	DM542T_SM_STATE_MOVE_CCW_4,
	DM542T_SM_STATE_ERROR_5,
	DM542T_SM_STATE_MAX
}DM542T_SMStates_te;

/**
 * @enum DM542T_Error_te
 * @brief Error codes for the stepper driver.
 */
typedef enum {
	DM542T_ERR_OK 					= 0,
	DM542T_ERR 						= -1,
	DM542T_ERR_NULL_PTR 			= -2,
	DM542T_ERR_INVALID_VALUE 		= -3,
	DM542T_ERR_PWM_START_FAILED 	= -4,
	DM542T_ERR_PWM_STOP_FAILED 		= -5
}DM542T_Error_te;

/**
 * @enum DM542T_PinState_te
 * @brief Pin state enumeration.
 */
typedef enum {
	DM542T_PIN_LOW = 0,
	DM542T_PIN_HIGH
}DM542T_PinState_te;

/**
 * @enum DM542T_Dir_te
 * @brief Direction enumeration.
 */
typedef enum {
	DM542T_DIR_NONE = 0,
	DM542T_DIR_CW,
	DM542T_DIR_CCW,
}DM542T_Dir_te;

/**
 * @struct DM542T_Config_ts
 * @brief Configuration structure for the stepper driver.
 */
typedef struct DM542T_Config{
	uint32_t ppr;  // DM54T driver pulse/rev  settings
	uint32_t microsteps; // microsteps/steps
	uint8_t isChanged;
}DM542T_Config_ts;

/**
 * @struct StepperMotorConfig_ts
 * @brief Configuration structure for the stepper motor.
 */
typedef struct StepperMotorConfig{
	float stepAngle;
	uint32_t stepsPerRev;
	float rpmMax;
	uint32_t ppsMax;
	uint8_t isChanged;
}StepperMotorConfig_ts;

/**
 * @struct TimerConfig_ts
 * @brief Configuration structure for the timer to generate PWM.
 */
typedef struct TimerConfig{
	uint32_t sysFreq;
	uint32_t prescaler;
	uint32_t period;
	uint8_t isChanged;
}TimerConfig_ts;

/**
 * @struct DM542T_ts
 * @brief Runtime structure for the stepper driver.
 */
typedef struct DM542T{


	uint8_t enable;		// enable the driver software
	uint8_t start;		// start the driver hardware
	DM542T_Dir_te dir;

	float posAngle;
	uint16_t pps;
	uint16_t rpm;
	uint16_t refRpm;
	uint8_t isChanged; // if any value is changed which need to update other params


	/*DM542T Driver configuration*/
	DM542T_Config_ts drvConfig;

	/*Stepper motor configuration*/
	StepperMotorConfig_ts motorConfig;

	/*Timer configuration for pwm*/
	TimerConfig_ts timerConfig;


	/*State machine*/
	DM542T_SMStates_te state;
	uint8_t fault;
	uint8_t faultClear;

	DM542T_Error_te (*init)(struct DM542T *drv);
	void (*enPin)(uint8_t status);
	void (*dirPin)(DM542T_Dir_te dir);
	DM542T_Error_te (*timerInit)(struct DM542T *drv);
	DM542T_Error_te (*startPWM)(void);
	DM542T_Error_te (*stopPWM)(void);
	uint32_t (*getTimerPrescaler)(void);
	uint32_t (*getSysFreq)(void);
	DM542T_Error_te (*setTimerPeriod)(struct DM542T *drv, uint32_t period);
	void (*delay_us)(uint32_t us);
}DM542T_ts;


DM542T_Error_te DM542T_Init(DM542T_ts *drv);
DM542T_Error_te DM542T_Enable(DM542T_ts *drv);
DM542T_Error_te DM542T_Disable(DM542T_ts *drv);
DM542T_Error_te DM542T_StartPulse(DM542T_ts *drv);
DM542T_Error_te DM542T_StopPulse(DM542T_ts *drv);
DM542T_Error_te DM542T_SetDir(DM542T_ts *drv, DM542T_Dir_te dir);
DM542T_Error_te DM542T_SetRPM(DM542T_ts *drv, float rpm);

DM542T_Error_te DM542T_SetMotorStepAngle(DM542T_ts *drv, float angle);
DM542T_Error_te DM542T_SetDriverMicrostep(DM542T_ts *drv, uint16_t microsteps);
DM542T_Error_te DM542T_CalcMotorStepsPerRev(uint32_t *stepsPerRev, float angle);
DM542T_Error_te DM542T_CalcMotorPPS(uint32_t *pps, float rpm, uint32_t stepsPerRev);
DM542T_Error_te DM542T_CalcDriverPPR(uint32_t *ppr, uint32_t stepsPerRev, uint16_t microsteps);

DM542T_Error_te DM542T_Update(DM542T_ts *drv);
DM542T_Error_te DM542T_Move(DM542T_ts *drv, DM542T_Dir_te dir);
DM542T_Error_te DM542T_Run(DM542T_ts *drv);

#endif /* SRC_DM542T_DM542T_H_ */
