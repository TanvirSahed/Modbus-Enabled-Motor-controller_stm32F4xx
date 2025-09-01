/*
 * EncoderMain.h
 *
 *  Created on: Apr 13, 2025
 *      Author: wsrra
 */

#ifndef SRC_ENCODER_ENCODERMAIN_H_
#define SRC_ENCODER_ENCODERMAIN_H_
#include "Encoder.h"

#define ENC_LEARN_RETRY_MAX		5
#define ENC_LEARN_RETRY_MIN		1

#define ENC_LEARN_SAMPLE_MAX	5
#define ENC_LEARN_SAMPLE_MIN	1

#define ENC_LEARN_SPEED_MAX		10
#define ENC_LEARN_SPEED_MIN		1



typedef enum Encoder_SMMode{
	ENC_SM_MODE_NONE = 0,
	ENC_SM_MODE_OPERATE,
	ENC_SM_MODE_LEARNING,
}Encoder_SMMode_te;

/*Encoder state machine*/
typedef enum Encoder_SMState{
	ENC_SM_STATE_INACTIVE_0 = 0,
	ENC_SM_STATE_IDLE_1,
	ENC_SM_STATE_OPERATE_2,
	ENC_SM_STATE_PREP_TO_LEARN_3,
	ENC_SM_STATE_MEASURE_RESOLUTION_4,
//	ENC_SM_STATE_CALCULATE_RESOLUTION_5,
	ENC_SM_STATE_VALIDATE_RESOLUTION_6,
	ENC_SM_STATE_FAULT_7,
	ENC_SM_STATE_PESUDO_JOIN_DISABLE_1,
	ENC_SM_STATE_MAX,
}Encoder_SMState_te;

/*Encoder state machine event*/
typedef enum Encoder_SMEvent{
	ENC_SM_EVENT_NONE = 0,
	ENC_SM_EVENT_ENABLE,
	ENC_SM_EVENT_DISABLE,
	ENC_SM_EVENT_VALID_RESOLUTION,
	ENC_SM_EVENT_LEARNING_START,
	ENC_SM_EVENT_LEARNING_STOP,
	ENC_SM_EVENT_READY_TO_MEASURE,
	ENC_SM_EVENT_SAMPLE_LESS_THAN_N,
	ENC_SM_EVENT_SAMPLE_EQUAL_TO_N,
	ENC_SM_EVENT_READY_TO_VALIDATE,
	ENC_SM_EVENT_NOT_VALID_AND_RETRY,
	ENC_SM_EVENT_NOT_VALID_AND_RETRY_OVER,
	ENC_SM_EVENT_VALID,
	ENC_SM_EVENT_FAULT,
	ENC_SM_EVENT_FAULT_CLEAR,
	ENC_SM_EVENT_MAX,
}Encoder_SMEvent_te;

typedef struct Encoder_SMTransition{
	Encoder_SMState_te statefrom;
	Encoder_SMEvent_te event;
	void (*action)(void *ctx);
	Encoder_SMState_te stateTo;
}Encoder_SMTransition_ts;

/*Encoder state machine handler*/
typedef struct Encoder_StateMachine{
	Encoder_SMState_te state;
	Encoder_SMState_te lastState;
	Encoder_SMEvent_te event;

	Encoder_SMTransition_ts *trans;
	uint16_t transIndex;
}Encoder_StateMachine_ts;

/*Encoder State machine handler*/
typedef struct Encoder_Handler{
	uint8_t id;
	Encoder_StateMachine_ts sm;
	Encoder_ts enc;

	uint8_t enable;
	uint8_t startLearning;
	uint8_t isFault;
	uint8_t clearFault;
	uint16_t resuTolrnc;
	Encoder_SMMode_te mode;
	uint8_t isReadyToMeasure;
	uint32_t resolution;
	uint32_t sum;
	uint8_t isReadCplt;
	uint8_t isNotFirstSample;
	uint8_t sampleCount;
	uint8_t sampleMax;
	uint8_t retry;
	uint8_t retryMax;
	float speed; //in rpm

	Encoder_Error_te (*startLearningCB)(void *ctx);
	Encoder_Error_te (*stopLearningCB)(void *ctx);
	Encoder_Error_te (*checkConfigCB)(void *ctx);
	Encoder_Error_te (*ReadyToMeasureCB)(void *ctx);
	Encoder_Error_te (*MeasureCpltCB)(void *ctx);
}Encoder_Handler_ts;

void Encoder_Init_If(void);
void Encoder_Run(Encoder_Handler_ts *enc);

#endif /* SRC_ENCODER_ENCODERMAIN_H_ */

