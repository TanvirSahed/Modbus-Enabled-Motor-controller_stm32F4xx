/*
 * EncoderMain.c
 *
 *  Created on: Apr 13, 2025
 *      Author: wsrra
 */

#include "EncoderMain.h"
#include "Encoder.h"
#include "debug.h"

static void setNextState(Encoder_Handler_ts *enc, Encoder_SMState_te state);

static void onEnable(void *ctx);
static void onDisable(void *ctx);
static void onFault(void *ctx);
static void onFaultClear(void *ctx);
static void onValidResolution(void *ctx);
static void onLearningStart(void *ctx);
static void onLearningStop(void *ctx);
static void onReadyToMeasure(void *ctx);
static void onSampleLessThanN(void *ctx);
static void onSampleEqualToN(void *ctx);
static void onReadyToValidate(void *ctx);
static void onValid(void *ctx);
static void onNotValidAndRetry(void *ctx);
static void onNotValidAndRetryOver(void *ctx);

static void setFault(Encoder_Handler_ts *enc);
static void clearFault(Encoder_Handler_ts *enc);
static int8_t checkResolution(Encoder_Handler_ts *enc, uint32_t resolution, uint32_t tolarance);

/*State machine transactions*/
// {ENC_SM_STATE_, ENC_SM_EVENT_, on, ENC_SM_STATE_},
static const Encoder_SMTransition_ts SM_TRANS[] = {
		/*State: Inactive 0*/
		{ENC_SM_STATE_INACTIVE_0, ENC_SM_EVENT_ENABLE, onEnable, ENC_SM_STATE_IDLE_1},	//T0

		/*State: Idle 1*/
		{ENC_SM_STATE_IDLE_1, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T1
		{ENC_SM_STATE_IDLE_1, ENC_SM_EVENT_FAULT, onFault, ENC_SM_STATE_FAULT_7},//T2
		{ENC_SM_STATE_IDLE_1, ENC_SM_EVENT_VALID_RESOLUTION, onValidResolution, ENC_SM_STATE_OPERATE_2},//T3
		{ENC_SM_STATE_IDLE_1, ENC_SM_EVENT_LEARNING_START, onLearningStart, ENC_SM_STATE_PREP_TO_LEARN_3},//T4

		/*State: Operate 2*/
		{ENC_SM_STATE_OPERATE_2, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T5
		{ENC_SM_STATE_OPERATE_2, ENC_SM_EVENT_FAULT, onFault, ENC_SM_STATE_FAULT_7},//T6
		{ENC_SM_STATE_OPERATE_2, ENC_SM_EVENT_LEARNING_START, onLearningStart, ENC_SM_STATE_PREP_TO_LEARN_3},//T7

		/*State: Prep to learn 3*/
		{ENC_SM_STATE_PREP_TO_LEARN_3, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T8
		{ENC_SM_STATE_PREP_TO_LEARN_3, ENC_SM_EVENT_FAULT, onFault, ENC_SM_STATE_FAULT_7},//T9
		{ENC_SM_STATE_PREP_TO_LEARN_3, ENC_SM_EVENT_LEARNING_STOP, onLearningStop, ENC_SM_STATE_IDLE_1},//T10
		{ENC_SM_STATE_PREP_TO_LEARN_3, ENC_SM_EVENT_READY_TO_MEASURE, onReadyToMeasure, ENC_SM_STATE_MEASURE_RESOLUTION_4},//T11

		/*State: Measure Resolution 4*/
		{ENC_SM_STATE_MEASURE_RESOLUTION_4, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T12
		{ENC_SM_STATE_MEASURE_RESOLUTION_4, ENC_SM_EVENT_FAULT, onFault, ENC_SM_STATE_FAULT_7},//T13
		{ENC_SM_STATE_MEASURE_RESOLUTION_4, ENC_SM_EVENT_LEARNING_STOP, onLearningStop, ENC_SM_STATE_IDLE_1},//T14
		{ENC_SM_STATE_MEASURE_RESOLUTION_4, ENC_SM_EVENT_SAMPLE_LESS_THAN_N, onSampleLessThanN, ENC_SM_STATE_MEASURE_RESOLUTION_4},//T15
		{ENC_SM_STATE_MEASURE_RESOLUTION_4, ENC_SM_EVENT_SAMPLE_EQUAL_TO_N, onSampleEqualToN, ENC_SM_STATE_VALIDATE_RESOLUTION_6},//T16

		/*State: Calculate Resolution 5*/
//		{ENC_SM_STATE_CALCULATE_RESOLUTION_5, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T17
//		{ENC_SM_STATE_CALCULATE_RESOLUTION_5, ENC_SM_EVENT_FAULT, onFault, ENC_SM_STATE_FAULT_7},//T18
//		{ENC_SM_STATE_CALCULATE_RESOLUTION_5, ENC_SM_EVENT_LEARNING_STOP, onLearningStop, ENC_SM_STATE_IDLE_1},//T19
//		{ENC_SM_STATE_CALCULATE_RESOLUTION_5, ENC_SM_EVENT_READY_TO_VALIDATE, onReadyToValidate, ENC_SM_STATE_VALIDATE_RESOLUTION_6},//T20

		/*State: Validate Resolution 6*/
		{ENC_SM_STATE_VALIDATE_RESOLUTION_6, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T21
		{ENC_SM_STATE_VALIDATE_RESOLUTION_6, ENC_SM_EVENT_FAULT, onFault, ENC_SM_STATE_FAULT_7},//T22
		{ENC_SM_STATE_VALIDATE_RESOLUTION_6, ENC_SM_EVENT_LEARNING_STOP, onLearningStop, ENC_SM_STATE_IDLE_1},//T23
		{ENC_SM_STATE_VALIDATE_RESOLUTION_6, ENC_SM_EVENT_VALID, onValid, ENC_SM_STATE_OPERATE_2},//T24
		{ENC_SM_STATE_VALIDATE_RESOLUTION_6, ENC_SM_EVENT_NOT_VALID_AND_RETRY, onNotValidAndRetry, ENC_SM_STATE_MEASURE_RESOLUTION_4},//T25
		{ENC_SM_STATE_VALIDATE_RESOLUTION_6, ENC_SM_EVENT_NOT_VALID_AND_RETRY_OVER, onNotValidAndRetryOver, ENC_SM_STATE_IDLE_1},//T26

		/*State: Fault 7*/
		{ENC_SM_STATE_FAULT_7, ENC_SM_EVENT_DISABLE, onDisable, ENC_SM_STATE_INACTIVE_0},//T27
		{ENC_SM_STATE_FAULT_7, ENC_SM_EVENT_DISABLE, onFaultClear, ENC_SM_STATE_IDLE_1}//T28
};

static void setNextState(Encoder_Handler_ts *enc, Encoder_SMState_te state){
	enc->sm.state = state;
}

static void onEnable(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

//	if(Encoder_StartTimer(&hEnc->enc) != ENCODER_ERR_NONE){
//		setFault(hEnc);
//		hEnc->enable = 0;
//		return;
//
//	}
	dbg_print("ENC_%d: Enabled\r\n",hEnc->id);
}

static void onDisable(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

//	if(Encoder_StopTimer(&hEnc->enc) != ENCODER_ERR_NONE){
//		setFault(hEnc);
//		return;
//	}
	hEnc->startLearning = 0;
	dbg_print("ENC_%d: Disabled\r\n",hEnc->id);
}

static void onFault(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
	hEnc->startLearning = 0;
	dbg_print("ENC_%d: Fault\r\n",hEnc->id);
}
static void onFaultClear(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
	hEnc->startLearning = 0;
	dbg_print("ENC_%d: Fault cleared\r\n",hEnc->id);
}

static void onValidResolution(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
//	if(Encoder_StartTimer(&hEnc->enc) != ENCODER_ERR_NONE){
//		setFault(hEnc);
//		return;
//	}
	hEnc->mode = ENC_SM_MODE_OPERATE;
	//TODO: write code here
	dbg_print("ENC_%d: Valid Resolution\r\n",hEnc->id);
}

static void onLearningStart(void *ctx){
	/* Start encoder
	 *
	 * */
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;

//	if(Encoder_StartTimer(&hEnc->enc) != ENCODER_ERR_NONE){
//		setFault(hEnc);
//		return;
//	}
	if(hEnc->startLearningCB != NULL){
		if(hEnc->startLearningCB(&hEnc->enc) != ENCODER_ERR_NONE){
//			Encoder_StopTimer(&hEnc->enc);
			setFault(hEnc);
			return;
		}
	}
	hEnc->mode = ENC_SM_MODE_LEARNING;
	dbg_print("ENC_%d: Learning Started\r\n",hEnc->id);
}

static void onLearningStop(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
//	if(Encoder_StopTimer(&hEnc->enc) != ENCODER_ERR_NONE){
//		setFault(hEnc);
//		return;
//	}
	if(hEnc->stopLearningCB != NULL){
		if(hEnc->stopLearningCB(&hEnc->enc) != ENCODER_ERR_NONE){
			setFault(hEnc);
			return;
		}
	}

	hEnc->mode = ENC_SM_MODE_NONE;
	dbg_print("ENC_%d: Learning Stopped\r\n",hEnc->id);
}
static void onReadyToMeasure(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
	hEnc->ReadyToMeasureCB(ctx);

}
static void onSampleLessThanN(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here

	hEnc->sum += hEnc->enc.countPerRev;
	hEnc->sampleCount++;
	dbg_print("ENC_%d: smpl:%d, cnt:%ld, sum:%ld\r\n",
			hEnc->id, hEnc->sampleCount, hEnc->enc.countPerRev, hEnc->sum);
}
static void onSampleEqualToN(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	if(hEnc->sampleMax==0){
		setFault(hEnc);
		return;
	}
	//TODO: write code here

	hEnc->MeasureCpltCB((void*)hEnc);

	hEnc->resolution = hEnc->sum/(hEnc->sampleCount*ENC_COUNT_TYPE);
	hEnc->sampleCount = 0;
	hEnc->sum = 0;

	dbg_print("ENC_%d: MeasuredReso:%ld\r\n",hEnc->id,hEnc->resolution);
}
static void onReadyToValidate(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
}
static void onValid(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
	hEnc->startLearning = 0;
	hEnc->enc.resolution = hEnc->resolution;
	hEnc->enc.ppr = hEnc->resolution*ENC_COUNT_TYPE;
	dbg_print("ENC_%d: Valid Reso:%ld, ppr:%ld\r\n",
			hEnc->id,hEnc->enc.resolution, hEnc->enc.ppr);
}
static void onNotValidAndRetry(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
}
static void onNotValidAndRetryOver(void *ctx){
	Encoder_Handler_ts *hEnc = (Encoder_Handler_ts*)ctx;
	//TODO: write code here
}


static void setFault(Encoder_Handler_ts *enc){
	enc->isFault = 1;
}

static void clearFault(Encoder_Handler_ts *enc){
	enc->isFault = 0;
}

static int8_t checkResolution(Encoder_Handler_ts *enc, uint32_t resolution, uint32_t tolarance){
	if(resolution == 0 && resolution <= tolarance){
		return ENCODER_ERR_INVALID_VALUE;
	}
	return ((resolution >= (resolution - tolarance)) &&
			(resolution <= (resolution + tolarance)));
}

Encoder_Error_te Encoder_Enable(Encoder_Handler_ts *enc){
	if(enc == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
	enc->enable = 1;
	return ENCODER_ERR_NONE;
}

Encoder_Error_te Encoder_Disable(Encoder_Handler_ts *enc){
	if(enc == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
	enc->enable = 0;
	return ENCODER_ERR_NONE;
}

Encoder_Error_te Encoder_StartLearnig(Encoder_Handler_ts *enc){
	if(enc == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
	enc->startLearning = 1;
	return ENCODER_ERR_NONE;
}

Encoder_Error_te Encoder_StopLearnig(Encoder_Handler_ts *enc){
	if(enc == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
	enc->startLearning = 0;
	return ENCODER_ERR_NONE;
}


void Encoder_ProcessEvent(Encoder_Handler_ts *enc){
	if(enc->sm.state == ENC_SM_STATE_INACTIVE_0){
		if(enc->enable){
			enc->sm.event = ENC_SM_EVENT_ENABLE;
		}
		return;
	}

	if(!enc->enable){
		enc->sm.event = ENC_SM_EVENT_DISABLE;
		return;
	}

	if(enc->isFault && enc->sm.state != ENC_SM_STATE_FAULT_7){
		enc->sm.event = ENC_SM_EVENT_FAULT;
		return;
	}

	switch (enc->sm.state) {
//		case ENC_SM_STATE_INACTIVE_0:
//			if(enc->enable){
//				enc->sm.event = ENC_SM_EVENT_ENABLE;
//			}
//
//			break;
		case ENC_SM_STATE_IDLE_1:
			{
				if(enc->startLearning){
					enc->sm.event = ENC_SM_EVENT_LEARNING_START;
				}
				int8_t ret = checkResolution(enc, enc->enc.resolution, enc->resuTolrnc);
				if(ret == 1){
					enc->sm.event = ENC_SM_EVENT_VALID_RESOLUTION;
				}else if(ret == ENCODER_ERR_INVALID_VALUE){
					dbg_print("ENC_%d: Res:%ld, Tol:%ld\r\n",enc->id, enc->enc.resolution,enc->resuTolrnc);
					setFault(enc);
					enc->sm.event = ENC_SM_EVENT_FAULT;
				}
			}break;
		case ENC_SM_STATE_OPERATE_2:
			if(enc->startLearning){
				enc->sm.event = ENC_SM_EVENT_LEARNING_START;
			}
			break;
		case ENC_SM_STATE_PREP_TO_LEARN_3:

			if(!enc->startLearning){
				enc->sm.event = ENC_SM_EVENT_LEARNING_STOP;
			}else {
				if(enc->checkConfigCB == NULL){
					setFault(enc);
					enc->sm.event = ENC_SM_EVENT_FAULT;
				}else{
					if(enc->checkConfigCB((void*)enc) != ENCODER_ERR_NONE){
						setFault(enc);
						enc->sm.event = ENC_SM_EVENT_FAULT;
					}else{
						enc->sm.event = ENC_SM_EVENT_READY_TO_MEASURE;
					}
				}

			}

			break;
		case ENC_SM_STATE_MEASURE_RESOLUTION_4:
			if(!enc->startLearning){
				enc->sm.event = ENC_SM_EVENT_LEARNING_STOP;
			}else if(enc->isReadCplt){
				enc->isReadCplt=0;
				if(enc->isNotFirstSample){
					if(enc->sampleCount < enc->sampleMax){
						enc->sm.event = ENC_SM_EVENT_SAMPLE_LESS_THAN_N;
					}
				}
				enc->isNotFirstSample = 1;
			}else if(enc->sampleCount >= enc->sampleMax){
				enc->sm.event = ENC_SM_EVENT_SAMPLE_EQUAL_TO_N;
			}
			break;
//		case ENC_SM_STATE_CALCULATE_RESOLUTION_5:
//			if(!enc->startLearning){
//				enc->sm.event = ENC_SM_EVENT_LEARNING_STOP;
//			}
//			break;
		case ENC_SM_STATE_VALIDATE_RESOLUTION_6:
			{
				if(!enc->startLearning){
					enc->sm.event = ENC_SM_EVENT_LEARNING_STOP;
				}
				int8_t ret = checkResolution(enc, enc->enc.resolution, enc->resuTolrnc);
				if(ret == 0){
					if(enc->retry < enc->retryMax){
						enc->sm.event = ENC_SM_EVENT_NOT_VALID_AND_RETRY;
					}else if(enc->retry >= enc->retryMax){
						enc->sm.event = ENC_SM_EVENT_NOT_VALID_AND_RETRY_OVER;
					}
				}else if(ret == 1){
					enc->sm.event = ENC_SM_EVENT_VALID;
				}else if(ret == ENCODER_ERR_INVALID_VALUE){
					setFault(enc);
					enc->sm.event = ENC_SM_EVENT_FAULT;
				}
			}
			break;
		case ENC_SM_STATE_FAULT_7:
			if(enc->clearFault){
				enc->sm.event = ENC_SM_EVENT_FAULT_CLEAR;
			}
			break;
		default:
			enc->sm.state = ENC_SM_STATE_INACTIVE_0;
			break;
	}
}

/*Encoder State Machine*/
void Encoder_Run(Encoder_Handler_ts *enc){
	if(enc == NULL){
		return;
	}
	Encoder_ProcessEvent(enc);
	const uint16_t noOfTrans =  sizeof(SM_TRANS)/sizeof(SM_TRANS[0]);
	for(uint16_t t = 0;  t < noOfTrans; t++){
		if(enc->sm.state == SM_TRANS[t].statefrom){
			if(enc->sm.event == SM_TRANS[t].event){
				dbg_print("ENC_%d: {T:%d, St:%d->%d, Ev:%d}\r\n",
						enc->id,t,enc->sm.state, SM_TRANS[t].stateTo, enc->sm.event);
				SM_TRANS[t].action((void*)enc);
				enc->sm.state = SM_TRANS[t].stateTo;
				enc->sm.event = ENC_SM_EVENT_NONE;
			}
		}
	}


	Encoder_Update(&enc->enc);


}
