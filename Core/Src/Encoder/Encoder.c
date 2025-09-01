/*
 * Encoder.c
 *
 *  Created on: Dec 3, 2024
 *      Author: RusselPC
 */

#include "Encoder.h"
#include <stdint.h>
#include <stdbool.h>
#include "debug.h"
#include <string.h>

int32_t Encoder_GetCount(Encoder_ts *enc);

/**
 * @brief Initializes the encoder with the specified timer and resolution.
 * @param enc Pointer to the encoder structure.
 * @param htim Pointer to the timer handle used for the encoder.
 * @param resolution Encoder resolution (ticks per revolution).
 * @return ENCODER_ERR_NONE on success, or an error code on failure.
 */
Encoder_Error_te Encoder_Init(Encoder_ts *enc, TIM_HandleTypeDef *htim, uint16_t resolution) {
	if(enc == NULL || htim == NULL){
		return ENCODER_ERR_NULL_PTR;
	}
    enc->htim = htim;
    enc->count = 0;
    enc->countPerRev = 0;
    enc->lastCount = 0;
    enc->countZ = 0;
    enc->dir = ENCODER_DIR_NONE;
    enc->rpm = 0;
    enc->resolution = resolution;
    enc->ppr = resolution*ENC_COUNT_TYPE;
    enc->posAngle = 0;
    enc->last_timestamp = HAL_GetTick();

    enc->chAInCap.is1stEadgCapturd = 0;
    enc->chAInCap.isCaptured = 0;
	enc->chAInCap.count1 = 0;
	enc->chAInCap.count2 = 0;
	enc->chAInCap.diff = 0;
	enc->chAInCap.index = 0;
	enc->chAInCap.freq = 0;
	FLTR_InitMedian(&enc->chAInCap.medFltr, FLTR_MED_WINDOW_SIZE);
	FLTR_InitMovAvg(&enc->chAInCap.movAvgFltr, FLTR_MOV_WINDOW_SIZE);
	memset(enc->chAInCap.diffBuf, 0, 20);

//	enc->test.i = 0;
    /*Start Encoder*/
	Encoder_SetCount(enc, 0);
	Encoder_SetPPR(enc, enc->ppr);
    Encoder_StartTimer(enc);
//    HAL_TIM_IC_Start_IT(enc->htim, TIM_CHANNEL_1);
    return ENCODER_ERR_NONE;
}

/**
 * @brief Sets the resolution and pulses per revolution (PPR) for the encoder.
 *
 * This function configures the encoder by setting its resolution and
 * calculating the corresponding pulses per revolution (PPR) using a
 * predefined count type multiplier.
 *
 * @param enc Pointer to the encoder structure to be configured.
 * @param resolution The resolution value to set (e.g., counts per revolution).
 */
void Encoder_SetResolution(Encoder_ts *enc, uint32_t resolution){
    enc->resolution = resolution;
    enc->ppr = resolution*ENC_COUNT_TYPE;
}

/**
 * @brief Starts the timer for the encoder.
 * @param enc Pointer to the encoder structure.
 * @return ENCODER_ERR_NONE on success, or an error code on failure.
 */
Encoder_Error_te Encoder_StartTimer(Encoder_ts *enc){
	if(HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL) != HAL_OK){
		return ENCODER_ERR_TIM_START_FAILED;
	}
	enc->isStarted = 1;
	return ENCODER_ERR_NONE;
}


/**
 * @brief Stops the timer for the encoder.
 * @param enc Pointer to the encoder structure.
 * @return ENCODER_ERR_NONE on success, or an error code on failure.
 */
Encoder_Error_te Encoder_StopTimer(Encoder_ts *enc){
	if(HAL_TIM_Encoder_Stop(enc->htim, TIM_CHANNEL_ALL) != HAL_OK){
		return ENCODER_ERR_TIM_START_FAILED;
	}
	enc->isStarted = 0;
	return ENCODER_ERR_NONE;
}


/**
 * @brief Sets the pulses per revolution (PPR) for the encoder.
 * @param enc Pointer to the encoder structure.
 * @param ppr Pulses per revolution.
 */
void Encoder_SetPPR(Encoder_ts *enc, uint32_t ppr){
	enc->ppr = ppr;
	__HAL_TIM_SET_AUTORELOAD(enc->htim, (ppr-1));
}

//// Reset count on Z-index and enforce limits
//void Encoder_Reset(void) {
//    if (encoder.count < encoder.min_limit) {
//        encoder.count = encoder.min_limit;
//        if (encoder.on_limit_hit) encoder.on_limit_hit(true); // Min limit hit
//    } else if (encoder.count > encoder.max_limit) {
//        encoder.count = encoder.max_limit;
//        if (encoder.on_limit_hit) encoder.on_limit_hit(false); // Max limit hit
//    } else {
//        encoder.count = 0; // Normal reset
//    }
//    encoder.dir = ENCODER_DIR_UNKNOWN;
//}
//


/**
 * @brief Updates the encoder state, including direction, position, and RPM.
 * @param enc Pointer to the encoder structure.
 */
void Encoder_Update(Encoder_ts *enc) {

	enc->dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(enc->htim);//Encoder_GetDir(enc);
	Encoder_GetCount(enc);

	enc->posAngle = Encoder_CalcPositionAngle(enc->count, enc->ppr);
	enc->lastCount = enc->count;

//	static uint16_t pos = 0;
//	if((uint16_t)enc->count != pos){
//		dbg_print("count:%ld, dir:%d, pos: %d\r\n",enc->count, enc->dir, (uint16_t)enc->posAngle );
//	}
//	pos = enc->count;

	Encoder_CalcRPM(enc);
}



/**
 * @brief Retrieves the current count from the encoder.
 * @param enc Pointer to the encoder structure.
 * @return The current encoder count as a 32-bit signed integer.
 */
int32_t Encoder_GetCount(Encoder_ts *enc){

//	if(enc->dir == 1){
//		/*Down count*/
//		enc->count = ENC_MAX_COUNT_RESOLUTION - __HAL_TIM_GET_COUNTER(enc->htim);
//	}else{
		/*Up count*/
		enc->count = __HAL_TIM_GET_COUNTER(enc->htim);
//	}

	return enc->count;
}


/**
 * @brief Sets the encoder count to a specified value.
 * @param enc Pointer to the encoder structure.
 * @param count The desired count value to set.
 * @return ENCODER_ERR_NONE on success, or an error code on failure.
 */
Encoder_Error_te Encoder_SetCount(Encoder_ts *enc, uint32_t count){
	/*Stop Timer*/
	if(Encoder_StopTimer(enc) != ENCODER_ERR_NONE){
		return ENCODER_ERR_TIM_STOP_FAILED;
	}

	/*Modify counter value*/
	__HAL_TIM_SET_COUNTER(enc->htim, count);
	enc->count = 0;

	/*Start Timer*/
	if(Encoder_StartTimer(enc) != ENCODER_ERR_NONE){
		return ENCODER_ERR_TIM_START_FAILED;
	}
	return ENCODER_ERR_NONE;
}


/**
 * @brief Determines the direction of the encoder's movement.
 * @param enc Pointer to the encoder structure.
 * @return The current direction of the encoder as an `EncoderDirection_te` value.
 */
EncoderDirection_te Encoder_GetDir(Encoder_ts *enc){



    if(enc->count > enc->lastCount){
    	/*Forward Direction*/
    	enc->dir = ENCODER_DIR_FORWARD;
    }else if(enc->count < enc->lastCount){
    	/*reverse Direction*/
    	enc->dir = ENCODER_DIR_REVERSE;
    }else{
    	/*No Direction*/
//    	enc->dir = ENCODER_DIR_NONE;
    }
    return enc->dir;
}


/**
 * @brief Calculates the position angle based on the encoder count and pulses per revolution (PPR).
 * @param count The current encoder count.
 * @param ppr Pulses per revolution of the encoder.
 * @return The calculated position angle in degrees. Returns `ENCODER_ERR_INVALID_VALUE` for invalid inputs.
 */
float Encoder_CalcPositionAngle(uint32_t count, uint32_t ppr){
	if(ppr <= 0 || count > ppr){
		return ENCODER_ERR_INVALID_VALUE;
	}

	return (float)(ENC_DEGREE_MAX*((float)count/(float)(ppr)));
}


/**
 * @brief Handles the Z-index capture event for the encoder.
 *
 * This function resets the encoder count and calculates the RPM based on the time interval between Z-index events.
 * @param enc Pointer to the encoder structure.
 */
void Encoder_IndexZCapture(Encoder_ts *enc){

//	dbg_print("enc: count: %d, ang: %d\r\n",enc->count, enc->posAngle);

	enc->countPerRev = __HAL_TIM_GET_COUNTER(enc->htim);
	dbg_print("Enc: Count:%ld\r\n",enc->countPerRev);
	enc->isZTriged = 1;
	enc->countZ++;
	enc->posAngle = 0;
	enc->count = 0;
	Encoder_SetCount(enc,0);

	/*TODO: Test*/
	uint32_t time = HAL_GetTick();
	static uint32_t lastTime = 0;
	uint16_t deltaTime = time - lastTime;
	lastTime = time;
	enc->rpmFromZ = (float)(60000.0f/(float)deltaTime);

}

/**
 * @brief Calculates the RPM (Revolutions Per Minute) of the encoder.
 *
 * This function applies filtering to the captured data, validates the input, calculates
 * the frequency, and finally computes the RPM based on the encoder's resolution.
 *
 * @param enc Pointer to the encoder structure.
 */
void Encoder_CalcRPM(Encoder_ts *enc){

	if (enc->resolution == 0 ){ //|| enc->chAInCap.index < 5
		return;
	}

//	uint32_t avrgDiff = 0;
//	for(uint8_t i = 0; i < 5; i++){
//		avrgDiff += enc->chAInCap.diffBuf[i];
//	}
//	avrgDiff = avrgDiff/5;
//	enc->chAInCap.index = 0;

	if(enc->chAInCap.diff == enc->chAInCap.lastDiff && enc->chAInCap.isCaptured == 0){return;}
	enc->chAInCap.lastDiff = enc->chAInCap.diff;
	enc->chAInCap.isCaptured = 0;



	/*1. Apply Median filter*/
	enc->chAInCap.medFltr.nextValue = enc->chAInCap.diff;//avrgDiff;//
	FLTR_ApplyMedian(&enc->chAInCap.medFltr);

	/*1. Apply moving average filter*/
	enc->chAInCap.movAvgFltr.nextValue = enc->chAInCap.medFltr.value;
	FLTR_ApplyMovAvg(&enc->chAInCap.movAvgFltr);

	/* CHeck the data validity*/
	float fltrdValue  = enc->chAInCap.movAvgFltr.value;
	if(fltrdValue == 0){
		return;
	}

	/* calculate frequency*/
	enc->chAInCap.freq =  ((enc->chAInCap.refClk/(float)enc->resolution)/fltrdValue);

	/*Calculate RPM*/
	enc->rpm = enc->chAInCap.freq * 60.0;

////	dbg_print("Enc Calc: avg: %ld, frq:%f\r\n",avrgDiff, enc->chAInCap.freq);
//	if(enc->test.i < 50) {
//		enc->test.dif[enc->test.i] = enc->chAInCap.diff;
//		enc->test.med[enc->test.i] = enc->chAInCap.medFltr.value;
//		enc->test.mov[enc->test.i] = enc->chAInCap.movAvgFltr.value;
//		enc->test.rpm[enc->test.i] = enc->rpm;
//		enc->test.i++;
//	}

}

/**
 * @brief Captures the timing information for channel A of the encoder.
 *
 * This function records the edges detected on channel A and calculates the time
 * difference between two consecutive edges. This time difference is used to
 * compute the encoder's speed and direction.
 *
 * @param enc Pointer to the encoder structure.
 * @param count The current timer count when the edge is captured.
 */
void Encoder_ChACapture(Encoder_ts *enc, uint32_t count){
	if(enc->chAInCap.is1stEadgCapturd == 0){
		/*First edge*/
		enc->chAInCap.count1 = count;
		enc->chAInCap.is1stEadgCapturd = 1;

	}else{
		/*2nd edge*/
		enc->chAInCap.count2 = count;



		if (enc->chAInCap.count2 > enc->chAInCap.count1){
			enc->chAInCap.diff = enc->chAInCap.count2 - enc->chAInCap.count1;
		}else if (enc->chAInCap.count1 > enc->chAInCap.count2){
			enc->chAInCap.diff = (0xFFFFFFFF - enc->chAInCap.count1) + enc->chAInCap.count2;

		}
		enc->chAInCap.count1 = enc->chAInCap.count2;
//		enc->chAInCap.is1stEadgCapturd = 0;
		enc->chAInCap.isCaptured = 1;

//		if(enc->chAInCap.index < 5){
//
//			enc->chAInCap.diffBuf[enc->chAInCap.index++] = enc->chAInCap.diff;
//		}

//		dbg_print("Enc Cap\r\n");
//		uint8_t i = 0;
//		if(i >= 50){ i = 0;}
//		enc->chAInCap.d[i] =  	enc->chAInCap.diff;

//		dbg_print("%ld, %ld, %ld\r\n",enc->chAInCap.count1 , enc->chAInCap.count2,diff);
	}
}


//// Limit hit handler (optional user-defined callback)
//void Encoder_SetLimitHitCallback(void (*callback)(bool is_min_limit)) {
//    encoder.on_limit_hit = callback;
//}
//
//// ISR for Z-index or proximity sensor
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    if (GPIO_Pin == GPIO_PIN_0) { // Z-index pin
//        Encoder_Reset();
//    } else if (GPIO_Pin == GPIO_PIN_10) { // Proximity sensor pin
//        Encoder_Reset();
//    }
//}



/*Encoder State machine---------------*/
