/*
 * Encoder.h
 *
 *  Created on: Dec 3, 2024
 *      Author: RusselPC
 */

#ifndef SRC_ENCODER_ENCODER_H_
#define SRC_ENCODER_ENCODER_H_

#include "stm32f4xx_hal.h"
#include "Filters.h"

/*Encoder configurations*/
#define ENC_RESOLUTION						1024U //
#define ENC_MAX_COUNT_RESOLUTION			0xFFFF
#define ENC_DEGREE_MAX						360U	//

#define ENC_COUNT_4X						4
#define ENC_COUNT_TYPE						ENC_COUNT_4X

/**
 * @brief Macro for encoder debounce filtering.
 *
 * This macro implements a delay-based debounce mechanism for encoder channels.
 *
 * @param _encCh_ Encoder channel to debounce.
 * @param _delay_ Delay in milliseconds for the debounce.
 */
#define ENC_Debounce(_encCh_, _delay_)		do{	\
												static uint32_t lastTime = 0;\
												uint32_t currentTime = (uint32_t)HAL_GetTick();\
												if((currentTime - lastTime)>= _delay_){\
													_encCh_++;\
													lastTime = currentTime;\
												}\
											}while(0)


/**
 * @brief Enumeration for encoder direction.
 */
typedef enum {
    ENCODER_DIR_NONE = 0,
    ENCODER_DIR_FORWARD,
    ENCODER_DIR_REVERSE
} EncoderDirection_te;

/**
 * @brief Enumeration for enabling or disabling the encoder.
 */
typedef enum {
	ENCODER_DISABLE = 0,
	ENCODER_ENABLE
}Encoder_EnableDisable_te;

/**
 * @brief Enumeration for encoder error codes.
 */
typedef enum{
	ENCODER_ERR_NONE = 0,
	ENCODER_ERR_NULL_PTR = -1,
	ENCODER_ERR_INVALID_VALUE = -2,
	ENCODER_ERR_TIM_START_FAILED = -3,
	ENCODER_ERR_TIM_STOP_FAILED = -4,
}Encoder_Error_te;

/**
 * @brief Structure for encoder input capture data.
 */
typedef struct Encoder_InputCapture{
	uint8_t is1stEadgCapturd;
	uint32_t count1;
	uint32_t count2;
	uint32_t diff;
	uint32_t diffBuf[5];
	uint32_t index;
	uint32_t lastDiff;
	uint8_t isCaptured;
	float refClk;
	float freq;
	FLTR_Median_ts medFltr;
	FLTR_MovingAverage_ts movAvgFltr;
}Encoder_InputCapture_ts;


/**
 * @brief Structure for encoder data and configuration.
 */
typedef struct {
	uint8_t enable;
	uint8_t isStarted;
	Encoder_Error_te error;
    uint16_t ppr;       // Encoder resolution
    uint16_t resolution;       // Encoder resolution
    volatile int32_t count;        // Current count
    uint32_t countPerRev;        // Current count
    volatile int32_t lastCount;        // Current count
    volatile uint32_t countZ;        // Current count
    uint32_t isZTriged;        // Current count
    volatile EncoderDirection_te dir; // 1 = down count, 0 =up countCurrent direction
    float rpmSet;                     // Calculated RPM
    float rpm;
    float rpmFromZ;
    float posAngle;						// Calculated position
    uint32_t last_timestamp;       // Last timestamp for RPM calculation
    TIM_HandleTypeDef *htim;        // Timer handle
    Encoder_InputCapture_ts chAInCap;

//    struct{
//    	uint8_t i;
//    	uint32_t dif[50];
//    	uint32_t med[50];
//    	uint32_t mov[50];
//    	float rpm[50];
//    }test;
} Encoder_ts;


/*Public Functions*/
Encoder_Error_te Encoder_Init(Encoder_ts *enc, TIM_HandleTypeDef *htim, uint16_t resolution) ;
void Encoder_SetResolution(Encoder_ts *enc, uint32_t resolution);
Encoder_Error_te Encoder_StartTimer(Encoder_ts *enc);
Encoder_Error_te Encoder_StopTimer(Encoder_ts *enc);
Encoder_Error_te Encoder_SetCount(Encoder_ts *enc, uint32_t count);
void Encoder_SetPPR(Encoder_ts *enc, uint32_t ppr);
void Encoder_Update(Encoder_ts *enc);
void Encoder_CalcRPM(Encoder_ts *enc);
EncoderDirection_te Encoder_GetDir(Encoder_ts *enc);
float Encoder_CalcPositionAngle(uint32_t count, uint32_t ppr);
void Encoder_IndexZCapture(Encoder_ts *enc);
void Encoder_ChACapture(Encoder_ts *enc, uint32_t count);

#endif /* SRC_ENCODER_ENCODER_H_ */
