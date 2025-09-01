/*
 * Filters.c
 *
 *  Created on: Mar 9, 2022
 *      Author: RASEL_EEE
 */

#include "Filters.h"
#include "stdio.h"
#include "string.h"
#include  "math.h"

#define pi 3.1416f
static float winBufTemp[FLTR_MED_WINDOW_SIZE_MAX] = {0};

int CompFunc (const void * a, const void * b);


#ifdef FILTER_FIR_ENABLED
//
//
//void Filter_IIRFistOrederFilterCalc(Fltr_IIRFilter *irrFilter){
//	double rc = (1.0f/(2.0f*pi*irrFilter->cutoffFreq));		// rc time constant
//	double ts =  (1.0f/irrFilter->samplingFreq);			// sampling time
//	irrFilter->alpha = (ts/(ts+rc));
//}
//
//void Filter_IIRFirstOrderFilter(Fltr_IIRFilter *irrFilter, uint16_t *outputBuffer, uint16_t *inputBuffer, uint16_t size){
//	outputBuffer[0] = irrFilter->alpha * inputBuffer[0];	// for the very first value
//	for(uint16_t i = 1; i < size; i++){
//		outputBuffer[i] = outputBuffer[i-1] + irrFilter->alpha * (inputBuffer[i]-outputBuffer[i-1]);
//	}
//}

static float32_t FLTR_FirState[FLTR_FIR_BLOCK_SIZE + FLTR_FIR_NUM_TAPS - 1];
const float32_t Fltr_FirCoeffs[FLTR_FIR_NUM_TAPS] = {
	0.0085f, 0.0352f, 0.1149f, 0.2127f, 0.2574f, 0.2127f, 0.1149f, 0.0352f, 0.0085f
};


void FIRFilterInit(Fltr_FIRFilter *fir){
	arm_fir_init_f32(&fir->instance, FLTR_FIR_NUM_TAPS, (float32_t *)&Fltr_FirCoeffs[0], &FLTR_FirState[0], FLTR_FIR_BLOCK_SIZE);
}

void FIRFilterPrecess(Fltr_FIRFilter *fir){

  for(uint16_t i=0U; i < (FLTR_FIR_SAMPLE_LENGTH/FLTR_FIR_BLOCK_SIZE); i++)
  {
	arm_fir_f32(&fir->instance, fir->inputBuffer + (i * FLTR_FIR_BLOCK_SIZE),  fir->outputBuffer + (i * FLTR_FIR_BLOCK_SIZE), FLTR_FIR_BLOCK_SIZE);
  }
}
#endif



const float fltrMedWindowMidPoint = ((FLTR_MED_WINDOW_SIZE_MAX-1U)/2U);

/**
  * @brief  Median filter
  * @param  mParams: median parameters
  * @retval filtered value
  */
FLTR_Error FLTR_ApplyMedian(FLTR_Median_ts *mParams){
	if(mParams == NULL){
		return FLTR_ERROR_NULL_PTR;
	}else if((mParams->windowSize == 0) ||
			((mParams->windowSize%2) == 0) ||
			(mParams->windowSize >= FLTR_MED_WINDOW_SIZE_MAX)){
		return FLTR_ERROR_INVALID_WINDOW_SIZE;
	}

	mParams->window[mParams->position++] = mParams->nextValue;					// set the new sample to the window buffer
	memcpy(winBufTemp, mParams->window, mParams->windowSize*sizeof(winBufTemp[0]));		// copy the samples from the window buffer to the local buffer
	qsort(winBufTemp, mParams->windowSize, sizeof(winBufTemp[0]), CompFunc);				// Arrange the local buffer elements in ascending order
	if(mParams->position >= mParams->windowSize)	mParams->position = 0;			// if window buffer is full, set the position to zero for adding new sample to the buffer
	mParams->value = winBufTemp[((mParams->windowSize-1U)/2U)];									// getting the center value of the buffer and return
	return FLTR_ERROR_NONE;
}
//184152	   1288	  21136	 206576	  326f0
//183328	   1288	  21136	 205752	  323b8
/**
  * @brief  Moving Average filter
  * @param  maParams: Moving average parameters
  * @retval filtered value
  */
FLTR_Error FLTR_ApplyMovAvg(FLTR_MovingAverage_ts *maParams){
	if(maParams == NULL){
		return FLTR_ERROR_NULL_PTR;
	}else if((maParams->windowSize == 0) ||
			(maParams->windowSize >= FLTR_MOV_WINDOW_SIZE_MAX)){
		return FLTR_ERROR_INVALID_WINDOW_SIZE;
	}
	maParams->sum = maParams->sum - maParams->window[maParams->position] + maParams->nextValue;		// add the new value and subtract the old value from the sum of the elements
	maParams->window[maParams->position++] = maParams->nextValue;									// set the new sample to the window buffer
	if(maParams->position >=  maParams->windowSize)	maParams->position = 0;								// if window buffer is full, set the position to zero for adding new sample to the buffer
	maParams->value = maParams->sum/ maParams->windowSize;//(uint16_t)ceil((float)maParams->sum/(float)FLTR_MOV_WINDOW_SIZE);															// calculate the average and return
	return FLTR_ERROR_NONE;
}


/**
  * @brief  Clears median filter
  * @param  mParams: median filter parameters
  * @retvalnone
  */
void FLTR_InitMedian(FLTR_Median_ts *mParams, uint8_t windowSIze){
	mParams->value = 0.0;
	mParams->nextValue = 0.0;
	mParams->position = 0.0;
	mParams->windowSize = windowSIze;
	memset(mParams->window, 0, FLTR_MED_WINDOW_SIZE_MAX*sizeof(mParams->window[0]));
}

/**
  * @brief  Clears moving average filter
  * @param  mParams: moving average filter parameters
  * @retvalnone
  */
void FLTR_InitMovAvg(FLTR_MovingAverage_ts *maParams, uint8_t windowSIze){
	maParams->value = 0.0;
	maParams->nextValue = 0.0;
	maParams->position = 0.0;
	maParams->sum = 0.0;
	maParams->windowSize = windowSIze;
	memset(maParams->window, 0, FLTR_MOV_WINDOW_SIZE_MAX*sizeof(maParams->window[0]));
}


/**
  * @brief  debounce filter to remove the bounce of the digital inputs
  * @param  debounce: debounce filter struct
  * @retval none
  */
uint8_t FLTR_ApplyDebounce(FLTR_Debounce *debounce){

	if((debounce->finalState == debounce->currentState) && (debounce->counter > 0U)){
		debounce->counter --;											// resets the bounce counter if the input state is not changed
	}
	if(debounce->finalState != debounce->currentState){
		debounce->counter++;												// increments the bounce counter value if the input state is changed
	}

	if(debounce->counter >= debounce->countMax){						// if debounce counter is exceed the max limit
		debounce->isInterrupt = 0U;
		debounce->counter = 0U;											// reset the counter
		debounce->finalState = debounce->currentState;										// update the last state variable
		return 1U;
	}
	return 0U;
}


void FLTR_StartDebounceExti(FLTR_Debounce *debounce){
	debounce->isInterrupt = 1U;
}

//void FLTR_DebounceTimerExit(FLTR_Debounce *debounce){
//		debounce->counter ++;											// reset the counter
//}
//
//void FLTR_CheckDebouncedExit(FLTR_Debounce *debounce){
//	if(debounce->counter >= debounce->countMax){						// if debounce counter is exceed the max limit
//		debounce->counter = 0U;											// reset the counter
//		debounce->finalState = debounce->currentState;										// update the last state variable
//	}
//}
/**
  * @brief  debounce filter for external interrupt
  * @param  debounce: debounce filter struct
  * @retval none
  */
uint8_t FLTR_ApplyDebounceExti(FLTR_Debounce *debounce){
	if(debounce->isInterrupt > 0U){
		return FLTR_ApplyDebounce(debounce);
	}
	return 0U;
}


/**
  * @brief  clear debounce filter parameter
  * @param  debounce: debounce filter struct
  * @retval none
  */
void FLTR_ClearDebounce(FLTR_Debounce *debounce){
	debounce->counter = 0;
	debounce->countMax = 0;
	debounce->currentState = 0;
	debounce->finalState = 0;
	debounce->isInterrupt = 0U;
}

/**
  * @brief  compare to numbers
  * @param a,b: two numbers
  * @retval return value can be positive or negative, if positive a is grater else b is grater
  */
int CompFunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}


