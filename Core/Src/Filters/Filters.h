/*
 * Filters.h
 *
 *  Created on: Mar 9, 2022
 *      Author: RASEL_EEE
 */

/*
 * Instruction to include the DSP libraries and header files
 *  Here are the steps to follow to copy the DSP drivers into the project:
 *
 *  Step-1: Support DSP in the project workspace
 *  	a) In the generated project, create a folder under ..\Drivers\CMSIS named DSP.
 *
 *  	b) Copy <STM32Cube_Repository>\STM32Cube_FW_F4_V.X.XX.X\Drivers\CMSIS\DSP\Include and paste in in the created folder
 *
 *  	c) Copy <STM32Cube_Repository>\STM32Cube_FW_F4_V.X.XX.X\Drivers\CMSIS\Lib and paste it under ..\Drivers\CMSIS.
 *
 *  After being copied and refreshing the project, the added folders appear automatically in the STM32CubeIDE workspace:
 *
 *  Step-2: Include paths
 * 		The purpose of this part is to describe the steps to follow in order to support new header files:
 *
 * 		a) Select the project from the Project Explorer section
 *
 * 		b) From Project menu or File menu, go to Project properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Include paths.
 *
 * 		c) Click on “Add” to include the new paths.
 *
 * 		d) Add ../Drivers/CMSIS/DSP/Include path
 *
 * 	Step-3: Update libraries
 * 	The purpose of this part is to describe how to update the libraries in the project.
 * 	The first step is to add the libraries' path in the project settings:
 * 		a) Select the project from the Project Explorer section, then from Project menu or File menu, go to Properties > C/C++ Build >
 * 		Settings > Tool Settings > MCU CGC Linker > Libraries > Library search Path > Add.
 *
 * 		b) Select the GCC library present in the workspace path: ../Drivers/CMSIS/Lib/GCC.
 *
 * 		c) Now, to add the specific library to work with, select the project from the Project Explorer section.
 *
 * 		d) From Project menu or File menu, go to Properties > C/C++ Build > Settings > Tool Settings >
 * 		MCU CGC Linker > Libraries > Libraries (-l) > Add and insert the following library: "arm_cortexM3l_math"(as per MCU, CM3 for cortex-m3) .
 *
 *  Step-4: The final step, is to add the “ARM_MATH_CM3”(as per MCU, CM3 for cortex-m3) symbol to the project
 *  	a) From Project menu or File menu, go to Project properties > C/C++ Build > Settings > Tool Settings > MCU GCC Compiler > Preprocessor
 *  	b) In "Define symbols (-D)" -> Add > "ARM_MATH_CM3" > Apply and Close
 *
 * */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include "main.h"
//#include "OD_Defines.h"
#include "stdlib.h"



//#define FILTER_FIR_ENABLED
#ifdef FILTER_FIR_ENABLED
	#include "arm_math.h"
#endif

#define FLTR_MED_WINDOW_SIZE_MAX 			21U		// Median filter window size
#define FLTR_MOV_WINDOW_SIZE_MAX 			21U		// Moving average filters window size

#define FLTR_MED_WINDOW_SIZE 			9U		// Default median filter window size
#define FLTR_MOV_WINDOW_SIZE 			9U		// Default moving average filters window size

#define __FLTR_CHECK_MED_WIN_SIZE(med)		(((med)->windowSize != 0) ||	\
											(((med)->windowSize%2) != 0) ||	\
											((med)->windowSize <= FLTR_MED_WINDOW_SIZE_MAX))
#define __FLTR_CHECK_MOVAVG_WIN_SIZE(mov)	(((mov)->windowSize != 0) ||	\
											((mov)->windowSize <= FLTR_MOV_WINDOW_SIZE_MAX))



/*Debounce filter*/
//#define DI_BOUNCE_COUNT_MAX			DEF_DI_BOUNCE_COUNT_MAX	// digital input max bounce counter
//#define FLTR_DEFAULT_INPUT_STATE		0U



#ifdef FILTER_FIR_ENABLED
/*
 * FIR Filter configuration
 * */
#define FLTR_FIR_SAMPLE_LENGTH			260U		// fir filter buffer size
#define FLTR_FIR_BLOCK_SIZE         	20U			// fir filter window or block size
#define FLTR_FIR_NUM_TAPS      		    9U			// number of coefficients of fir filter


/* coefficients are calculated from MATLAB
 * Where:
 * 		Sampling frequency: 5000Hz
 * 		Cutoff frequency: 4000Hz
 * 		Nyquist frequency: 24000Hz
 * 		No. of order(N): 8
 * MATLAB gives the coefficient N+1 = 8+1=9
 */

typedef struct Fltr_FIRFilter{
	arm_fir_instance_f32 instance;
	float32_t inputBuffer[FLTR_FIR_SAMPLE_LENGTH];
	float32_t outputBuffer[FLTR_FIR_SAMPLE_LENGTH];
//	uint32_t samplingFreq;				// sampling frequency
//	uint32_t cutoffFreq;				// cutoff frwqurency
//	uint32_t blockSize;
//	uint32_t numBlocks;
//
}Fltr_FIRFilter;
#endif


typedef enum FLTR_Error{
	FLTR_ERROR_NONE = 0,
	FLTR_ERROR_NULL_PTR,
	FLTR_ERROR_INVALID_WINDOW_SIZE,
}FLTR_Error;


typedef struct FLTR_Median{
	float window[FLTR_MED_WINDOW_SIZE_MAX];		// median filter window
	uint8_t windowSize;
	float value;									// current value
	float nextValue;								// next value
	uint8_t position;								// target value position
}FLTR_Median_ts;


typedef struct FLTR_MovingAverage{
	float window[FLTR_MOV_WINDOW_SIZE_MAX];		// median filter window
	uint8_t windowSize;
	float value;									// current value
	float nextValue;								// next value
	uint8_t position;								// target value position
	uint32_t sum;									// sum of the samples
}FLTR_MovingAverage_ts;


typedef struct FLTR_Debounce{
	uint16_t counter;
	uint16_t countMax;
	uint16_t finalState;
	uint16_t currentState;
	uint8_t isInterrupt;
}FLTR_Debounce;

#ifdef FILTER_FIR_ENABLED
void FIRFilterInit(Fltr_FIRFilter *fir);
void FIRFilterPrecess(Fltr_FIRFilter *fir);
#endif

FLTR_Error FLTR_ApplyMedian(FLTR_Median_ts *params);
FLTR_Error FLTR_ApplyMovAvg(FLTR_MovingAverage_ts *maParams);
void FLTR_InitMedian(FLTR_Median_ts *mParams, uint8_t windowSIze);
void FLTR_InitMovAvg(FLTR_MovingAverage_ts *maParams, uint8_t windowSIze);

//Debounce filter
uint8_t FLTR_ApplyDebounce(FLTR_Debounce *debounce);
void FLTR_StartDebounceExti(FLTR_Debounce *debounce);
uint8_t FLTR_ApplyDebounceExti(FLTR_Debounce *debounce);
void FLTR_ClearDebounce(FLTR_Debounce *debounce);



#endif /* OD_LIB_INC_FILTERS_H_ */
