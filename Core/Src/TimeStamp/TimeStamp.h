/*
 * MH_Timer.h
 *
 *  Created on: Mar 28, 2023
 *      Author: Mahmudul Hasan (Russell)
 *      
 */

#ifndef INC_TIMESTAMP_H_
#define INC_TIMESTAMP_H_
#include "stdint.h"
#include "main.h"

typedef struct TS_Format{
	uint32_t hr;
	uint32_t min;
	uint32_t sec;
	uint32_t ms;
	uint32_t us;
} TS_Format;

void TS_Init(void);
void TS_Start(void);
void TS_Stop(void);

uint64_t TS_GetUS(void);
uint64_t TS_GetMS(void);
TS_Format TS_GetTime(void);
void TS_CatchOVF(void);





#endif /* INC_TIMESTAMP_H_ */

