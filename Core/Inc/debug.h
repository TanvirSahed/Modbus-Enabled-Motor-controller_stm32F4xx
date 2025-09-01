/*
 * debug.h
 *
 *  Created on: Dec 3, 2024
 *      Author: RusselPC
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_
#include <stdint.h>



#define DEBUG_ENABLE		1
#define DEBUG_DIABLE		0

int dbg_print(char *format, ...);
void dbg_trace(unsigned char enable, char *format, ...);
void dbg_enable(void);
void dbg_disable(void);
uint8_t dbg_isEnable(void);
#endif /* INC_DEBUG_H_ */
