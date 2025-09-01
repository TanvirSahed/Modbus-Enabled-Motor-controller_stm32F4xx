/*
 * @file debug.c
 * @brief Debug utility functions for UART-based logging.
 *
 * This file provides a set of functions to enable/disable debug output
 * and to print debug messages over UART. It uses `USART2` for transmission.
 * Debug messages can be conditionally compiled or enabled at runtime.
 *
 * @author RusselPC
 * @date Dec 3, 2024
 */
#include "debug.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"

extern UART_HandleTypeDef huart2;
static char str[256] = {0};
uint8_t dbgEnable = 0;


/**
 * @brief Prints a formatted debug message if debugging is enabled.
 *
 * Outputs the message over UART using `HAL_UART_Transmit`.
 *
 * @param format printf-style format string.
 * @param ... Variable arguments corresponding to the format.
 * @return Number of characters transmitted, or 0 if debug is disabled.
 */
int dbg_print(char *format, ...){
	if(!dbgEnable){return -1;}

	va_list aptr;
	int32_t ret;


	va_start(aptr, format);
	ret = vsprintf(str, format, aptr);
	va_end(aptr);
	if(ret>0){
		HAL_UART_Transmit(&huart2, (uint8_t *)str, ret, ret);
		memset((char *)str,0,ret);
	}

	return ret;
}

/**
 * @brief Prints a formatted debug message if both global and local debug flags are enabled.
 *
 * @param enable Local flag to determine whether to output the message.
 * @param format printf-style format string.
 * @param ... Variable arguments corresponding to the format.
 */
void dbg_trace(unsigned char enable, char *format, ...){
	if(!dbgEnable){return;}
	if(enable){
		va_list aptr;
		int32_t ret;


		va_start(aptr, format);
		ret = vsprintf(str, format, aptr);
		va_end(aptr);
		if(ret>0){
			HAL_UART_Transmit(&huart2, (uint8_t *)str, ret, ret);
			memset((char *)str,0,ret);
		}
	}
}


/**
 * @brief Enables debug output.
 */
void dbg_enable(void){
	dbgEnable = 1;
}


/**
 * @brief Disables debug output.
 */
void dbg_disable(void){
	dbgEnable = 0;
}


/**
 * @brief Checks if debug output is enabled.
 *
 * @return 1 if enabled, 0 if disabled.
 */
uint8_t dbg_isEnable(void){
	return dbgEnable;
}
