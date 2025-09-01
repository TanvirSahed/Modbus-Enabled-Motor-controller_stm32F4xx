/*
 * uart_callback.h
 *
 *  Created on: Dec 9, 2024
 *      Author: Shakil Tanvir
 */

#ifndef SRC_UART_CALLBACK_H_
#define SRC_UART_CALLBACK_H_

#include "modbus_uart_manager.h"
#include "stm32f4xx_hal.h"

// UART receive callback function
void Modbus_UARTE_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* SRC_UART_CALLBACK_H_ */
