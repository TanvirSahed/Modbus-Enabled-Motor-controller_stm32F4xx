/*
 * datalink.h
 *
 *  Created on: Aug 16, 2024
 *      Author: Shakil Tanvir
 */

#ifndef INC_DATALINK_H_
#define INC_DATALINK_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "modbus_uart_manager.h"

extern UARTModbusConfig uart_contexts[MAX_UARTS];
extern UARTModbusConfig *currentuartmodbus;
/**
 * @brief Function to print buffer in the uart
*/
void print_buffer(unsigned char *buffer, size_t length);


int closeuart(int ctx, UART_HandleTypeDef *huart);
/**
 * @brief Function to send data over UART. This function replaces the POSIX
 * 'write' function. It dynamically selects the UART instance (huart1 or huart2)
 * that triggered the receive callback and sends data through that UART interface.
 *
 * @param __fd File descriptor (not used in this implementation, kept for POSIX compatibility).
 * @param __buf Pointer to the data buffer to be sent.
 * @param __nbyte Number of bytes to send.
 * @return int The number of bytes successfully sent or -1 in case of an error.
 */
int senduart(int __fd, const void *__buf, size_t __nbyte);

/**
 * @brief Function to receive data over UART. This function replaces the POSIX
 * 'read' function. It dynamically selects the UART instance (huart1 or huart2)
 * that triggered the receive callback and receives data from that UART interface.
 *
 * @param __fd File descriptor (not used in this implementation, kept for POSIX compatibility).
 * @param __buf Pointer to the buffer where received data will be stored.
 * @param __nbyte Number of bytes to receive.
 * @return int The number of bytes successfully read or -1 in case of an error.
 */
int recvuart(int __fd, void *__buf, size_t __nbyte);


UARTModbusConfig *get_uart_context(int fd);


#endif /* INC_DATALINK_H_ */
