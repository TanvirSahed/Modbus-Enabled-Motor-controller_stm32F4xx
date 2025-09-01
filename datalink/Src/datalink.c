/*
 * datalink.c
 *
 *  Created on: August 17,2024
 *      Author: Shakil Tanvir
 */
#include <string.h>
#include "datalink.h"
#include "modbus.h"
#include "rs485_handler.h"
#include "modbus-rtu.h"

#include "modbus_uart_manager.h"

extern UART_HandleTypeDef huart1;
// Global variable to store the current UART handle (huart1 or huart2)
UART_HandleTypeDef *current_uart = NULL;


void print_buffer(unsigned char *buffer, size_t length) {
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
}

int closeuart(int ctx, UART_HandleTypeDef *huart) {


	//Hardfault error occurs here
//	if (HAL_UART_DeInit(huart) == HAL_OK) {
//		return 1;
//	}
	return 0;
}

//// Function to replace the POSIX 'write' function
//int senduart(int __fd, const void *__buf, size_t __nbyte) {
//	// Ensure the buffer is valid and the byte count is greater than 0
//	if (__buf == NULL || __nbyte == 0) {
//#if DEBUG_ON
//		memcpy(&TxData, __buf, __nbyte);
//		printf("error sending data ");
//		print_buffer(&TxData, sizeof(TxData));
//#endif
//
//		return -1;
//	}
//
//#if DEBUG_ON
//	memcpy(&TxData, __buf, __nbyte);
//	printf("sending data ");
//	print_buffer(&TxData, sizeof(TxData));
//#endif
//	memcpy(&TxData, __buf, __nbyte);
//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
//#if DMA
//	if (HAL_UART_Transmit_DMA(current_uart, (uint8_t*) __buf, __nbyte)
//			== HAL_OK) {
//
//		return __nbyte;  // Return the number of bytes written
//
//#elif INTERRUPT
//	if (HAL_UART_Transmit_IT(current_uart, (uint8_t*) __buf, __nbyte
//			== HAL_OK) {
//		return __nbyte;  // Return the number of bytes written
//
//#else
//	// Use HAL_UART_Transmit to send data over UART
//	if (HAL_UART_Transmit(current_uart, (uint8_t*) __buf, __nbyte, HAL_MAX_DELAY)
//			== HAL_OK) {
//		return __nbyte;  // Return the number of bytes written
//		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//#endif
//	} else {
//		return -1;  // Return -1 in case of an error
//	}
//}

// Function to replace the POSIX 'read' function
int recvuart(int __fd, void *__buf, size_t __nbyte) {
	// Ensure the buffer is valid and the byte count is greater than 0
	if (__buf == NULL || __nbyte == 0) {
		return -1;
	}

	// Use HAL_UART_Receive to receive data over UART
	//if (HAL_UARTEx_ReceiveToIdle_IT(current_uart, (uint8_t *)__buf, __nbyte) == HAL_OK)
	if (HAL_UART_Receive(&huart1, __buf, __nbyte, HAL_MAX_DELAY) == HAL_OK) {
		//memcpy(&RxData, __buf, __nbyte);
#if DEBUG_ON
		printf("received data");
		print_buffer(&RxData, sizeof(RxData));
#endif
		return __nbyte;  // Return the number of bytes read
	} else {
#if DEBUG_ON
		printf("error receiving data\n");
#endif
		return -1;  // Return -1 in case of an error
	}
}

int senduart(int __fd, const void *__buf, size_t __nbyte) {
    // Validate input parameters
    if (__buf == NULL || __nbyte == 0) {
#if DEBUG_ON
        printf("Error: Invalid buffer or byte count\n");
#endif
        return -1;
    }

// 	Get the current UART context from __fd (e.g., use it as an index or handle)
//  UARTModbusConfig *ctx = get_uart_context(__fd); // Implement a function to map __fd to UARTConfig
//  printUARTModbusConfig(&ctx);

    //printUARTModbusConfig(&currentuartmodbus);
    if (currentuartmodbus == NULL) {
        printf("Error: Invalid UART context\n");
        return -1;
    }

    // Set UART control pin to transmission mode
    HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin, GPIO_PIN_RESET);

#if DEBUG_ON
    printf("Sending data: ");
    print_buffer((unsigned char *)__buf, __nbyte);
#endif

    // Transmit data using the appropriate method
#if DMA
    if (HAL_UART_Transmit_DMA(currentuartmodbus->uart, (uint8_t *)__buf, __nbyte) == HAL_OK) {
    //if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)__buf, __nbyte) == HAL_OK) {

        HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin, GPIO_PIN_SET); // Set back to receiving mode
        return __nbyte; // Return the number of bytes written
    }
#elif INTERRUPT
    if (HAL_UART_Transmit_IT(currentuartmodbus->uart, (uint8_t *)__buf, __nbyte) == HAL_OK) {
        HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin, GPIO_PIN_SET); // Set back to receiving mode
        return __nbyte; // Return the number of bytes written
    }
#else
    if (HAL_UART_Transmit(ctx->uart_handle, (uint8_t *)__buf, __nbyte, HAL_MAX_DELAY) == HAL_OK) {
        HAL_GPIO_WritePin(ctx->ctrl_port, ctx->ctrl_pin, GPIO_PIN_SET); // Set back to receiving mode
        return __nbyte; // Return the number of bytes written
    }
#endif

    // Handle transmission error
    HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin, GPIO_PIN_SET); // Ensure the pin is reset to receiving mode
    printf("Error: Failed to transmit data\n");
    return -1;
}


UARTModbusConfig *get_uart_context(int fd) {
    if (fd < 0 || fd >= MAX_UARTS) {
        return NULL; // Invalid fd
    }
    // print uart context
    printf("UART Contex: %d \n", fd);
    return &uart_contexts[fd]; // Assuming fd corresponds to the UART index
}
