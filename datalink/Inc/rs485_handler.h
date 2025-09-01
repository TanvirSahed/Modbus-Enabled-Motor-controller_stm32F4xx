///*
// * rs485_handler.h
// *
// *  Created on: Mar 21, 2024
// *      Author: Shakil Tanvir
// *
// * Description:
// * This header file provides declarations for functions and global variables
// * used in the Modbus communication process via RS485 using UART on STM32.
// * It supports DMA and interrupt-based communication handling, as well as
// * callbacks for both data reception and transmission completion.
// */
//
//#ifndef INC_RS485_MASTER_RS485_HANDLER_H_
//#define INC_RS485_MASTER_RS485_HANDLER_H_
//
//#include "stm32f4xx.h"
//#include "stdlib.h"
//#include <string.h>
//#include "modbus.h"
//
//
//
//
//
//// Define buffer sizes for Modbus communication
//#define MASTER_RX_BUFFER_SIZE 64
//#define MASTER_TX_BUFFER_SIZE 64
//
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart4;
//
//
///**
// * @brief Global variable to store the current UART instance (huart1 or huart2).
// * This is used in both the send and recv functions to determine which UART
// * instance should be used for communication.
// */
//extern UART_HandleTypeDef *current_uart;
//
//// Global buffers for UART data reception and transmission
//extern uint8_t RxData[MASTER_RX_BUFFER_SIZE];
//extern uint8_t TxData[MASTER_TX_BUFFER_SIZE];
//
//
//
//
//
///*
// * Function: Init_RS485
// * --------------------
// *  Initializes RS485 communication using UART. This function configures UART
// *  to use either DMA or interrupt mode to receive data until the idle frame is detected.
// *
// *  uarthandle: pointer to the UART handle used for RS485 communication.
// *
// *  Returns: None
// */
//void Init_RS485(UART_HandleTypeDef *uarthandle);
//
///*
// * Function: HAL_UARTEx_RxEventCallback
// * ------------------------------------
// *  This callback function is called when UART receives data and detects the idle frame.
// *  It handles the Modbus response by toggling an LED, processing the received data,
// *  and clearing the receive buffer.
// *
// *  huart: pointer to the UART handle that triggered the callback.
// *  Size: the number of bytes received.
// *
// *  Returns: None
// */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
//
///*
// * Function: HAL_UART_TxCpltCallback
// * ---------------------------------
// *  This callback function is called when UART has finished transmitting data.
// *  It can be used to perform post-transmission operations, such as toggling
// *  an indicator LED or resetting flags.
// *
// *  huart: pointer to the UART handle that completed transmission.
// *
// *  Returns: None
// */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
//
//
//#endif /* INC_RS485_MASTER_RS485_HANDLER_H_ */
