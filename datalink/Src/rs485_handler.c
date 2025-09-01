///*
// * modbus_handler.c
// *
// *  Created on: Mar 21, 2024
// *      Author: Shakil Tanvir
// */
//
//#include "rs485_handler.h"
//#include "main.h"
//#include "string.h"
//#include "stdio.h"
//#include "stm32f4xx_hal.h"
//#include "database.h"
//
//// Define buffer size for RX and TX data
//uint8_t RxData[MASTER_RX_BUFFER_SIZE];  // Buffer to store received data
//uint8_t TxData[MASTER_TX_BUFFER_SIZE]; // Buffer to store data to be transmitted
//
//
//
//// Function to initialize RS485 communication via UART
//void Init_RS485(UART_HandleTypeDef *uarthandle) {
//
//	// Reset LD1 (indicating start of initialization)
//	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, RESET);
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
//	HAL_GPIO_WritePin(UART1_CTRL_GPIO_Port, UART1_CTRL_Pin, RESET);
//	HAL_GPIO_WritePin(UART2_CTRL_GPIO_Port, UART2_CTRL_Pin, RESET);
//	HAL_GPIO_WritePin(UART4_CTRL_GPIO_Port, UART4_CTRL_Pin, RESET);
//	// Choose between DMA or Interrupt-based receive to idle
//#if DMA
//	// Initialize UART receive with DMA, waits for data till the idle frame
//	HAL_UARTEx_ReceiveToIdle_DMA(uarthandle, RxData, sizeof(RxData));
//#elif INTERRUPT
//    // Initialize UART receive with interrupt, waits for data till the idle frame
//    HAL_UARTEx_ReceiveToIdle_IT(uarthandle, RxData, sizeof(RxData));
//#else
//     // Default to DMA if no flag is defined
//    HAL_UARTEx_ReceiveToIdle_DMA(uarthandle, RxData, sizeof(RxData));
//#endif
//}
//
//int rc; // Variable for Modbus reply function result
///********** Callback when data is received via UART **********/
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//	//Process UART1 Data
//	if (huart == &huart1) {
//
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//
//		// Store the current UART handle (for transmission)
//		current_uart = huart;
//		// Respond to the Modbus master
//
//		// modbus_reply(conext1, RxData, rc, database1);
//		modbus_reply(slave, RxData, rc, slavedata);
//		memset(&RxData, '0', sizeof(RxData));
//
//
//
//		HAL_UARTEx_ReceiveToIdle_DMA(current_uart, RxData, sizeof(RxData));
//	}
//	//Process UART2 Data
//	if (huart == &huart2) {
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//
//		// Respond to the Modbus master with the received data
//		// modbus_reply(conext1, RxData, rc, database1);
//		modbus_reply(slave, RxData, rc, slavedata);
//		// Clear the receive buffer after handling the received data
//		memset(&RxData, '0', sizeof(RxData));
//		// Store the current UART handle (for transmission)
//		current_uart = huart;
//
//		// Re-enable UART to continue receiving data using DMA
//		HAL_UARTEx_ReceiveToIdle_DMA(current_uart, RxData, sizeof(RxData));
//	}
//	//Process UART4 Data
//	if (huart == &huart4) {
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//
//		// Respond to the Modbus master
//		// modbus_reply(conext1, RxData, rc, database1);
//		memset(&RxData, '0', sizeof(RxData));
//
//		// Store the current UART handle (for transmission)
//		current_uart = huart;
//
//		HAL_UARTEx_ReceiveToIdle_DMA(current_uart, RxData, sizeof(RxData));
//	}
//
//
//}
//
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//	// Check if the transmission is complete on huart2
//	if (huart == &huart2) {
//		/*
//		 * Write code here to handle after the full buffer has been transmitted.
//		 * For example, you could reset flags or prepare the system for the next transmission.
//		 */
//
//		// Reset LD2 to indicate completion of the transmission process
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//	}
//
//	// Check if the transmission is complete on huart1
//	if (huart == &huart1) {
//		/*
//		 * Write code here to handle after the full buffer has been transmitted.
//		 * For example, you could reset flags or prepare the system for the next transmission.
//		 */
//
//		// Reset LD1 to indicate completion of the transmission process
//		// HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//	}
//
//
//}
//
