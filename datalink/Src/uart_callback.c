/*
 * uart_callback.c
 *
 *  Created on: Dec 9, 2024
 *      Author: Shakil Tanvir
 */

#include "uart_callback.h"

extern UARTModbusConfig uart_contexts[];

UARTModbusConfig *currentuartmodbus = NULL;

// UART receive callback function
void Modbus_UARTE_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	for (int i = 0; i < MAX_UARTS; i++) {
		if (huart == uart_contexts[i].uart) {
			currentuartmodbus = &uart_contexts[i];

//            printf("")
			// Handle the Modbus request for this UART
			// Handle_ModbusRequest(i);

			if (i < 0 || i >= MAX_UARTS) {
				printf("Invalid UART index: %d\n", i);
				return;
			}


			/*Valid slave ID checking*/
			uint8_t slaveIdRcvd =  currentuartmodbus->rx_buffer[0];
			uint8_t slaveIdLocal = modbus_get_slave(currentuartmodbus->modbus_context);
//			printf("slaveIdRcvd: %d, slaveIdLocal: %d\r\n",slaveIdRcvd, slaveIdLocal);
			if(slaveIdLocal != -1 && slaveIdLocal == slaveIdRcvd){

				// Set UART_CTRL to receiving mode
				HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin,
						GPIO_PIN_SET);
	//			int rc = 1;

				// Set UART_CTRL to transmission mode
				HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin,
						GPIO_PIN_RESET);

				modbus_reply(currentuartmodbus->modbus_context, currentuartmodbus->rx_buffer, Size,
						currentuartmodbus->modbus_mapping);

				// Return to receiving mode
				HAL_GPIO_WritePin(currentuartmodbus->ctrl_port, currentuartmodbus->ctrl_pin,
						GPIO_PIN_SET);
			}

			memset(currentuartmodbus->rx_buffer, 0, Size);
				// Re-enable DMA for the next reception
			if (HAL_UARTEx_ReceiveToIdle_DMA(currentuartmodbus->uart, currentuartmodbus->rx_buffer,
					sizeof(currentuartmodbus->rx_buffer)) != HAL_OK) {
				printf("Error re-enabling DMA for UART %d\n", i);
			}

			return;
		}
	}


}
