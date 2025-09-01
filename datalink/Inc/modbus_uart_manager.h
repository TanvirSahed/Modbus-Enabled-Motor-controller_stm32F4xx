/*
 * modbus_uart_manager.h
 *
 *  Created on: Dec 9, 2024
 *      Author: Qbits
 */

#ifndef INC_MODBUS_UART_MANAGER_H_
#define INC_MODBUS_UART_MANAGER_H_


#include "modbus.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define STM_HAL 1 //STM hal layers activated
#define  DEBUG_ON 0
#define POLLING 0
#define INTERRUPT 0 //To send and receive data using INTERRUPT
#define DMA 1 //To send and receive data using DMA


// Maximum UART contexts
#define MAX_UARTS 1

//Modbus UART configurations
typedef enum{
	RTU_DATA_BIT_7=7,
	RTU_DATA_BIT_8,
	RTU_DATA_BIT_9
}SerialDataBit_t;

typedef enum{
	RTU_STOP_BIT_1=1,
	RTU_STOP_BIT_2,
}SerialStopBit_t;

typedef enum{
	RTU_PARITY_BIT_NONE='N',
	RTU_PARITY_BIT_ODD='O',
	RTU_PARITY_BIT_EVEN='E'
}SerialParity_t;


typedef struct{
	uint32_t baudRate;
	SerialDataBit_t dataBit;
	SerialStopBit_t stopBit;
	SerialParity_t parity;

}SerialCommParams;



// Modbus configuration for UART
typedef struct {
    UART_HandleTypeDef *uart;         // UART handle
    modbus_t *modbus_context;         // Modbus context
    modbus_mapping_t *modbus_mapping; // Modbus data mapping
    GPIO_TypeDef *ctrl_port;          // GPIO port for UART_CTRL pin
    uint16_t ctrl_pin;                // GPIO pin for UART_CTRL
    uint8_t rx_buffer[256];           // Receive buffer for UART
} UARTModbusConfig;

// Function prototypes
int Initialize_ModbusRTU(UART_HandleTypeDef *huart,SerialCommParams *serial, GPIO_TypeDef *ctrl_port, uint16_t ctrl_pin, int slave_id, int uart_index);
void Deinitialize_ModbusRTU(int uart_index);
void Handle_ModbusRequest(int uart_index);
void printUARTModbusConfig(const UARTModbusConfig *config);
void printAllUARTContexts(const UARTModbusConfig contexts[], size_t count);
void Assign_Modbus_Data(modbus_mapping_t *mapping, int uart_index);
int Reconfigure_ModbusSerialParams(int uart_index, const SerialCommParams *new_serial);

#endif /* INC_MODBUS_UART_MANAGER_H_ */
