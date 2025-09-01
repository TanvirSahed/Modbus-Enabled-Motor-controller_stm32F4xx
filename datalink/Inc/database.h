///*
// * database.h
// *
// *  Created on: Sep 5, 2024
// *      Author: Shakil Tanvir
// */
//
//#ifndef INC_DATABASE_H_
//#define INC_DATABASE_H_
//
//#include "modbus.h"
//#include "main.h"
//
//
//
//// Define the maximum number of registers and coils for Modbus mapping
//#define MAX_COILS           100
//#define MAX_DISCRETE_INPUTS 100
//#define MAX_INPUT_REGISTERS 100
//#define MAX_HOLDING_REGISTERS 100
//
//
//extern modbus_t *slave;// Exported Pointer to Modbus RTU context
//extern modbus_mapping_t *slavedata;//Exported Pointer to Modbus data mapping
//
//
///**
// * @brief Initializes the Modbus RTU context for a specific slave device.
// *
// * This function creates a Modbus RTU context using the specified UART handle,
// * sets the slave ID, and allocates memory for the Modbus data mappings. It also
// * initializes multiple coils, discrete inputs, input registers, and holding registers
// * with predefined values.
// *
// * @param huart Pointer to the UART_HandleTypeDef structure that contains
// *              the configuration information for the UART module.
// * @param Slave_ID The address of the Modbus slave device (1-247).
// *
// * @return None
// *
// * @note Ensure that the UART is properly initialized before calling this function.
// *       Error handling is done via the Error_Handler() function if any
// *       allocation or initialization fails.
// */
//void Open_ModbusRTU(UART_HandleTypeDef *huart, int Slave_ID);
//
//
//
//
//// Coils (0x) - Single-bit read/write
//void set_coil(modbus_mapping_t *mb_mapping, int coil_address, uint8_t value);
//uint8_t get_coil(modbus_mapping_t *mb_mapping, int coil_address);
//void set_multiple_coils(modbus_mapping_t *mb_mapping, int start_address, uint8_t *values, int count);
//
//// Discrete Inputs (1x) - Single-bit read-only
//void set_discrete_input(modbus_mapping_t *mb_mapping, int input_address, uint8_t value);
//uint8_t get_discrete_input(modbus_mapping_t *mb_mapping, int input_address);
//void set_multiple_discrete_inputs(modbus_mapping_t *mapping, int start_address, uint8_t *values, int count);
//
//// Input Registers (3x) - 16-bit read-only
//void set_input_register(modbus_mapping_t *mb_mapping, int reg_address, uint16_t value);
//uint16_t get_input_register(modbus_mapping_t *mb_mapping, int reg_address);
//void set_multiple_input_registers(modbus_mapping_t *mapping, int start_address, uint16_t *values, int count);
//
//// Holding Registers (4x) - 16-bit read/write
//void set_holding_register(modbus_mapping_t *mb_mapping, int reg_address, uint16_t value);
//uint16_t get_holding_register(modbus_mapping_t *mb_mapping, int reg_address);
//void set_multiple_holding_registers(modbus_mapping_t *mb_mapping, int start_address, uint16_t *values, int count);
//
//
//
//#endif /* INC_DATABASE_H_ */
