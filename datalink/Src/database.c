///*
// * database.c
// *
// * Created on: Sep 5, 2024
// * Author: Shakil Tanvir
// *
// * Description:
// * This file implements Modbus RTU communication functions for managing coils,
// * discrete inputs, input registers, and holding registers. It includes functions
// * to set and get values for these registers, as well as initializing the Modbus
// * context with a specified UART interface.
// *
// * Notes:
// * - Ensure that the Modbus library is correctly linked in your project.
// * - Proper error handling is done via Error_Handler() function.
// */
//
//
//#include "database.h"
//#include "stdio.h"
//
//
//
//modbus_t *slave;            // Pointer to Modbus RTU context
//modbus_mapping_t *slavedata; // Pointer to Modbus data mapping
//
//
//
//void Open_ModbusRTU(UART_HandleTypeDef *huart, int Slave_ID)
//{
//    // Create Modbus RTU context (UART is initialized to 115200 baud, 8N1)
//	//slave = modbus_new_rtu(&huart1, 115200, 'N', 8, 1); //This one is better or not??
//    slave = modbus_new_rtu(huart, 115200, 'N', 8, 1);
//    if (slave == NULL) {
//        Error_Handler();
//    }
//
//    // Set slave address
//    modbus_set_slave(slave, Slave_ID); // Slave ID = 2
//
//    // Allocate memory for Modbus mapping (adjust sizes as necessary)
//    slavedata = modbus_mapping_new(MAX_COILS, MAX_DISCRETE_INPUTS, MAX_INPUT_REGISTERS, MAX_HOLDING_REGISTERS); // 100 holding registers, 100 input registers
//    if (slavedata == NULL) {
//        Error_Handler();
//    }
//
//
//    // Allocate memory for Modbus mapping (adjust sizes as necessary)
//    slavedata = modbus_mapping_new(10, 10, 100, 100); // 100 holding registers, 100 input registers
//    if (slavedata == NULL) {
//        Error_Handler();
//    }
//
//    // Example of setting multiple coil values
//        uint8_t coil_values[] = {1, 0, 1, 1, 0};  // Values to set in coils
//        set_multiple_coils(slavedata, 10, coil_values, sizeof(coil_values) / sizeof(coil_values[0]));
//
//        // Example of setting multiple discrete input values
//        uint8_t discrete_input_values[] = {1, 1, 0, 1, 0};  // Values to set in discrete inputs
//        set_multiple_discrete_inputs(slavedata, 20, discrete_input_values, sizeof(discrete_input_values) / sizeof(discrete_input_values[0]));
//
//        // Example of setting multiple input register values
//        uint16_t input_register_values[] = {100, 200, 300};  // Values to set in input registers
//        set_multiple_input_registers(slavedata, 30, input_register_values, sizeof(input_register_values) / sizeof(input_register_values[0]));
//
//        // Example of setting multiple holding register values
//        uint16_t holding_register_values[] = {1000, 2000, 3000};  // Values to set in holding registers
//        set_multiple_holding_registers(slavedata, 40, holding_register_values, sizeof(holding_register_values) / sizeof(holding_register_values[0]));
//
//}
//
//
//// Coils (0x) - Single-bit read/write
//void set_coil(modbus_mapping_t *mb_mapping, int coil_address, uint8_t value) {
//    if (coil_address >= 0 && coil_address < mb_mapping->nb_bits) {
//        mb_mapping->tab_bits[coil_address] = value; // Set coil value
//    } else {
//        printf("Invalid coil address: %d\n", coil_address); // Error message for invalid address
//    }
//}
//
//uint8_t get_coil(modbus_mapping_t *mb_mapping, int coil_address) {
//    if (coil_address >= 0 && coil_address < mb_mapping->nb_bits) {
//        return mb_mapping->tab_bits[coil_address]; // Return coil value
//    } else {
//        printf("Invalid coil address: %d\n", coil_address); // Error message for invalid address
//        return 0;
//    }
//}
//
//void set_multiple_coils(modbus_mapping_t *mb_mapping, int start_address, uint8_t *values, int count) {
//    if (start_address >= 0 && (start_address + count) <= mb_mapping->nb_bits) {
//        memcpy(&mb_mapping->tab_bits[start_address], values, count * sizeof(uint8_t)); // Set multiple coils
//    } else {
//        printf("Invalid coil address range: %d to %d\n", start_address, start_address + count - 1); // Error message for invalid range
//    }
//}
//
//// Discrete Inputs (1x) - Single-bit read-only
//void set_discrete_input(modbus_mapping_t *mb_mapping, int input_address, uint8_t value) {
//    if (input_address >= 0 && input_address < mb_mapping->nb_input_bits) {
//        mb_mapping->tab_input_bits[input_address] = value; // Set discrete input value
//    } else {
//        printf("Invalid discrete input address: %d\n", input_address); // Error message for invalid address
//    }
//}
//
//uint8_t get_discrete_input(modbus_mapping_t *mb_mapping, int input_address) {
//    if (input_address >= 0 && input_address < mb_mapping->nb_input_bits) {
//        return mb_mapping->tab_input_bits[input_address]; // Return discrete input value
//    } else {
//        printf("Invalid discrete input address: %d\n", input_address); // Error message for invalid address
//        return 0;
//    }
//}
//
//void set_multiple_discrete_inputs(modbus_mapping_t *mapping, int start_address, uint8_t *values, int count) {
//    for (int i = 0; i < count; i++) {
//        if (start_address + i < MAX_DISCRETE_INPUTS) {
//            mapping->tab_input_bits[start_address + i] = values[i]; // Set multiple discrete inputs
//        }
//    }
//}
//
//// Function to set multiple input registers
//void set_multiple_input_registers(modbus_mapping_t *mapping, int start_address, uint16_t *values, int count) {
//    for (int i = 0; i < count; i++) {
//        if (start_address + i < MAX_INPUT_REGISTERS) {
//            mapping->tab_input_registers[start_address + i] = values[i]; // Set multiple input registers
//        }
//    }
//}
//
//// Input Registers (3x) - 16-bit read-only
//void set_input_register(modbus_mapping_t *mb_mapping, int reg_address, uint16_t value) {
//    if (reg_address >= 0 && reg_address < mb_mapping->nb_input_registers) {
//        mb_mapping->tab_input_registers[reg_address] = value; // Set input register value
//    } else {
//        printf("Invalid input register address: %d\n", reg_address); // Error message for invalid address
//    }
//}
//
//uint16_t get_input_register(modbus_mapping_t *mb_mapping, int reg_address) {
//    if (reg_address >= 0 && reg_address < mb_mapping->nb_input_registers) {
//        return mb_mapping->tab_input_registers[reg_address]; // Return input register value
//    } else {
//        printf("Invalid input register address: %d\n", reg_address); // Error message for invalid address
//        return 0;
//    }
//}
//
//// Holding Registers (4x) - 16-bit read/write
//void set_holding_register(modbus_mapping_t *mb_mapping, int reg_address, uint16_t value) {
//    if (reg_address >= 0 && reg_address < mb_mapping->nb_registers) {
//        mb_mapping->tab_registers[reg_address] = value; // Set holding register value
//    } else {
//        printf("Invalid holding register address: %d\n", reg_address); // Error message for invalid address
//    }
//}
//
//uint16_t get_holding_register(modbus_mapping_t *mb_mapping, int reg_address) {
//    if (reg_address >= 0 && reg_address < mb_mapping->nb_registers) {
//        return mb_mapping->tab_registers[reg_address]; // Return holding register value
//    } else {
//        printf("Invalid holding register address: %d\n", reg_address); // Error message for invalid address
//        return 0;
//    }
//}
//
//void set_multiple_holding_registers(modbus_mapping_t *mb_mapping, int start_address, uint16_t *values, int count) {
//    if (start_address >= 0 && (start_address + count) <= mb_mapping->nb_registers) {
//        memcpy(&mb_mapping->tab_registers[start_address], values, count * sizeof(uint16_t)); // Set multiple holding registers
//    } else {
//        printf("Invalid holding register address range: %d to %d\n", start_address, start_address + count - 1); // Error message for invalid range
//    }
//}
