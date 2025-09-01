/*
 * modbus_uart_manager.c
 *
 *  Created on: Dec 9, 2024
 *      Author: Qbits
 */

#include "modbus-rtu.h"
#include "modbus_uart_manager.h"
#include "Registers.h"
#include "debug.h"


// Array of Modbus UART con
UARTModbusConfig uart_contexts[MAX_UARTS];


// Function to initialize Modbus RTU on a UART
int Initialize_ModbusRTU(UART_HandleTypeDef *huart, SerialCommParams *serial,
								GPIO_TypeDef *ctrl_port, uint16_t ctrl_pin,
								int slave_id, int uart_index) {

    if (uart_index < 0 || uart_index >= MAX_UARTS) {
        dbg_print("Modbus RTU: Invalid UART index: %d\n", uart_index);
        return -1; // Invalid index
    }
    uart_contexts[uart_index].uart = huart;
    // Create Modbus RTU context
    uart_contexts[uart_index].modbus_context = modbus_new_rtu(huart, serial->baudRate, (char)serial->parity,
    												      serial->dataBit, serial->stopBit);

    if (uart_contexts[uart_index].modbus_context == NULL) {
    	dbg_print("Modbus RTU: Failed to create Modbus context for UART %d\n", uart_index);
        return -1;
    }

    // Set slave ID
    if (modbus_set_slave(uart_contexts[uart_index].modbus_context, slave_id) == -1) {
    	dbg_print("Modbus RTU: Failed to set slave ID for UART %d\n", uart_index);
        return -1;
    }

    // Allocate Modbus mapping
    uart_contexts[uart_index].modbus_mapping = modbus_reg_map();
    if (uart_contexts[uart_index].modbus_mapping == NULL) {
    	dbg_print("Modbus RTU: Failed to allocate Modbus mapping for UART %d\n", uart_index);
        return -1;
    }

    // Initialize UART_CTRL GPIO
    uart_contexts[uart_index].ctrl_port = ctrl_port;
    uart_contexts[uart_index].ctrl_pin = ctrl_pin;
    HAL_GPIO_WritePin(ctrl_port, ctrl_pin, GPIO_PIN_SET); // Set to receiving mode by default


    // Initialize UART receive based on configuration
   #if DMA
       if (HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_contexts[uart_index].rx_buffer, sizeof(uart_contexts[uart_index].rx_buffer)) != HAL_OK) {
           dbg_print("Modbus RTU: Failed to initialize UART DMA for UART %d\n", uart_index);
           return -1;
       }
   #elif INTERRUPT
       if (HAL_UARTEx_ReceiveToIdle_IT(huart, uart_contexts[uart_index].rx_buffer, sizeof(uart_contexts[uart_index].rx_buffer)) != HAL_OK) {
           dbg_print("Failed to initialize UART interrupt for UART %d\n", uart_index);
           return -1;
       }
   #else
       // Default to DMA if no flag is defined
       if (HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_contexts[uart_index].rx_buffer, sizeof(uart_contexts[uart_index].rx_buffer)) != HAL_OK) {
           dbg_print("Failed to initialize UART DMA for UART %d\n", uart_index);
           return -1;
       }
   #endif



    // Assign default Modbus data for this UART
//    Assign_Modbus_Data(uart_contexts[uart_index].modbus_mapping, uart_index);

//    // Assign fixed data to coils (tab_bits)
//    for (int i = 0; i < 10; i++) {
//        uart_contexts[uart_index].modbus_mapping->tab_bits[i] = (i % 2); // Alternating 1, 0
//    }
//
//    // Assign fixed data to discrete inputs (tab_input_bits)
//    for (int i = 0; i < 10; i++) {
//        uart_contexts[uart_index].modbus_mapping->tab_input_bits[i] = 1; // All set to 1
//    }
//
//    // Assign fixed data to input registers (tab_input_registers)
//    for (int i = 0; i < 100; i++) {
//        uart_contexts[uart_index].modbus_mapping->tab_input_registers[i] = i * 10; // Multiples of 10
//    }
//
//    // Assign fixed data to holding registers (tab_registers)
//    for (int i = 0; i < 100; i++) {
//        uart_contexts[uart_index].modbus_mapping->tab_registers[i] = i + 100; // Values 100, 101, 102...
//    }

       dbg_print("Modbus RTU: Init Success!\r\n");
    return 0;
}

// Function to deinitialize Modbus RTU on a UART
void Deinitialize_ModbusRTU(int uart_index) {
    if (uart_index < 0 || uart_index >= MAX_UARTS) {
        dbg_print("Invalid UART index: %d\n", uart_index);
        return;
    }

    if (uart_contexts[uart_index].modbus_mapping) {
        modbus_mapping_free(uart_contexts[uart_index].modbus_mapping);
        uart_contexts[uart_index].modbus_mapping = NULL;
    }

    if (uart_contexts[uart_index].modbus_context) {
        modbus_free(uart_contexts[uart_index].modbus_context);
        uart_contexts[uart_index].modbus_context = NULL;
    }
}


void Assign_Modbus_Data(modbus_mapping_t *mapping, int uart_index) {
    if (mapping == NULL) {
        dbg_print("Error: Null Modbus mapping for UART %d\n", uart_index);
        return;
    }

    switch (uart_index) {
        case 0:
            for (int i = 0; i < 10; i++) {
                mapping->tab_bits[i] = 1; // All coils ON
                mapping->tab_input_bits[i] = 0; // All discrete inputs OFF
            }
            for (int i = 0; i < 100; i++) {
                mapping->tab_input_registers[i] = i; // Linear values
                mapping->tab_registers[i] = i + 100; // Offset by 100
            }
            break;

        case 1:
            for (int i = 0; i < 10; i++) {
                mapping->tab_bits[i] = (i % 2); // Alternating coils
                mapping->tab_input_bits[i] = 1; // All discrete inputs ON
            }
            for (int i = 0; i < 100; i++) {
                mapping->tab_input_registers[i] = i * 10; // Multiples of 10
                mapping->tab_registers[i] = 1000 + i; // Offset by 1000
            }
            break;

        default:
            for (int i = 0; i < 10; i++) {
                mapping->tab_bits[i] = 0; // All coils OFF
                mapping->tab_input_bits[i] = 1; // All discrete inputs ON
            }
            for (int i = 0; i < 100; i++) {
                mapping->tab_input_registers[i] = 500 + i; // Starting at 500
                mapping->tab_registers[i] = 2000 + i; // Offset by 2000
            }
            break;
    }
}

//// Function to handle Modbus requests
//void Handle_ModbusRequest(int uart_index) {
//    if (uart_index < 0 || uart_index >= MAX_UARTS) {
//        dbg_print("Invalid UART index: %d\n", uart_index);
//        return;
//    }
//
//    UARTModbusConfig *config = &uart_contexts[uart_index];
//
//    // Set UART_CTRL to receiving mode
//    HAL_GPIO_WritePin(config->ctrl_port, config->ctrl_pin, GPIO_PIN_SET);
//    int rc = 1;
////    int rc = modbus_receive(config->modbus_context, config->rx_buffer);
////    if (rc > 0) {
//        // Set UART_CTRL to transmission mode
//        HAL_GPIO_WritePin(config->ctrl_port, config->ctrl_pin, GPIO_PIN_RESET);
//
//        modbus_reply(config->modbus_context, config->rx_buffer, rc, config->modbus_mapping);
//
//        // Return to receiving mode
//        HAL_GPIO_WritePin(config->ctrl_port, config->ctrl_pin, GPIO_PIN_SET);
////    } else {
////        dbg_print("Error processing Modbus request on UART %d\n", uart_index);
////    }
//
//    memset(config->rx_buffer, 0, sizeof(config->rx_buffer));
//}

// Function to print a single UARTModbusConfig struct
void printUARTModbusConfig(const UARTModbusConfig *config) {
    dbg_print("UART Handle: %p\n", (void *)config->uart);
    dbg_print("Modbus Context: %p\n", (void *)config->modbus_context);
    dbg_print("Modbus Mapping: %p\n", (void *)config->modbus_mapping);
    dbg_print("Control Port: %p\n", (void *)config->ctrl_port);
    dbg_print("Control Pin: 0x%X\n", config->ctrl_pin);

    dbg_print("RX Buffer: ");
    for (size_t i = 0; i < sizeof(config->rx_buffer); ++i) {
        dbg_print("0x%02X ", config->rx_buffer[i]);
        if ((i + 1) % 16 == 0) { // Print 16 bytes per line for readability
            dbg_print("\n");
        }
    }
    dbg_print("\n");
}

// Function to print all Modbus UART contexts
void printAllUARTContexts(const UARTModbusConfig contexts[], size_t count) {
    for (size_t i = 0; i < count; ++i) {
        dbg_print("UART Context %zu:\n", i + 1);
        printUARTModbusConfig(&contexts[i]);
        dbg_print("-------------------------\n");
    }
}

/**
 * @brief Reconfigures Modbus serial parameters with new settings (STM32 optimized)
 * @param uart_index Index of the UART to reconfigure
 * @param new_serial Pointer to new serial parameters
 * @return 0 on success, -1 on failure
 */
int Reconfigure_ModbusSerialParams(int uart_index, const SerialCommParams *new_serial) {
    if (uart_index < 0 || uart_index >= MAX_UARTS) {
        dbg_print("Invalid UART index: %d\n", uart_index);
        return -1;
    }

    if (new_serial == NULL) {
        dbg_print("Null serial parameters pointer\n");
        return -1;
    }

    UARTModbusConfig *config = &uart_contexts[uart_index];
    if (config->modbus_context == NULL || config->uart == NULL) {
        dbg_print("Modbus context or UART not initialized for UART %d\n", uart_index);
        return -1;
    }

    // Validate parameters more strictly for STM32
    if (new_serial->baudRate == 0 ||
        (new_serial->parity != 'N' && new_serial->parity != 'E' && new_serial->parity != 'O') ||
        new_serial->dataBit < 7 || new_serial->dataBit > 9 ||  // STM32 typically supports 7-9 bits
        new_serial->stopBit < 1 || new_serial->stopBit > 2) {
        dbg_print("Invalid serial parameters for STM32: %d,%c,%d,%d\n",
                 new_serial->baudRate, new_serial->parity,
                 new_serial->dataBit, new_serial->stopBit);
        return -1;
    }

    // Lock the UART context (if using RTOS)
    // osMutexAcquire(uart_context_mutex, osWaitForever);

    // Close the current Modbus context
    modbus_close(config->modbus_context);

    /* Create a completely new Modbus context with the new parameters */
    modbus_t *new_ctx = modbus_new_rtu(config->uart,
                                      new_serial->baudRate,
                                      new_serial->parity,
                                      new_serial->dataBit,
                                      new_serial->stopBit);

    if (new_ctx == NULL) {
        dbg_print("Failed to create new Modbus context with updated parameters\n");
        // Try to restore previous configuration
        modbus_connect(config->modbus_context);
        // osMutexRelease(uart_context_mutex);
        return -1;
    }

    // Copy over important settings from old context
    int slave_id = modbus_get_slave(config->modbus_context);
    modbus_set_slave(new_ctx, slave_id);

    // Set the same response timeout (using correct function signature)
    uint32_t to_sec, to_usec;
    modbus_get_response_timeout(config->modbus_context, &to_sec, &to_usec);
    modbus_set_response_timeout(new_ctx, to_sec, to_usec);

    // Free the old context and replace with new one
    modbus_free(config->modbus_context);
    config->modbus_context = new_ctx;

    /* STM32-specific UART reconfiguration */
    // First disable the UART
    HAL_UART_DeInit(config->uart);

    // Update UART hardware configuration
    config->uart->Init.BaudRate = new_serial->baudRate;

    // Handle parity (STM32 uses hardware parity bits)
    switch (new_serial->parity) {
        case 'N':
            config->uart->Init.Parity = UART_PARITY_NONE;
            break;
        case 'E':
            config->uart->Init.Parity = UART_PARITY_EVEN;
            break;
        case 'O':
            config->uart->Init.Parity = UART_PARITY_ODD;
            break;
    }

    // Handle word length (STM32 specific)
    if (new_serial->parity == 'N') {
        // No parity - can use 8 or 9 bits
        config->uart->Init.WordLength = (new_serial->dataBit == 9) ? UART_WORDLENGTH_9B :
                                                                     UART_WORDLENGTH_8B;
    } else {
        // With parity - STM32 uses one bit for parity, so:
        // 7 data bits + parity = 8 bits total
        // 8 data bits + parity = 9 bits total
        config->uart->Init.WordLength = (new_serial->dataBit == 8) ? UART_WORDLENGTH_9B :
                                                                     UART_WORDLENGTH_8B;
    }

    // Handle stop bits
    config->uart->Init.StopBits = (new_serial->stopBit == 1) ? UART_STOPBITS_1 :
                                                               UART_STOPBITS_2;

    // Reinitialize UART
    if (HAL_UART_Init(config->uart) != HAL_OK) {
        dbg_print("Failed to reinitialize UART hardware\n");
        // osMutexRelease(uart_context_mutex);
        return -1;
    }

    // Restart reception
#if DMA
    if (HAL_UARTEx_ReceiveToIdle_DMA(config->uart, config->rx_buffer, sizeof(config->rx_buffer)) != HAL_OK) {
        dbg_print("Failed to restart UART DMA\n");
        // osMutexRelease(uart_context_mutex);
        return -1;
    }
#elif INTERRUPT
    if (HAL_UARTEx_ReceiveToIdle_IT(config->uart, config->rx_buffer, sizeof(config->rx_buffer)) != HAL_OK) {
        dbg_print("Failed to restart UART interrupt\n");
        // osMutexRelease(uart_context_mutex);
        return -1;
    }
#endif

    // Reconnect the Modbus context
    if (modbus_connect(config->modbus_context) == -1) {
        dbg_print("Failed to reconnect after reconfiguration\n");
        // osMutexRelease(uart_context_mutex);
        return -1;
    }

    // Unlock the UART context
    // osMutexRelease(uart_context_mutex);

    dbg_print("Successfully reconfigured UART %d to: %d baud, %c parity, %d data bits, %d stop bits\n",
             uart_index, new_serial->baudRate, new_serial->parity,
             new_serial->dataBit, new_serial->stopBit);

    return 0;
}
