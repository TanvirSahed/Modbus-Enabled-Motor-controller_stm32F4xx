/*
 * modbus_tcp_manager.h
 *
 *  Created on: Dec 11, 2024
 *      Author: Shakil Tanvir
 */

#ifndef INC_MODBUS_TCP_MANAGER_H_
#define INC_MODBUS_TCP_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>
#include <modbus.h>

//#define SPI2_Interrupt_Pin 0x01  // Replace with the actual pin definition

// Global Flags Structure
typedef struct {
    volatile bool connection_requested;
    volatile bool data_received;
    volatile bool connection_closed;
    volatile bool reset_required;

    uint8_t connection_requested_count;
    uint8_t data_received_count;
    uint8_t connection_closed_count;
    uint8_t reset_required_count;
} ModbusFlags;

// Global Modbus Flags
extern ModbusFlags modbus_flags;

// Function Prototypes
/**
 * Handles incoming Modbus requests and sends appropriate responses.
 *
 * @param ctx Pointer to the Modbus context.
 * @param mb_mapping Pointer to the Modbus mapping structure.
 */
void modbus_requests(modbus_t *ctx, modbus_mapping_t *mb_mapping);
/**
 * Accepts and establishes a new connection with a Modbus client.
 *
 * @param ctx Pointer to the Modbus context.
 * @param client_socket Pointer to the client socket.
 */
void connection_request(modbus_t *ctx, int *client_socket);
/**
 * Processes data received from the Modbus client and updates register values.
 *
 * @param ctx Pointer to the Modbus context.
 * @param mb_mapping Pointer to the Modbus mapping structure.
 */
void data_reception(modbus_t *ctx, modbus_mapping_t *mb_mapping);
/**
 * Handles cleanup and resource management after a client disconnection.
 */
void disconnection(modbus_t *ctx);
/**
 * Resets the Modbus server, reinitializing context and connection if needed.
 *
 * @param ctx Pointer to the Modbus context.
 */
void reset_server(modbus_t *ctx);
/**
 * Reads and clears the interrupt register for the specified W5500 socket.
 *
 * @param socket Socket number to read the interrupt register from.
 * @return Interrupt status register value.
 */
uint8_t W5500_ReadSocketInterruptRegister(uint8_t socket);
/**
 * GPIO interrupt callback function to handle events like connection, data reception, or disconnection.
 *
 * @param GPIO_Pin GPIO pin that triggered the interrupt.
 */
int run_modbus_server_blocking(const char *ip_address, int port);

int run_modbus_server_noneblocking(const char *ip_address, int port);
void run_modbus_tcp(void);
void handle_exti_Callback(uint16_t GPIO_Pin);

#endif /* INC_MODBUS_TCP_MANAGER_H_ */
