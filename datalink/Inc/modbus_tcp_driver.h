/*
 * modbus_tcp_driver.h
 *
 *  Created on: Apr 21, 2025
 *      Author: User
 */

#ifndef INC_MODBUS_TCP_DRIVER_H_
#define INC_MODBUS_TCP_DRIVER_H_

#include <stdint.h>

/**
 * @brief Initialize the Modbus TCP Server using W5500 stack.
 *
 * This function sets up the Modbus context and internal holding registers.
 * It initializes all W5500 sockets to listen for TCP connections on the given port.
 *
 * @param ip      The server IP address as a string (e.g., "192.168.1.100"). [Currently unused by libmodbus]
 * @param port    The TCP port to listen on (usually 502 for Modbus TCP)
 * @return int    0 on success, negative value on error
 */
int Modbus_TCP_Init(const char *ip, uint16_t port);

/**
 * @brief Main handler loop for Modbus TCP over W5500.
 *
 * Call this function periodically from your main loop.
 * It processes incoming W5500 events, manages sockets,
 * and executes Modbus requests.
 */
void Modbus_TCP_Handler_Loop(void);


#endif /* INC_MODBUS_TCP_DRIVER_H_ */
