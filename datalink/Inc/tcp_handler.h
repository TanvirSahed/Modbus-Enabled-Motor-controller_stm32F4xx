/*
 * tcp_handler.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Shakil Tanvir
 */

#ifndef INC_TCP_HANDLER_H_
#define INC_TCP_HANDLER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "socket.h"
#include "w5500.h"
#include "w5500_spi_handler.h"

#define MAX_SOCK_NUM 8
#define SERVER_PORT 8080

typedef struct {
    uint8_t socket;
    uint8_t connected;
    uint8_t disconnected;
    uint8_t received;
    uint8_t timeout;
    uint8_t sent;
} W5500_EventFlags;

extern W5500_EventFlags w5500_event_flags[];
extern int32_t len;

//void W5500_Enable_Interrupts(void);
void W5500_Handle_Events(void);
void W5500_Init_Sockets(void);
void W5500_InterruptHandler(void);
void handle_connection(uint8_t sn);
void handle_disconnection(uint8_t sn);
int32_t handle_received(uint8_t sn);
void handle_timeout(uint8_t sn);
void handle_sent(uint8_t sn);
void process_received_command(uint8_t sn, const char *buffer);
int8_t SendToSocket(uint8_t sn, const char *msg);

#endif /* INC_TCP_HANDLER_H_ */
