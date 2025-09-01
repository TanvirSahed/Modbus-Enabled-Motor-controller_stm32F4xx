/*
 * tcp_handler.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Shakil Tanvir
 */
#include "tcp_handler.h"
#include "wizchip_conf.h"
#include "socket.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

W5500_EventFlags w5500_event_flags[MAX_SOCK_NUM] = {0};
uint8_t buf[32];

uint8_t recv_buf[MAX_SOCK_NUM][32] = {0};
int32_t len;
uint8_t client_ip[MAX_SOCK_NUM][4] = {0};

uint8_t sn;


void W5500_Handle_Events(void)
{
    for (uint8_t sn = 0; sn < MAX_SOCK_NUM; sn++) {
        if (w5500_event_flags[sn].connected)
            handle_connection(sn);
        if (w5500_event_flags[sn].disconnected)
            handle_disconnection(sn);
        if (w5500_event_flags[sn].received)
            handle_received(sn);
        if (w5500_event_flags[sn].timeout)
            handle_timeout(sn);
        if (w5500_event_flags[sn].sent)
            handle_sent(sn);
    }
    // Clear all flags at once
    memset(w5500_event_flags, 0, sizeof(w5500_event_flags));
}

void W5500_Init_Sockets(void) {
    for (uint8_t sn = 0; sn < MAX_SOCK_NUM; sn++) {
        if (socket(sn, Sn_MR_TCP, SERVER_PORT, 0) == sn) {
            listen(sn);
        }
    }
}


void handle_connection(uint8_t sn) {
    // Wait for established state before reading DIPR
    if (getSn_SR(sn) == SOCK_ESTABLISHED) {
        getSn_DIPR(sn, client_ip[sn]);
        printf("Socket %d connected from %d.%d.%d.%d", sn,
               client_ip[sn][0], client_ip[sn][1], client_ip[sn][2], client_ip[sn][3]);
    } else {
        memset(client_ip[sn], 0, 4);
        printf("Socket %d connection event, but not established ", sn);
    }

    memset(&w5500_event_flags[sn], 0, sizeof(W5500_EventFlags));
}


void handle_disconnection(uint8_t sn) {
    printf("Socket %d disconnected ", sn);
    memset(client_ip[sn], 0, 4); // Reset client IP on disconnection
    disconnect(sn);
    closesock(sn);
    socket(sn, Sn_MR_TCP, SERVER_PORT, 0);
    listen(sn);

    memset(&w5500_event_flags[sn], 0, sizeof(W5500_EventFlags));
}

//int32_t handle_received(uint8_t sn) {
//	int32_t length;
//    int32_t size = getSn_RX_RSR(sn);
//    if (size > 0 && size < sizeof(recv_buf[sn])) {
////    	length = recv(sn, recv_buf[sn], size);
//
//    	while ((size = getSn_RX_RSR(sn)) > 0) {
//    		length = recv(sn, recv_buf[sn], size);
//    	}
//
//        if (length > 0) {
//            recv_buf[sn][length] = '\0';
//
//            //process_received_command
//        }
//    }
//    return length;
//    memset(&w5500_event_flags[sn], 0, sizeof(W5500_EventFlags));
//}
int32_t handle_received(uint8_t sn) {
    int32_t total = 0;
    int32_t lenthh = 0;

    while ((lenthh = getSn_RX_RSR(sn)) > 0 && total < sizeof(recv_buf[sn])) {
        int32_t received = recv(sn, recv_buf[sn] + total, lenthh);
        if (received > 0) total += received;
        else break;
    }

    return total;
}

void handle_timeout(uint8_t sn) {
    printf("Timeout on socket %d\n", sn);
    disconnect(sn);
    closesock(sn);
    socket(sn, Sn_MR_TCP, SERVER_PORT, 0);
    listen(sn);

    memset(&w5500_event_flags[sn], 0, sizeof(W5500_EventFlags));
}

void handle_sent(uint8_t sn) {
    printf("Data sent on socket %d\n", sn);
    memset(&w5500_event_flags[sn], 0, sizeof(W5500_EventFlags));
}

void process_received_command(uint8_t sn, const char *buffer)
{
    if (strncmp(buffer, "Hello", 5) == 0) {
        const char *msg = "Received\r\n";
        send(sn, (uint8_t *)msg, strlen(msg));
    }
}

void W5500_InterruptHandler(void) {
	for (uint8_t sn = 0; sn < MAX_SOCK_NUM; sn++) {
		uint8_t ir = getSn_IR(sn);
		if (ir) {
			setSn_IR(sn, ir);  // Clear all flags that are set

			w5500_event_flags[sn].socket = sn;
			w5500_event_flags[sn].connected = (ir & Sn_IR_CON) ? 1 : 0;
			w5500_event_flags[sn].disconnected = (ir & Sn_IR_DISCON) ? 1 : 0;
			w5500_event_flags[sn].received = (ir & Sn_IR_RECV) ? 1 : 0;
			w5500_event_flags[sn].timeout = (ir & Sn_IR_TIMEOUT) ? 1 : 0;
			w5500_event_flags[sn].sent = (ir & Sn_IR_SENDOK) ? 1 : 0;
		}
	}
}


#define MAX_SEND_RETRIES 10

int8_t SendToSocket(uint8_t sn, const char *msg)
{
    if (getSn_SR(sn) == SOCK_ESTABLISHED) {
        int retry = 0;
        int32_t result;

        while (retry < MAX_SEND_RETRIES) {
            result = send(sn, (uint8_t *)msg, strlen(msg));
            if (result > 0) {
                printf("Sent on socket %d: %s\n", sn, msg);
                return 0; // Success
            } else {
                printf("Retry %d failed to send on socket %d || result: %ld\n", retry + 1, sn, result);
                retry++;
            }
        }

        printf("All retries failed on socket %d\n", sn);
        return -2; // All retries failed
    } else {
        printf("Socket %d not connected\n", sn);
        return -1; // Socket not established
    }
}


//// GPIO interrupt callback
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    if (GPIO_Pin == W5500_INT_Pin) {
//    	W5500_InterruptHandler();
//    }
//}
