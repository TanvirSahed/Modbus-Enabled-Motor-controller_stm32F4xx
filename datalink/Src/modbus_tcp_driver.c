/*
 * modbus_tcp_driver.c
 *
 *  Created on: Apr 21, 2025
 *      Author: User
 */

#include "modbus_tcp_driver.h"

#include "tcp_handler.h"
#include "modbus.h"
#include "modbus-tcp.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "Registers.h"

#define HOLDING_REGISTERS 10
#define RECV_BUF_SIZE     32

static modbus_t *ctx = NULL;
static modbus_mapping_t *mb_mapping = NULL;
//static char server_ip[16];
//static uint16_t server_port;

extern uint8_t recv_buf[MAX_SOCK_NUM][RECV_BUF_SIZE];
extern W5500_EventFlags w5500_event_flags[MAX_SOCK_NUM];

int Modbus_TCP_Init(const char *ip, uint16_t port) {

    // Create a Modbus TCP context
    ctx = modbus_new_tcp(ip, port);
    if (ctx == NULL) {
        printf("Unable to allocate Modbus context: %s\n", modbus_strerror(errno));
        return 0;
    }

    modbus_set_debug(ctx, 1);

    // Create a new Modbus mapping

//    mb_mapping = modbus_mapping_new(10, 10, HOLDING_REGISTERS, 10);
    mb_mapping = modbus_reg_map();
    if (mb_mapping == NULL) {
        printf("Failed to allocate the Modbus mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 0;
    }

	for (int i = 0; i < HOLDING_REGISTERS; i++) {
		mb_mapping->tab_registers[i] = i * 10;
	}

    // Start listening for clients
    int server_socket = modbus_tcp_listen(ctx, 1);
    if (server_socket == -1) {
        printf("Unable to listen: %s\n", modbus_strerror(errno));
//        modbus_mapping_free(mb_mapping);
//        modbus_free(ctx);
        return 0;
    }
    printf("Modbus TCP server is running on %s:%d\n", ip, port);
	//printf("Modbus TCP initialized at %s:%d\n", server_ip, server_port);
	return 0;
}
static uint8_t rcv = 0;

void Modbus_TCP_Handler_Loop(void) {
    for (int sn = 0; sn < MAX_SOCK_NUM; sn++) {
    	if (w5500_event_flags[sn].received) {
    	    rcv++;
    	    int32_t len = handle_received(sn);
    	    printf("RECEIVED LENGTH: %ld \n", len);
    	    modbus_set_socket(ctx, sn);

    	    printf("Socket %d received %ld bytes:\n", sn, len);
    	    for (int i = 0; i < len; i++) {
    	        printf("%02X ", recv_buf[sn][i]);
    	    }
    	    printf("\n");

    	    if (len >= 6) {
    	        uint16_t modbus_len = (recv_buf[sn][4] << 8) | recv_buf[sn][5];
    	        uint16_t expected_len = 6 + modbus_len;

    	        if (len == expected_len) {
    	            printf("Valid Modbus TCP packet received.\n");

    	            uint8_t stat = getSn_SR(sn);
    	            printf("Before reply, socket %d status: %02X\n", sn, stat);




    				/*Valid slave ID checking*/
    				uint8_t slaveIdRcvd =  recv_buf[sn][6];
//    				uint8_t slaveIdLocal = modbus_get_slave(ctx);
    				printf("slaveIdRcvd: %d\r\n",slaveIdRcvd);
//    				if(slaveIdLocal == -1){
//    					return;
//    				}
//    				if(slaveIdRcvd != 255){
//    					return;
//    				}


    	            modbus_reply(ctx, recv_buf[sn], len, mb_mapping);

    	            stat = getSn_SR(sn);
    	            printf("After reply, socket %d status: %02X\n", sn, stat);

    	            if (stat == SOCK_CLOSE_WAIT) {
    	                printf("Closing socket %d as it's in CLOSE_WAIT\n", sn);
    	                closesock(sn);
    	            }
    	        } else {
    	            printf("Invalid packet length: expected %d, got %ld\n", expected_len, len);
    	        }
    	    } else {
    	        printf("Too short for Modbus TCP.\n");
    	    }

    	    memset(recv_buf[sn], 0, sizeof(recv_buf[sn]));
    	    memset(&w5500_event_flags[sn], 0, sizeof(W5500_EventFlags));
    	}

        if (w5500_event_flags[sn].connected) {
            handle_connection(sn);
            printf("Connection Requested on sock: %d\n", sn);
            int client_socket = modbus_tcp_accept(ctx, &sn);
            if (client_socket >= 0) {
                printf("Connection established on socket %d\n", client_socket);
            }
        }

        if (w5500_event_flags[sn].disconnected) {
            handle_disconnection(sn);
            printf("Disconnected sock: %d\n", sn);
        }

        if (w5500_event_flags[sn].timeout) {
            handle_timeout(sn);
        }

        if (w5500_event_flags[sn].sent) {
            handle_sent(sn);
        }
    }
}
