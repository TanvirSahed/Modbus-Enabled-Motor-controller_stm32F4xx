/*
 * modbus_tcp_manager.c
 *
 *  Created on: Dec 11, 2024
 *      Author: Shakil Tanvir
 */


#include "modbus_tcp_manager.h"
#include "tcp_handler.h"
//#include "server.h"
#include <errno.h>
#include "w5500_spi_handler.h"

#define COILS 10                // Number of coil registers
#define DISCRETE_INPUTS 10       // Number of discrete input registers
#define HOLDING_REGISTERS 10     // Number of holding registers
#define INPUT_REGISTERS 10       // Number of input registers
ModbusFlags modbus_flags;
modbus_t *cntx;
modbus_mapping_t *mb_map;

// Main Modbus Server Function
int run_modbus_server_noneblocking(const char *ip_address, int port) {
    // Create a Modbus TCP context
    modbus_t *ctx = modbus_new_tcp(ip_address, port);
    if (ctx == NULL) {
        printf("Unable to allocate Modbus context: %s\n", modbus_strerror(errno));
        return 0;
    }

    cntx = ctx;
    // Create a new Modbus mapping
    modbus_mapping_t *mb_mapping = modbus_mapping_new(0, 0, HOLDING_REGISTERS, 0);
    if (mb_mapping == NULL) {
        printf("Failed to allocate the Modbus mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 0;
    }
    mb_map = mb_mapping;
    // Initialize holding registers
    for (int i = 0; i < HOLDING_REGISTERS; i++) {
        mb_mapping->tab_registers[i] = i * 10;
    }

    printf("Modbus TCP server is running on %s:%d\n", ip_address, port);

    // Start listening for clients
    int server_socket = modbus_tcp_listen(ctx, 1);
    if (server_socket == -1) {
        printf("Unable to listen: %s\n", modbus_strerror(errno));
        modbus_mapping_free(mb_mapping);
        modbus_free(ctx);
        return 0;
    }

    int client_socket;
//    while (1) {
//        if (modbus_flags.connection_requested) {
//            connection_request(ctx, &client_socket);
//        }
//
//        if (modbus_flags.data_received) {
//            data_reception(ctx, mb_mapping);
//        }
//
//        if (modbus_flags.connection_closed) {
//        	disconnection(ctx);
//        }
//
//        if (modbus_flags.reset_required) {
//            reset_server(ctx);
//        }
//    }

//    // Free resources
//    modbus_mapping_free(mb_mapping);
//    modbus_free(ctx);
//    closesock(server_socket);
    return 1;
}

void run_modbus_tcp(void){
    int client_socket;
//    while (1) {
        if (modbus_flags.connection_requested) {
            connection_request(cntx, &client_socket);
        }

        if (modbus_flags.data_received) {
            data_reception(cntx, mb_map);
        }

        if (modbus_flags.connection_closed) {
        	disconnection(cntx);
        }

        if (modbus_flags.reset_required) {
            reset_server(cntx);
        }
//    }

//    // Free resources
//    modbus_mapping_free(mb_mapping);
//    modbus_free(ctx);
//    closesock(server_socket);
}

// Handles incoming client connections
void connection_request(modbus_t *ctx, int *client_socket) {
    modbus_flags.connection_requested = false;
    *client_socket = modbus_tcp_accept(ctx, SOCK_0);
    if (*client_socket >= 0) {
        printf("New connection established on socket %d\n", *client_socket);
    }
}

// Processes Modbus requests from clients
void data_reception(modbus_t *ctx, modbus_mapping_t *mb_mapping) {
    modbus_flags.data_received = false;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    int rc = modbus_receive(ctx, query);
    if (rc > 0) {
        modbus_reply(ctx, query, rc, mb_mapping);

        // Display register values
        printf("Current register values:\n");
        for (int i = 0; i < HOLDING_REGISTERS; i++) {
            printf("Register[%d] = %d\n", i, mb_mapping->tab_registers[i]);
        }
    } else if (rc == -1) {
        printf("Client disconnected or error occurred.\n");
    }
}

// Handles client disconnection
void disconnection(modbus_t *ctx) {
    modbus_flags.connection_closed = false;
    closesock(SOCK_0);
    printf("Connection closed.\n");
    modbus_tcp_listen(ctx, 1);

}

// Resets the server in case of errors
void reset_server(modbus_t *ctx) {
    modbus_flags.reset_required = false;
    W5500_Reset();
    modbus_tcp_listen(ctx, 1);
}

// Reads the interrupt register for the specified socket
uint8_t W5500_ReadSocketInterruptRegister(uint8_t socket) {
    uint8_t interrupt_status = getSn_IR(socket);
    setSn_IR(socket, interrupt_status);
    return interrupt_status;
}



int run_modbus_server_blocking(const char *ip_address, int port) {
	// Create a Modbus TCP context
		    modbus_t *ctx = modbus_new_tcp(ip_address, port);
		    if (ctx == NULL) {
		       // fprintf(stderr, "Unable to allocate Modbus context: %s\n", modbus_strerror(errno));
		    	printf("Unable to allocate Modbus context: %s\n", modbus_strerror(errno));
		        return 0;
		    }

		    // Enable debug mode
		    modbus_set_debug(ctx, TRUE);

		    // Create a new Modbus mapping with all data types allocated
		      modbus_mapping_t *mb_mapping = modbus_mapping_new(COILS, DISCRETE_INPUTS, HOLDING_REGISTERS, INPUT_REGISTERS);
		      if (mb_mapping == NULL) {
		          printf("Failed to allocate the Modbus mapping: %s\n", modbus_strerror(errno));
		          modbus_free(ctx);
		          return 0;
		      }

		      // Initialize Coil values (1 = ON, 0 = OFF)
		      for (int i = 0; i < COILS; i++) {
		          mb_mapping->tab_bits[i] = i % 2;  // Alternating 0 and 1
		      }

		      // Initialize Discrete Inputs (Read-Only, typically sensors)
		      for (int i = 0; i < DISCRETE_INPUTS; i++) {
		          mb_mapping->tab_input_bits[i] = (i % 2 == 0) ? 1 : 0;  // Even indexed inputs ON
		      }

		      // Initialize Holding Registers (Read/Write registers)
		      for (int i = 0; i < HOLDING_REGISTERS; i++) {
		          mb_mapping->tab_registers[i] = i * 10;  // Multiples of 10
		      }

		      // Initialize Input Registers (Read-Only registers)
		      for (int i = 0; i < INPUT_REGISTERS; i++) {
		          mb_mapping->tab_input_registers[i] = i * 5;  // Multiples of 5
		      }
		    printf("Modbus TCP server is running on %s:%d\n", ip_address, port);

		    // Start listening for clients
		    int server_socket = modbus_tcp_listen(ctx, 1); // 1 = max number of clients
		    if (server_socket == -1) {
		       printf("Unable to listen: %s\n", modbus_strerror(errno));
		        modbus_mapping_free(mb_mapping);
		        modbus_free(ctx);
		        return 0;
		    }

		    for (;;) {
		      printf("Waiting for a client to connect...\n");

		        // Accept a client connection
		        int client_socket = modbus_tcp_accept(ctx, SOCK_0);
		        if (client_socket == -1) {
		          printf("Accept failed: %s\n", modbus_strerror(errno));
		            continue;
		        }

		        printf("Client connected.\n");

		        // Process client requests
		        for (;;) {
		            uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

		            //int rx_size = getSn_RX_RSR(SOCK_0);

		            int rc = modbus_receive(ctx, query);
		            //rec=rc;
		            if (rc > 0) {
		                // Handle the received request
		                modbus_reply(ctx, query, rc, mb_mapping);

		                // Display register values after each client operation
		               printf("Current register values:\n");
		                for (int i = 0; i < HOLDING_REGISTERS; i++) {
		                 printf("Register[%d] = %d\n", i, mb_mapping->tab_registers[i]);
		                }
		            } else if (rc == -1) {
		                // Error or disconnection
		             printf("Client disconnected or error occurred.\n");
		                break;
		            }
		        }

		        // Close the client socket
		        closesock(client_socket);
		    }

		    // Free resources
		    modbus_mapping_free(mb_mapping);
		    modbus_free(ctx);
		    closesock(server_socket);
}

// GPIO interrupt callback
void handle_exti_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == W5500_IT5_Pin) {
//        uint8_t ir = W5500_ReadSocketInterruptRegister(SOCK_0);
//
//        if (ir & Sn_IR_CON) {
//            modbus_flags.connection_requested = true;
//        }
//
//        if (ir & Sn_IR_RECV) {
//            modbus_flags.data_received = true;
//        }
//
//        if (ir & Sn_IR_DISCON) {
//            modbus_flags.connection_closed = true;
//        }
    	W5500_InterruptHandler();

    }
}
