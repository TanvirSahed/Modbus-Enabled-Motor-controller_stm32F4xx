/*
 * w5500_spi_handler.h
 *
 *  Created on: Dec 12, 2024
 *      Author: Qbits
 */

#ifndef INC_W5500_SPI_HANDLER_H_
#define INC_W5500_SPI_HANDLER_H_

#include "main.h"
#include "wizchip_conf.h"
#include "stm32f4xx_hal.h"
#include "w5500.h"
#include "socket.h"

#define SOCK_0	0
#define NETWORK_MSG  		 "Network configuration:\r\n"
#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"
#define CONN_ESTABLISHED_MSG "\r\nConnection established with remote IP: %d.%d.%d.%d:%d\r\n"
#define WRONG_RETVAL_MSG	 "Something went wrong; return value: %d\r\n"
#define WRONG_STATUS_MSG	 "Something went wrong; STATUS: %d\r\n"
#define LISTEN_ERR_MSG		 "LISTEN Error!\r\n"
// Structure to hold GPIO configuration for W5500 and SPI
typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *int_port;
    uint16_t int_pin;
    SPI_HandleTypeDef *spi_handle;
} W5500_GPIO_Config_t;

//exported
extern uint8_t remoteIP[4];
extern uint16_t remotePort;


// Function Prototypes
//void W5500Init(W5500_GPIO_Config_t *gpio_config);
void W5500Init(W5500_GPIO_Config_t *gpio_config, wiz_NetInfo *net_info);
void W5500_Reset(void);
void W5500_Enable_Interrupts(void);
void W5500_Update_NetInfo(wiz_NetInfo *net_info); // added later by TS
void wizchip_select(void);
void wizchip_deselect(void);
uint8_t wizchip_read(void);
void wizchip_write(uint8_t wb);
void wizchip_readburst(uint8_t *pBuf, uint16_t len);
void wizchip_writeburst(uint8_t *pBuf, uint16_t len);
void wizchip_write_register(uint16_t address, uint8_t value);
void check_phy_status(void);
void retrieve_peer_details(void);

#endif /* INC_W5500_SPI_HANDLER_H_ */
