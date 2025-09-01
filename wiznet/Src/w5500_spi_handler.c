/*
 * w5500_spi_handler.c
 *
 *  Created on: Dec 12, 2024
 *      Author: Qbits
 */

#include "w5500_spi_handler.h"
#include "debug.h"

#define DATA_BUF_SIZE 128 //
static W5500_GPIO_Config_t w5500_gpio_config;

wiz_NetInfo netInfo = { .mac = { 0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef }, .ip = {
		192, 168, 0, 14 }, .sn = { 255, 255, 255, 0 }, .gw = { 192, 168, 0, 1 },
		.dns = { 8, 8, 8, 8 }, .dhcp = NETINFO_STATIC };

uint8_t remoteIP[4];
uint16_t remotePort;
char msg[DATA_BUF_SIZE];  // Buffer for messages

// MCU-Specific Critical Section Handling
void mcu_cris_enter(void) {
    __set_PRIMASK(1);
}

void mcu_cris_exit(void) {
    __set_PRIMASK(0);
}

// W5500 Chip Select and Deselect
void wizchip_select(void) {
    HAL_GPIO_WritePin(w5500_gpio_config.cs_port, w5500_gpio_config.cs_pin, GPIO_PIN_RESET);
}

void wizchip_deselect(void) {
    HAL_GPIO_WritePin(w5500_gpio_config.cs_port, w5500_gpio_config.cs_pin, GPIO_PIN_SET);
}

// SPI Communication Functions
uint8_t wizchip_read(void) {
    uint8_t rbuf;
    HAL_SPI_Receive(w5500_gpio_config.spi_handle, &rbuf, 1, 100);
    return rbuf;
}

void wizchip_write(uint8_t wb) {
    HAL_SPI_Transmit(w5500_gpio_config.spi_handle, &wb, 1, 100);
}

void wizchip_readburst(uint8_t *pBuf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        *pBuf++ = wizchip_read();
    }
}

void wizchip_writeburst(uint8_t *pBuf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        wizchip_write(*pBuf++);
    }
}
void check_phy_status(void) {
    if (!(getPHYCFGR() & PHYCFGR_LNK_ON)) {
        printf("\n\rPHY GONE!\n\r");
//        W5500Init();
        //state = STATE_SOCKET_INIT;
    }
}

void retrieve_peer_details(void) {
    getsockopt(SOCK_0, SO_DESTIP, remoteIP);
    getsockopt(SOCK_0, SO_DESTPORT, (uint8_t*) &remotePort);

    // Connection established message
    sprintf(msg, CONN_ESTABLISHED_MSG, remoteIP[0], remoteIP[1], remoteIP[2], remoteIP[3], remotePort);
//    print_string(msg);
}
// Interrupt Configuration
void W5500_Enable_Interrupts(void) {
    setIMR(IM_IR7 | IM_IR6 | IM_IR5 | IM_IR4); // Global interrupts
    setSIMR(0xFF); // Enable socket interrupts for all sockets
    setSn_IMR(SOCK_0, Sn_IR_CON | Sn_IR_DISCON | Sn_IR_RECV | Sn_IR_TIMEOUT);
}

//// W5500 Initialization
//void W5500Init(W5500_GPIO_Config_t *gpio_config) {
//    uint8_t tmp;
//    uint8_t memsize[2][8] = {{4, 2, 2, 2, 2, 2, 1, 1}, {4, 2, 2, 2, 2, 2, 1, 1}};
//
//    // Save GPIO configuration
//    w5500_gpio_config = *gpio_config;
//
//    // Set CS pin high and reset W5500
//    HAL_GPIO_WritePin(w5500_gpio_config.cs_port, w5500_gpio_config.cs_pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(w5500_gpio_config.reset_port, w5500_gpio_config.reset_pin, GPIO_PIN_RESET);
//    tmp = 0xFF;
//    while (tmp--);
//    HAL_GPIO_WritePin(w5500_gpio_config.reset_port, w5500_gpio_config.reset_pin, GPIO_PIN_SET);
//
//    // Register MCU-specific callbacks
//    reg_wizchip_cris_cbfunc(mcu_cris_enter, mcu_cris_exit);
//    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
//    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
//    reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);
//
//    // Initialize W5500
//    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1) {
//        printf("Wizchip failed to initialize.\r\n");
//        while (1);
//    }
//
//    W5500_Enable_Interrupts();
//    printf("Wizchip initialized successfully.\r\n");
//
//    wizchip_setnetinfo(&netInfo);
//    wizchip_getnetinfo(&netInfo);
////    print_netinfo(&netInfo);
//}

// W5500 Initialization
void W5500Init(W5500_GPIO_Config_t *gpio_config, wiz_NetInfo *net_info) {
    uint8_t tmp;
    uint8_t memsize[2][8] = {{4, 2, 2, 2, 2, 2, 1, 1}, {4, 2, 2, 2, 2, 2, 1, 1}};

    // Save GPIO configuration
    w5500_gpio_config = *gpio_config;

    // Set CS pin high and reset W5500
    HAL_GPIO_WritePin(w5500_gpio_config.cs_port, w5500_gpio_config.cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(w5500_gpio_config.reset_port, w5500_gpio_config.reset_pin, GPIO_PIN_RESET);
    tmp = 0xFF;
    while (tmp--);
    HAL_GPIO_WritePin(w5500_gpio_config.reset_port, w5500_gpio_config.reset_pin, GPIO_PIN_SET);

    // Register MCU-specific callbacks
    reg_wizchip_cris_cbfunc(mcu_cris_enter, mcu_cris_exit);
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
    reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);

    // Initialize W5500
    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1) {
    	dbg_print("W5500: Failed to initialize.\r\n");
        while (1);
    }

    W5500_Enable_Interrupts();
    dbg_print("W5500: initialized successfully.\r\n");

    // Set the network information
    if (net_info != NULL) {
        wizchip_setnetinfo(net_info);
        dbg_print("W5500: Network information set successfully.\r\n");
    } else {
    	dbg_print("W5500: Network information set failed!\r\n");
        while (1);
    }

    // Optionally, retrieve and verify the network configuration
    dbg_print("W5500: Network information-\r\n");
    wizchip_getnetinfo(net_info);
    dbg_print("MAC\t: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           net_info->mac[0], net_info->mac[1], net_info->mac[2],
           net_info->mac[3], net_info->mac[4], net_info->mac[5]);
    dbg_print("IP\t: %d.%d.%d.%d\r\n",
           net_info->ip[0], net_info->ip[1], net_info->ip[2], net_info->ip[3]);
    dbg_print("SN\t: %d.%d.%d.%d\r\n",
           net_info->sn[0], net_info->sn[1], net_info->sn[2], net_info->sn[3]);
    dbg_print("GW\t: %d.%d.%d.%d\r\n",
           net_info->gw[0], net_info->gw[1], net_info->gw[2], net_info->gw[3]);
    dbg_print("DNS\t: %d.%d.%d.%d\r\n\r\n",
           net_info->dns[0], net_info->dns[1], net_info->dns[2], net_info->dns[3]);

}


// W5500 Reset
void W5500_Reset(void) {
    HAL_GPIO_WritePin(w5500_gpio_config.reset_port, w5500_gpio_config.reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(w5500_gpio_config.reset_port, w5500_gpio_config.reset_pin, GPIO_PIN_SET);
    HAL_Delay(100);
    W5500Init(&w5500_gpio_config, &netInfo);
}

/*@brief : writes new net info to the chip
 * */
void W5500_Update_NetInfo(wiz_NetInfo *net_info){
	wizchip_setnetinfo(net_info);
	W5500Init(&w5500_gpio_config, net_info);

}


