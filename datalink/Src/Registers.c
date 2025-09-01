/*
 * Registers.c
 *
 *  Created on: Mar 1, 2025
 *      Author: User
 */
#include "Registers.h"
#include <string.h>
#include "debug.h"
#include "app_main.h"
#include "flash_addrs.h"
#include "w25qxx_flash.h"
#include "modbus_tcp_driver.h"
#include "w5500_spi_handler.h"
#include "modbus_uart_manager.h"
#include "Config.h"
#include "main.h"

#define GET_CO(address)  modbus_reg_mapping->tab_bits[address]
#define GET_HR(address)  modbus_reg_mapping->tab_registers[address]

extern GlobalVar_ts gv;
extern UART_HandleTypeDef huart1;
modbus_mapping_t *modbus_reg_mapping;


/**
 * @brief   Initialize all Modbus register arrays (zeroed out).
 * @note    This should be called once at system initialization.
 */
MB_REG_Error modbus_reg_init(modbus_reg_ts *reg){

	if(reg == NULL){
		return MB_REG_ERR_NULL_POINTER;
	}
	modbus_reg_temp tesmpZeor = {0};
	reg->co = tesmpZeor;
	reg->hr = tesmpZeor;

    // Allocate Modbus register mapping
	modbus_reg_mapping = modbus_mapping_new(MB_COA_MAX, MB_DIA_MAX, MB_HRA_MAX, MB_IRA_MAX);
    if (modbus_reg_mapping == NULL) {
        return MB_REG_ERR_NULL_POINTER;
    }
    reg->reg = modbus_reg_mapping;
    return MB_REG_ERR_OK;
}

/**
 * @brief Returns a pointer to the Modbus register mapping.
 *
 * This function provides access to the internal Modbus register mapping
 * structure, which contains the configuration of holding registers,
 * input registers, coils, and discrete inputs.
 *
 * @return Pointer to the Modbus register mapping structure.
 */
modbus_mapping_t* modbus_reg_map(void){
	return modbus_reg_mapping;
}

void modbus_reg_update(){

}

/*Discrete input Resister------------------------------------*/
/**
 * @brief   Set value in Discrete Input register.
 * @param   address Address of the register.
 * @param   value   0 or 1.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 */
MB_REG_Error modbus_reg_setDI(uint16_t address, uint8_t value){
	if(address >= MB_DIA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}
	modbus_reg_mapping->tab_input_bits[address] = (value>0);
	return MB_REG_ERR_OK;
}

/**
 * @brief   Get value from Discrete Input register.
 * @param   address Address of the register.
 * @param   value   Pointer to store the value.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 * @retval  MB_REG_ERR_NULL_POINTER if value is NULL.
 */
MB_REG_Error modbus_reg_getDI(uint16_t address, uint8_t *value){
	if(address >= MB_DIA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}else if(value == NULL){
		return MB_REG_ERR_NULL_POINTER;
	}
	*value = modbus_reg_mapping->tab_input_bits[address];
	return MB_REG_ERR_OK;
}

/*Coil output Resister------------------------------------------*/
/**
 * @brief   Set value in Coil Output register.
 * @param   address Address of the register.
 * @param   value   0 or 1.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 */
MB_REG_Error modbus_reg_setCO(uint16_t address, uint8_t value){
	if(address >= MB_COA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}
	modbus_reg_mapping->tab_bits[address] = (value>0);
	return MB_REG_ERR_OK;
}

/**
 * @brief   Get value from Coil Output register.
 * @param   address Address of the register.
 * @param   value   Pointer to store the value.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 * @retval  MB_REG_ERR_NULL_POINTER if value is NULL.
 */
MB_REG_Error modbus_reg_getCO(uint16_t address, uint8_t *value){
	if(address >= MB_COA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}else if(value == NULL){
		return MB_REG_ERR_NULL_POINTER;
	}
	*value = modbus_reg_mapping->tab_bits[address];
	return MB_REG_ERR_OK;
}

/*Input Resister------------------------------------------------*/
/**
 * @brief   Set value in Input Register.
 * @param   address Address of the register.
 * @param   value   16-bit value to write.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 */
MB_REG_Error modbus_reg_setIR(uint16_t address, uint16_t value){
	if(address >= MB_IRA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}
	modbus_reg_mapping->tab_input_registers[address] = value;
	return MB_REG_ERR_OK;
}

/**
 * @brief   Get value from Input Register.
 * @param   address Address of the register.
 * @param   value   Pointer to store the value.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 * @retval  MB_REG_ERR_NULL_POINTER if value is NULL.
 */
MB_REG_Error modbus_reg_getIR(uint16_t address, uint16_t *value){
	if(address >= MB_IRA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}else if(value == NULL){
		return MB_REG_ERR_NULL_POINTER;
	}
	*value = modbus_reg_mapping->tab_input_registers[address];
	return MB_REG_ERR_OK;
}

/*Input Resister-------------------------------------*/
/**
 * @brief   Set value in Holding Register.
 * @param   address Address of the register.
 * @param   value   16-bit value to write.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 */
MB_REG_Error modbus_reg_setHR(uint16_t address, uint16_t value){
	if(address >= MB_HRA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}
	modbus_reg_mapping->tab_registers[address] = value;
	return MB_REG_ERR_OK;
}

/**
 * @brief   Get value from Holding Register.
 * @param   address Address of the register.
 * @param   value   Pointer to store the value.
 * @retval  MB_REG_ERR_OK on success.
 * @retval  MB_REG_ERR_INVALID_ADDRESS if address is out of range.
 * @retval  MB_REG_ERR_NULL_POINTER if value is NULL.
 */
MB_REG_Error modbus_reg_getHR(uint16_t address, uint16_t *value){
	if(address >= MB_HRA_MAX){
		return MB_REG_ERR_INVALID_ADDRESS;
	}else if(value == NULL){
		return MB_REG_ERR_NULL_POINTER;
	}
	*value = modbus_reg_mapping->tab_registers[address];
	return MB_REG_ERR_OK;
}



/* Resister Read Write functions-------------------------------------*/

/**
 * @brief Modbus callback to read Discrete Inputs (DI).
 *
 * This function is invoked when the Modbus master requests to read
 * discrete input registers. It validates the address range and sets
 * the appropriate error code if any address is invalid.
 *
 * @param ctx Pointer to the Modbus context (not used here, but kept for API compatibility).
 * @param pData Pointer to the buffer where read data should be stored.
 * @param address Starting address of the discrete input to be read.
 * @param numOfReg Number of discrete input registers to read.
 *
 * @return MB_REG_ERR_OK if all addresses are valid,
 *         MB_REG_ERR_INVALID_ADDRESS if an invalid address is encountered.
 */
MB_REG_Error modbus_reg_readDI_cb(void *ctx, uint8_t *pData, uint16_t address, uint16_t numOfReg){
	MB_REG_Error ret = MB_REG_ERR_OK;
	uint16_t endAddress = address + numOfReg -1;


	for(; address <= endAddress; address++){
		switch (address) {
			case 0:

				break;

			default:
				ret = MB_REG_ERR_INVALID_ADDRESS;
				break;
		}
	}
	return ret;
}


/**
 * @brief Modbus callback to read Coil (CO) values.
 *
 * This function handles requests to read coil values from the Modbus master.
 * It reads the appropriate coil value for each address in the specified range
 * and stores it in the output buffer. If an invalid address is encountered,
 * it sets an error code.
 *
 * @param ctx Pointer to the Modbus context (unused in this implementation).
 * @param pData Pointer to the buffer where the coil data should be written.
 * @param address Starting coil address to read.
 * @param numOfReg Number of coils to read.
 *
 * @return MB_REG_ERR_OK on success, or MB_REG_ERR_INVALID_ADDRESS if any address is invalid.
 */
MB_REG_Error modbus_reg_readCO_cb(void *ctx, uint8_t *pData, uint16_t address, uint16_t numOfReg){

	MB_REG_Error ret = MB_REG_ERR_OK;
	uint16_t endAddress = address + numOfReg -1;



	for(; address <= endAddress; address++){
		switch (address) {
			case MB_COA_RESTART:
				pData[address] = 0;
				break;
			case MB_COA_DBUG_ENABLE:
				pData[address] = dbg_isEnable();
				break;

			/*STPR controller */
			case MB_COA_STPR_CTRL_ENABLE:
				pData[address] = gv.motoCtrlStpr.enable;
				break;
			case MB_COA_STPR_CTRL_CLEAR_FAULT:
				pData[address] = gv.motoCtrlStpr.faultClear;
				break;

			/*HB controller */
			case MB_COA_HB_CTRL_ENABLE:
				pData[address] = gv.motoCtrlHB.enable;
				break;

			case MB_COA_HB_CTRL_CLEAR_FAULT:
				pData[address] = gv.motoCtrlHB.faultClear;
				break;

			default:
				ret = MB_REG_ERR_INVALID_ADDRESS;
				break;
		}
	}
	return ret;
}


/**
 * @brief Modbus callback to write Coil (CO) values.
 *
 * This function handles requests from the Modbus master to write to coil addresses.
 * It updates the corresponding coil states.
 *
 * @param ctx Pointer to the Modbus context (currently unused).
 * @param pData Pointer to the buffer containing the new coil values.
 * @param address Starting coil address to write.
 * @param numOfReg Number of coils to write.
 *
 * @return MB_REG_ERR_OK on success,
 *         MB_REG_ERR_INVALID_VALUE if the system is in an invalid state for writing.
 */
MB_REG_Error modbus_reg_writeCO_cb(void *ctx, uint8_t *pData, uint16_t address, uint16_t numOfReg){
//	gv.mbReg.co.isChanged = 1;
//	gv.mbReg.co.startAddr = address;
//	gv.mbReg.co.numOfReg = numOfReg;
//
//
	dbg_print("wCO: [%d, %d]: ", address, numOfReg);
	for(uint8_t i = address; i < address+numOfReg; i++){
		dbg_print("%02X ",pData[i]);
	}
	dbg_print("\r\n");

	if(gv.motoCtrlStpr.isStartUp && gv.motoCtrlStpr.isLearnOpenPos) {
		return MB_REG_ERR_INVALID_VALUE;
	}

	MB_REG_Error ret = MB_REG_ERR_OK;
	uint16_t endAddress = address + numOfReg -1;

	for(; address <= endAddress; address++){
		uint8_t value = pData[address]>0;
		switch (address) {
			case MB_COA_RESTART:
				gv.restart = value;
				break;
			case MB_COA_DBUG_ENABLE:
				value ? dbg_enable() : dbg_disable();
				break;
			case MB_COA_STPR_CTRL_ENABLE:
				gv.motoCtrlStpr.enable = value;
				break;
			case MB_COA_STPR_CTRL_CLEAR_FAULT:
				gv.motoCtrlStpr.faultClear = value;
				break;
			default:
				break;
		}
	}
	return ret;
}

/**
 * @brief Modbus callback to read Input Registers (IR).
 *
 * This function processes requests from a Modbus master to read input registers.
 * Each address is mapped to a specific system parameter.
 *
 * @param ctx Pointer to the Modbus context (unused).
 * @param pData Pointer to the buffer where input register data should be written.
 *              Each value is 16-bit.
 * @param address Starting input register address to read.
 * @param numOfReg Number of input registers to read.
 *
 * @return MB_REG_ERR_OK if successful.
 */
MB_REG_Error modbus_reg_readIR_cb(void *ctx, uint16_t *pData, uint16_t address, uint16_t numOfReg){

	MB_REG_Error ret = MB_REG_ERR_OK;
	uint16_t endAddress = address + numOfReg -1;
	for(; address <= endAddress; address++){
		uint16_t *value = &pData[address];
		switch (address) {
		/*STEPPER controller enc 1*/
			case MB_IRA_ENC1_POSITION:
				*value = gv.enc1.posAngle*100.0f;
				break;
			case MB_IRA_ENC1_DIRECTION:
				*value = gv.enc1.dir;
				break;
			case MB_IRA_ENC1_PPR:
				*value = gv.enc1.ppr;
				break;
			case MB_IRA_ENC1_COUNT_LSB16:
				*value = (uint16_t)gv.enc1.count;
				break;

		   /*HB controller enc 0*/
			case MB_IRA_ENC0_POSITION:
				*value = gv.enc0.posAngle*100.0f;
				break;
			case MB_IRA_ENC0_DIRECTION:
				*value = gv.enc0.dir;
				break;
			case MB_IRA_ENC0_PPR:
				*value = gv.enc0.ppr;
				break;
			case MB_IRA_ENC0_COUNT_LSB16:
				*value = (uint16_t)gv.enc0.count;
				break;


			case MB_IRA_STEPPER_MOTOR_STEPSPERREV:
				*value = gv.dm542tDrv->motorConfig.stepsPerRev;
				break;
			case MB_IRA_DM542T_PPR:
				*value = gv.dm542tDrv->drvConfig.ppr;
				break;

				/*PID-------------------*/
					//HB-----------------
			case MB_IRA_PID0_FEEDBACK:
				*value = gv.pid0.measurement*100.0;
				break;
			case MB_IRA_PID0_OUTPUT:
				*value = gv.pid0.output_pid*100.0;
				break;
				//STP----------------
			case MB_IRA_PID1_FEEDBACK:
				*value = gv.pid1.measurement*100.0;
				break;
			case MB_IRA_PID1_OUTPUT:
				*value = gv.pid1.output_pid*100.0;
				break;

				/*Stepper Motor Controller-------*/
			case MB_IRA_STPR_CTRL_DIR:
				*value = gv.motoCtrlStpr.dir;
				break;
			case MB_IRA_STPR_CTRL_POS_PERCENT:
				*value = (gv.motoCtrlStpr.pos*100.00*100.0)/gv.motoCtrlStpr.maxPos;
				break;
			case MB_IRA_STPR_CTRL_POS_DEGREE:
				*value = gv.motoCtrlStpr.pos*100.00;
				break;
			case MB_IRA_STPR_CTRL_MAX_POS_DEGREE:
				*value = gv.motoCtrlStpr.maxPos*100.00;
				break;
//			case MB_IRA_MOTR_CTRL_LEARNING_SATAUS:
//				*value = (gv.motoCtrlStpr.isStartUp || gv.motoCtrlStpr.isLearnOpenPos);
//				break;
			case MB_IRA_STPR_CTRL_FAULT_STATUS:
				*value = gv.motoCtrlStpr.fault;
				break;

			/*HB Motor Controller-------*/
			case MB_IRA_HB_CTRL_DIR:
				*value = gv.motoCtrlHB.dir;
				break;
			case MB_IRA_HB_CTRL_POS_PERCENT:
				*value = (gv.motoCtrlHB.pos*100.00*100.0)/gv.motoCtrlHB.maxPos;
				break;
			case MB_IRA_HB_CTRL_POS_DEGREE:
				*value = gv.motoCtrlHB.pos*100.00;
				break;
			case MB_IRA_HB_CTRL_MAX_POS_DEGREE:
				*value = gv.motoCtrlHB.maxPos*100.00;
				break;
			case MB_IRA_HB_CTRL_FAULT_STATUS:
				*value = gv.motoCtrlHB.fault;
				break;

			default:
				break;
		}
	}
	return ret;
}

/**
 * @brief Modbus callback to read Holding Registers (HR).
 *
 * This function reads configuration parameters from the system. It writes the
 * values to the buffer provided by the Modbus master.
 *
 * @param ctx Pointer to the Modbus context (unused).
 * @param pData Pointer to the buffer where holding register values will be stored.
 *              Each value is 16-bit.
 * @param address Starting register address requested by the Modbus master.
 * @param numOfReg Number of registers requested.
 *
 * @return MB_REG_ERR_OK on success.
 */
MB_REG_Error modbus_reg_readHR_cb(void *ctx, uint16_t *pData, uint16_t address, uint16_t numOfReg){

	MB_REG_Error ret = MB_REG_ERR_OK;
	uint16_t endAddress = address + numOfReg -1;
	for(; address <= endAddress; address++){
		uint16_t *value = &pData[address];
		switch (address) {

			case MB_HRA_ENC0_RESOLUTION:
				*value = gv.enc0.resolution;
				break;

			case MB_HRA_ENC1_RESOLUTION:
				*value = gv.enc1.resolution;
				break;
			case MB_HRA_STEPPER_MOTOR_STEPANGLE:
				*value = gv.dm542tDrv->motorConfig.stepAngle*100;
				break;
			case MB_HRA_STEPPER_MOTOR_MAX_RPM:
				*value = gv.dm542tDrv->motorConfig.rpmMax;
				break;
			case MB_HRA_DM542T_MICROSTEP:
				*value = gv.dm542tDrv->drvConfig.microsteps;
				break;


			   /*HB MOotr CTRL------------------*/

			case MB_HRA_HB_CTRL_CALIBRATION_SPEED:
				*value = gv.motoCtrlHB.learningSpeed*100;
				break;
			case MB_HRA_HB_CTRL_WAITING_TIMEOUT:
				*value = (uint16_t)(gv.motoCtrlHB.timer.timeout);
				break;

			case MB_HRA_HB_CTRL_PATH:
				*value = gv.motoCtrlHB.path;
				break;

				/*STEPPER Motor Controller----------*/
//			case MB_HRA_MOTR_CTRL_SET_POS_PERCENT:{
//				break;
//			case MB_HRA_STPR_CTRL_TYPE:
//				*value = gv.motoCtrlStpr.type;
//				break;
			case MB_HRA_STPR_CTRL_CALIBRATION_SPEED:
				*value = gv.motoCtrlStpr.learningSpeed*100;
				break;
			case MB_HRA_STPR_CTRL_WAITING_TIMEOUT:
				*value = (uint16_t)(gv.motoCtrlStpr.timer.timeout);
				break;
//			case MB_HRA_STPR_CTRL_WAITING_TIMEOUT_MSB:
//				*value = (uint16_t)((gv.motoCtrlStpr.timer.timeout>>16)&0x0000FFFF);
//				break;
			case MB_HRA_STPR_CTRL_PATH:
				*value = gv.motoCtrlStpr.path;
				break;

				/*Motor safety---------------*/
			case MB_HRA_MOTR_SAFETY_HOLDING_DUTY:
				*value = gv.safety.holdingDuty;
				break;
			case MB_HRA_MOTR_SAFETY_RUNNING_DUTY:
				*value = gv.safety.runningDuty;
				break;

			case MB_HRA_MOTR_SAFETY_WAITING_TIME:
				*value = gv.safety.timer.waitingTime;
				break;

				/*PID 0*/
			case MB_HRA_PID0_SETPOINT:
				*value = gv.pid0.setpoint*100;
				break;
			case MB_HRA_PID0_KP:
				*value = gv.pid0.kp*1000;
				break;
			case MB_HRA_PID0_KI:
				*value = gv.pid0.ki*1000;
				break;
			case MB_HRA_PID0_KD:
				*value = gv.pid0.kd*1000;
				break;
			case MB_HRA_PID0_I_OUT_LIMIT:
				*value = gv.pid0.output_i_max*100;
				break;


				/*PID 1---------------------*/
			case MB_HRA_PID1_SETPOINT:
				*value = gv.pid1.setpoint*100;
				break;
			case MB_HRA_PID1_KP:
				*value = gv.pid1.kp*1000;
				break;
			case MB_HRA_PID1_KI:
				*value = gv.pid1.ki*1000;
				break;
			case MB_HRA_PID1_KD:
				*value = gv.pid1.kd*1000;
				break;
			case MB_HRA_PID1_I_OUT_LIMIT:
				*value = gv.pid1.output_i_max*100;
				break;

				/*Modbus TCP Net info--------*/
			case MB_HRA_MB_TCPS_PORT:
				*value = gv.mbtcps.port;
				break;

			case MB_HRA_MB_TCPS_MAC1:
				*value = gv.mbtcps.netif.mac[0];
				break;
			case MB_HRA_MB_TCPS_MAC2:
				*value = gv.mbtcps.netif.mac[1];
				break;
			case MB_HRA_MB_TCPS_MAC3:
				*value = gv.mbtcps.netif.mac[2];
				break;
			case MB_HRA_MB_TCPS_MAC4:
				*value = gv.mbtcps.netif.mac[3];
				break;
			case MB_HRA_MB_TCPS_MAC5:
				*value = gv.mbtcps.netif.mac[4];
				break;
			case MB_HRA_MB_TCPS_MAC6:
				*value = gv.mbtcps.netif.mac[5];
				break;

			case MB_HRA_MB_TCPS_IP1:
				*value = gv.mbtcps.netif.ip[0];
				break;
			case MB_HRA_MB_TCPS_IP2:
				*value = gv.mbtcps.netif.ip[1];
				break;
			case MB_HRA_MB_TCPS_IP3:
				*value = gv.mbtcps.netif.ip[2];
				break;
			case MB_HRA_MB_TCPS_IP4:
				*value = gv.mbtcps.netif.ip[3];
				break;

			case MB_HRA_MB_TCPS_SN1:
				*value = gv.mbtcps.netif.sn[0];
				break;
			case MB_HRA_MB_TCPS_SN2:
				*value = gv.mbtcps.netif.sn[1];
				break;
			case MB_HRA_MB_TCPS_SN3:
				*value = gv.mbtcps.netif.sn[2];
				break;
			case MB_HRA_MB_TCPS_SN4:
				*value = gv.mbtcps.netif.sn[3];
				break;

			case MB_HRA_MB_TCPS_GW1:
				*value = gv.mbtcps.netif.gw[0];
				break;
			case MB_HRA_MB_TCPS_GW2:
				*value = gv.mbtcps.netif.gw[1];
				break;
			case MB_HRA_MB_TCPS_GW3:
				*value = gv.mbtcps.netif.gw[2];
				break;
			case MB_HRA_MB_TCPS_GW4:
				*value = gv.mbtcps.netif.gw[3];
				break;
			case MB_HRA_MB_TCPS_APPLY_CHANGE:
				*value = gv.mbtcps.applyChange;
				break;

				/*Modbus RTU parameters --------------------*/
			case MB_HRA_MB_RTU_BAUD_LSB:
				*value = gv.mbrtu.serial.baudRate & 0xFFFF;
				break;
			case MB_HRA_MB_RTU_BAUD_MSB:
				*value = (gv.mbrtu.serial.baudRate >> 16) & 0xFFFF;
				break;
			case MB_HRA_MB_RTU_DATA_BIT:
				*value = gv.mbrtu.serial.dataBit;
				break;
			case MB_HRA_MB_RTU_STOP_BIT:
				*value = gv.mbrtu.serial.stopBit;
				break;
			case MB_HRA_MB_RTU_PARITY:
				*value = gv.mbrtu.serial.parity;
				break;


			default:
				break;
		}
	}
	return ret;
}

/**
 * @brief Modbus callback to write Holding Registers (HR).
 *
 * This function updates device parameters based on values received from the Modbus master.
 *
 * It validates input values and ensures parameters remain within safe and logical boundaries.
 *
 * @param ctx Pointer to the Modbus context (unused).
 * @param pData Pointer to the array of register values received from the Modbus master.
 * @param address Starting address to write to.
 * @param numOfReg Number of registers to write.
 *
 * @return MB_REG_ERR_OK on success, or appropriate error code.
 */
MB_REG_Error modbus_reg_writeHR_cb(void *ctx, uint16_t *pData, uint16_t address, uint16_t numOfReg){

	if(gv.motoCtrlStpr.isStartUp && gv.motoCtrlStpr.isLearnOpenPos) {
		return MB_REG_ERR_INVALID_VALUE;
	}

	MB_REG_Error ret = MB_REG_ERR_OK;
	uint16_t endAddress = address + numOfReg -1;
	uint16_t startAddress = address;
	uint16_t i = 0;



	for(; address <= endAddress; address++){
		i = address - startAddress;
		uint16_t value = pData[i];
		switch (address) {
				/*Encoder-------------------*/

			case MB_HRA_ENC0_RESOLUTION: //HB
				Encoder_SetResolution(&gv.enc0, value);
				if(flash_write(FLS_ADDR_ENC0_RESOLUTION, value)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;

			case MB_HRA_ENC1_RESOLUTION: //STEPPER
				Encoder_SetResolution(&gv.enc1, value);
				if(flash_write(FLS_ADDR_ENC1_RESOLUTION, value)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;

				/*Stepper motor-------------*/
			case MB_HRA_STEPPER_MOTOR_STEPANGLE:{
					float angle = (float)value/100.00;
					if(angle > 0.0 && angle <= (float)ANGLE_MAX){
						if(DM542T_SetMotorStepAngle(gv.dm542tDrv, angle) != DM542T_ERR_OK){
							ret = MB_REG_ERR_INVALID_VALUE;
						}
						if(flash_write_float(FLS_ADDR_STEPPER_MOTOR_STEPANGLE, angle)){
							return MB_REG_ERR_INVALID_VALUE;
						}
					}else{
						ret = MB_REG_ERR_INVALID_VALUE;
					}
				}
				break;
			case MB_HRA_STEPPER_MOTOR_MAX_RPM:
				gv.dm542tDrv->motorConfig.rpmMax = value;
				if(flash_write(FLS_ADDR_STEPPER_MOTOR_MAX_RPM, value)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;

				/*DM542T Driver-------------*/
			case MB_HRA_DM542T_MICROSTEP:
				if(DM542T_SetDriverMicrostep(gv.dm542tDrv, value) != DM542T_ERR_OK){
					ret = MB_REG_ERR_INVALID_VALUE;
				}else{
					if(flash_write(FLS_ADDR_DM542T_MICROSTEP, value)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}
				break;

			/*HB Motor CTRL-----------------------------------*/

			case MB_HRA_HB_CTRL_CALIBRATION_SPEED:
				float spd = (float)value/100.0;
				if(spd >= MC_LEARNING_SPEED_MIN && spd <= MC_LEARNING_SPEED_MAX){
					gv.motoCtrlHB.learningSpeed = spd;
					if(flash_write_float(FLS_ADDR_HB_CTRL_CALIBRATION_SPEED, spd)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}else{
					ret = MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_HB_CTRL_WAITING_TIMEOUT:
				if(value >=0 && value <= UINT16_MAX){
				gv.motoCtrlHB.timer.timeout = value;
				if(flash_write(FLS_ADDR_HB_CTRL_WAITING_TIMEOUT, gv.motoCtrlHB.timer.timeout)){
					return MB_REG_ERR_INVALID_VALUE;
				}}
				break;

			case MB_HRA_HB_CTRL_PATH:
				if(value == MOTR_CTRL_PATH_CW || value == MOTR_CTRL_PATH_CCW){
					gv.motoCtrlHB.path = value;

					if(flash_write(FLS_ADDR_HB_CTRL_PATH, value)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}else{
					ret = MB_REG_ERR_INVALID_VALUE;
				}
				break;





				/*STEPPER Motor Controller----------*/
//			case MB_HRA_MOTR_CTRL_SET_POS_PERCENT:{
//					float pos = (float)value/100.0;
//					if(pos >= 0.0 && pos <= 100.0){
//						pos = (pos * gv.motoCtrlStpr.maxPos) / 100;
//						if(pos>=0.00f && pos <= gv.motoCtrlStpr.maxPos){
//							gv.motoCtrlStpr.setPos = pos;
//
//						}else{
//							ret = MB_REG_ERR_INVALID_VALUE;
//						}
//					}
//				}
//				break;

//			case MB_HRA_STPR_CTRL_TYPE:
//				if(value == MOTOR_CTRL_TYPE_STEPPER || value == MOTOR_CTRL_TYPE_HBRIDGE){
//					gv.motoCtrlStpr.type = value;
////					dbg_print("MB: Value: %d", value);
//					if(flash_write(FLS_ADDR_MOTR_CTRL_TYPE, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
//				}else{
//					ret = MB_REG_ERR_INVALID_VALUE;
//				}
//				break;
			case MB_HRA_STPR_CTRL_CALIBRATION_SPEED:
				float speed = (float)value/100.0;
				if(speed >= MC_LEARNING_SPEED_MIN && speed <= MC_LEARNING_SPEED_MAX){
					gv.motoCtrlStpr.learningSpeed = speed;
					if(flash_write_float(FLS_ADDR_STPR_CTRL_CALIBRATION_SPEED, speed)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}else{
					ret = MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_STPR_CTRL_WAITING_TIMEOUT:
//				gv.motoCtrlStpr.timer.timeout = (gv.motoCtrlStpr.timer.timeout&0xFFFF0000);
				if(value >=0 && value <= UINT16_MAX){
				gv.motoCtrlStpr.timer.timeout = value;
				if(flash_write(FLS_ADDR_STPR_CTRL_WAITING_TIMEOUT, gv.motoCtrlStpr.timer.timeout)){
					return MB_REG_ERR_INVALID_VALUE;
				}}
				break;
//			case MB_HRA_STPR_CTRL_WAITING_TIMEOUT_MSB:
//				gv.motoCtrlStpr.timer.timeout = (gv.motoCtrlStpr.timer.timeout&0x0000FFFF);
//				gv.motoCtrlStpr.timer.timeout |= (((uint32_t)value)<<16)&0xFFFF0000;
//				if(flash_write(FLS_ADDR_STPR_CTRL_WAITING_TIMEOUT, gv.motoCtrlStpr.timer.timeout)){
//					return MB_REG_ERR_INVALID_VALUE;
//				}
//				break;
			case MB_HRA_STPR_CTRL_PATH:
				if(value == MOTR_CTRL_PATH_CW || value == MOTR_CTRL_PATH_CCW){
					gv.motoCtrlStpr.path = value;

					if(flash_write(FLS_ADDR_STPR_CTRL_PATH, value)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}else{
					ret = MB_REG_ERR_INVALID_VALUE;
				}
				break;

				/*Motor Safety - -----------------*/
			case MB_HRA_MOTR_SAFETY_HOLDING_DUTY:
				 if(value <= 10000){
					 gv.safety.holdingDuty = (uint32_t)value;
					 flash_write(FLS_ADDR_MOTR_SAFETY_HOLDING_DUTY, value);
					 gv.safety.applyChange = 1;
				 }else{
					ret = MB_REG_ERR_INVALID_VALUE;
				}
				break;

			case MB_HRA_MOTR_SAFETY_RUNNING_DUTY:
				if(value <= 10000){
					gv.safety.runningDuty = (uint32_t)value;
					flash_write(FLS_ADDR_MOTR_SAFETY_RUNNING_DUTY, value);
					gv.safety.applyChange = 1;
				}else{
						ret = MB_REG_ERR_INVALID_VALUE;
					}
				break;

			case MB_HRA_MOTR_SAFETY_WAITING_TIME:
				if(value >= 100 && value <=1500){

					gv.safety.timer.waitingTime = value;


				}else{
					gv.safety.timer.waitingTime = CONF_DEF_MOTOSFTY_WAITING_TIME_MS;
					ret = MB_REG_ERR_INVALID_VALUE;
				}
				flash_write(FLS_ADDR_MOTR_SAFETY_WAITING_TIME, gv.safety.timer.waitingTime);
				break;

				/*PID 0---------------------*/
			case MB_HRA_PID0_SETPOINT:{
					float pos = (float)value/100.0;
					if(pos >= 0.0 && pos <= 100.0){
						pos = (pos * gv.motoCtrlHB.maxPos) / 100;
						if(pos>=0.00f && pos <= gv.motoCtrlHB.maxPos){
							gv.motoCtrlHB.setPos = pos;
							gv.pid0.setpoint = pos;
//							if(flash_write_float(FLS_ADDR_PID1_SETPOINT, pos)){
//								return MB_REG_ERR_INVALID_VALUE;
//							}
						}else{
							ret = MB_REG_ERR_INVALID_VALUE;
						}
					}else{
						ret = MB_REG_ERR_INVALID_VALUE;
					}
				}
				break;
			case MB_HRA_PID0_KP:
				gv.pid0.kp = ((float)value/1000.00);
				if(flash_write_float(FLS_ADDR_PID0_KP, gv.pid0.kp)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_PID0_KI:
				gv.pid0.ki = ((float)value/1000.00);
				if(flash_write_float(FLS_ADDR_PID0_KI, gv.pid0.ki)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_PID0_KD:
				gv.pid0.kd = ((float)value/1000.00);
				if(flash_write_float(FLS_ADDR_PID0_KD, gv.pid0.kd)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_PID0_I_OUT_LIMIT:
				float temp = (float)value/100.00;
				if(temp>=0.00 && temp <= 100.00){
					gv.pid0.output_i_max = temp;
					if(flash_write_float(FLS_ADDR_PID0_I_OUT_LIMIT, gv.pid0.output_i_max)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}
				break;



				/*PID 1---------------------*/
			case MB_HRA_PID1_SETPOINT:{
					float pos = (float)value/100.0;
					if(pos >= 0.0 && pos <= 100.0){
						pos = (pos * gv.motoCtrlStpr.maxPos) / 100;
						if(pos>=0.00f && pos <= gv.motoCtrlStpr.maxPos){
							gv.motoCtrlStpr.setPos = pos;
							gv.pid1.setpoint = pos;
//							if(flash_write_float(FLS_ADDR_PID1_SETPOINT, pos)){
//								return MB_REG_ERR_INVALID_VALUE;
//							}
						}else{
							ret = MB_REG_ERR_INVALID_VALUE;
						}
					}else{
						ret = MB_REG_ERR_INVALID_VALUE;
					}
				}
				break;
			case MB_HRA_PID1_KP:
				gv.pid1.kp = ((float)value/1000.00);
				if(flash_write_float(FLS_ADDR_PID1_KP, gv.pid1.kp)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_PID1_KI:
				gv.pid1.ki = ((float)value/1000.00);
				if(flash_write_float(FLS_ADDR_PID1_KI, gv.pid1.ki)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_PID1_KD:
				gv.pid1.kd = ((float)value/1000.00);
				if(flash_write_float(FLS_ADDR_PID1_KD, gv.pid1.kd)){
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_PID1_I_OUT_LIMIT:
				float tmp = (float)value/100.00;
				if(tmp>=0.00 && tmp <= 100.00){
					gv.pid1.output_i_max = tmp;
					if(flash_write_float(FLS_ADDR_PID1_I_OUT_LIMIT, gv.pid1.output_i_max)){
						return MB_REG_ERR_INVALID_VALUE;
					}
				}
				break;

				/*Modbus TCP Net info--------*/
			case MB_HRA_MB_TCPS_PORT:
				gv.mbtcps.port = value;
//				if(flash_write(FLS_ADDR_MB_TCPS_PORT, value)){
//					return MB_REG_ERR_INVALID_VALUE;
//				}
				break;

				//MAC
			case MB_HRA_MB_TCPS_MAC1:
				if(value <= UINT8_MAX){
					//modify this - -   ----  ----  -- - -- -
					gv.mbtcps.netif.mac[0] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_MAC1, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_MAC2:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.mac[1] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_MAC2, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_MAC3:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.mac[2] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_MAC3, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_MAC4:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.mac[3] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_MAC4, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_MAC5:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.mac[4] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_MAC5, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_MAC6:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.mac[5] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_MAC6, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;

				//IP
			case MB_HRA_MB_TCPS_IP1:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.ip[0] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_IP1, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_IP2:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.ip[1] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_IP2, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_IP3:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.ip[2] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_IP3, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_IP4:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.ip[3] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_IP4, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;

				//SN
			case MB_HRA_MB_TCPS_SN1:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.sn[0] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_SN1, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_SN2:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.sn[1] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_SN2, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_SN3:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.sn[2] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_SN3, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_SN4:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.sn[3] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_SN4, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;

			case MB_HRA_MB_TCPS_GW1:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.gw[0] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_GW1, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_GW2:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.gw[1] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_GW2, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_GW3:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.gw[2] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_GW3, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_GW4:
				if(value <= UINT8_MAX){
					gv.mbtcps.netif.gw[3] = value;
//					if(flash_write(FLS_ADDR_MB_TCPS_GW4, value)){
//						return MB_REG_ERR_INVALID_VALUE;
//					}
				}else{
					return MB_REG_ERR_INVALID_VALUE;
				}
				break;
			case MB_HRA_MB_TCPS_APPLY_CHANGE:   // -----------------------------------changes to be made
				if(value <= UINT8_MAX){
					gv.mbtcps.applyChange = value;
					if(value){
						flash_write(FLS_ADDR_MB_TCPS_PORT, gv.mbtcps.port);

						flash_write(FLS_ADDR_MB_TCPS_MAC1, gv.mbtcps.netif.mac[0]);
						flash_write(FLS_ADDR_MB_TCPS_MAC2, gv.mbtcps.netif.mac[1]);
						flash_write(FLS_ADDR_MB_TCPS_MAC3, gv.mbtcps.netif.mac[2]);
						flash_write(FLS_ADDR_MB_TCPS_MAC4, gv.mbtcps.netif.mac[3]);
						flash_write(FLS_ADDR_MB_TCPS_MAC5, gv.mbtcps.netif.mac[4]);
						flash_write(FLS_ADDR_MB_TCPS_MAC6, gv.mbtcps.netif.mac[5]);

						flash_write(FLS_ADDR_MB_TCPS_IP1, gv.mbtcps.netif.ip[0]);
						flash_write(FLS_ADDR_MB_TCPS_IP2, gv.mbtcps.netif.ip[1]);
						flash_write(FLS_ADDR_MB_TCPS_IP3, gv.mbtcps.netif.ip[2]);
						flash_write(FLS_ADDR_MB_TCPS_IP4, gv.mbtcps.netif.ip[3]);

						flash_write(FLS_ADDR_MB_TCPS_SN1, gv.mbtcps.netif.sn[0]);
						flash_write(FLS_ADDR_MB_TCPS_SN2, gv.mbtcps.netif.sn[1]);
						flash_write(FLS_ADDR_MB_TCPS_SN3, gv.mbtcps.netif.sn[2]);
						flash_write(FLS_ADDR_MB_TCPS_SN4, gv.mbtcps.netif.sn[3]);

						flash_write(FLS_ADDR_MB_TCPS_GW1, gv.mbtcps.netif.gw[0]);
						flash_write(FLS_ADDR_MB_TCPS_GW2, gv.mbtcps.netif.gw[1]);
						flash_write(FLS_ADDR_MB_TCPS_GW3, gv.mbtcps.netif.gw[2]);
						flash_write(FLS_ADDR_MB_TCPS_GW4, gv.mbtcps.netif.gw[3]);



					}
				}else{
						return MB_REG_ERR_INVALID_VALUE;}
				break;

				/*Modbus RTU ----------------------------------------------- */

			case MB_HRA_MB_RTU_BAUD_LSB:
				if (value > UINT16_MAX) {
					// fallback to default
					gv.mbrtu.serial.baudRate = CONF_DEF_MB_RTU_BAUD;

					return MB_REG_ERR_INVALID_VALUE;

				} else {
					gv.mbrtu.serial.baudRate = (gv.mbrtu.serial.baudRate & 0xFFFF0000) |
							value;
				}
				break;

			case MB_HRA_MB_RTU_BAUD_MSB:
				if (value > UINT16_MAX) {
					// fallback to default
					gv.mbrtu.serial.baudRate = CONF_DEF_MB_RTU_BAUD;

						return MB_REG_ERR_INVALID_VALUE;

				} else {
					gv.mbrtu.serial.baudRate = (gv.mbrtu.serial.baudRate & 0x0000FFFF) |
							((uint32_t)value << 16);
					}
				break;

			// parity data bit stop bit etc

			case MB_HRA_MB_RTU_APPLY_CHANGE:
				if (value < UINT8_MAX) {
					gv.mbrtu.applyChange = value;
					if(gv.mbrtu.applyChange){
						flash_write(FLS_ADDR_MB_RTU_BAUD, gv.mbrtu.serial.baudRate);
						flash_write(FLS_ADDR_MB_RTU_DATA_BIT, gv.mbrtu.serial.dataBit);
						flash_write(FLS_ADDR_MB_RTU_STOP_BIT, gv.mbrtu.serial.stopBit);
						flash_write(FLS_ADDR_MB_RTU_PARITY, gv.mbrtu.serial.parity);
					}


				}else{
					return MB_REG_ERR_INVALID_VALUE;
					}
				break;










			default:
				ret = MB_REG_ERR_INVALID_ADDRESS;
				break;}
	}
	return ret;
}



void modbus_reg_updateCO(void){
	if(gv.mbReg.co.isChanged){
		gv.mbReg.co.isChanged = 0;

		/*Encoder-----------*/
//		gv.encHandler1.enable = GET_CO(MB_COA_ENC1_ENABLE);
//		gv.encHandler1.startLearning = GET_CO(MB_COA_ENC1_LEARN_START);


	}
}


void modbus_reg_updateHR(void){
	if(gv.mbReg.hr.isChanged){
		gv.mbReg.hr.isChanged = 0;

		/*Encoder-----------*/
//		gv.encHandler1.sampleMax = GET_HR(MB_HRA_ENC1_LEARN_SAMPLE);
//		gv.encHandler1.retryMax = GET_HR(MB_HRA_ENC1_LEARN_RETRY);
//		gv.encHandler1.speed = GET_HR(MB_HRA_ENC1_LEARN_SPEED);


	}
}

void modbus_tcp_update_netIfs(void){
	W5500_Update_NetInfo(&gv.mbtcps.netif);
	Modbus_TCP_Init((char*)gv.mbtcps.netif.ip, gv.mbtcps.port);

}

void modbus_rtu_update_serialConfig(void){
//	HAL_UART_DMAStop(&huart1);
////	__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
//	Deinitialize_ModbusRTU(CONF_DEF_MB_RTU_UART_IDX);
//
	Initialize_ModbusRTU(&huart1, &gv.mbrtu.serial,
				RS485_DIR_GPIO_Port, RS485_DIR_Pin, 1, CONF_DEF_MB_RTU_UART_IDX);

//	    // Step 1: Stop everything cleanly
//	    HAL_UART_DMAStop(&huart1);
//	    __HAL_UART_DISABLE(&huart1);
//	    __HAL_UART_FLUSH_DRREGISTER(&huart1);
//	    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//	    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
//	    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
//
//	    HAL_UART_DeInit(&huart1);  // Crucial
//
//	    // Step 2: Deinit modbus context
//	    Deinitialize_ModbusRTU(CONF_DEF_MB_RTU_UART_IDX);

	    // Step 3: Re-init modbus context with updated serial config
//	    if (Initialize_ModbusRTU(&huart1, &gv.mbrtu.serial,
//	                             RS485_DIR_GPIO_Port, RS485_DIR_Pin,
//	                             1, CONF_DEF_MB_RTU_UART_IDX) == 0) {
//	        dbg_print("✅ Modbus UART reinitialized with new settings.\n");
//	    } else {
//	        dbg_print("❌ Failed to reinitialize UART/Modbus.\n");
//	    }



}
