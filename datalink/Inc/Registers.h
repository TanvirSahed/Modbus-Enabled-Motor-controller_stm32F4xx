/*
 * Registers.h
 *
 *  Created on: Mar 1, 2025
 *      Author: User
 */

#ifndef INC_REGISTERS_H_
#define INC_REGISTERS_H_
#include <stdint.h>
#include "modbus.h"


/*Discrete input Resister address list ---------------------*/
typedef enum {
	MB_DIA_,
	MB_DIA_MAX,
}MB_DIAddres;

/*Coil output Resister address -----------------------------*/
typedef enum {
	/*Utilities--------------*/
	MB_COA_RESTART = 0,
	MB_COA_DBUG_ENABLE,

	/*Motor Controller----------*/
	MB_COA_STPR_CTRL_ENABLE = 10,
	MB_COA_STPR_CTRL_CLEAR_FAULT,

	MB_COA_HB_CTRL_ENABLE =15,
	MB_COA_HB_CTRL_CLEAR_FAULT,

	MB_COA_MAX,
}MB_COAddres;


/*Input Resister address ---------------------*/
typedef enum {
	/*Utilities----------------*/
	MB_IRA_DEV_HW_VERSION = 0,
	MB_IRA_DEV_FW_VERSION,

	/*Modbus RTU --------------*/
	MB_IRA_MB_RTU_SLAVE_ID,



	/*Encoder1 Stepper-------------------*/
	MB_IRA_ENC1_POSITION = 10,
	MB_IRA_ENC1_DIRECTION,
	MB_IRA_ENC1_PPR,
	MB_IRA_ENC1_COUNT_LSB16,

	/*Encoder1 HB-------------------*/
	MB_IRA_ENC0_POSITION = 15,
	MB_IRA_ENC0_DIRECTION,
	MB_IRA_ENC0_PPR,
	MB_IRA_ENC0_COUNT_LSB16,

//	MB_IRA_ENC1_COUNT_MSB16,

	/*Stepper motor-------------*/
	MB_IRA_STEPPER_MOTOR_STEPSPERREV = 20,

	/*DM542T Driver-------------*/
	MB_IRA_DM542T_PPR = 30,

	/*PID-----------------------*/
	MB_IRA_PID0_FEEDBACK = 40, //HB
	MB_IRA_PID0_OUTPUT,		   //HB

	MB_IRA_PID1_FEEDBACK = 45,//STP
	MB_IRA_PID1_OUTPUT,		  //STP


	/*HB Controller*/
	MB_IRA_HB_CTRL_DIR = 50,
	MB_IRA_HB_CTRL_POS_PERCENT,
	MB_IRA_HB_CTRL_POS_DEGREE,
	MB_IRA_HB_CTRL_MAX_POS_DEGREE,
//	MB_IRA_MOTR_CTRL_LEARNING_SATAUS,
	MB_IRA_HB_CTRL_FAULT_STATUS,


	/*Stepper Motor Controller-----------*/
	MB_IRA_STPR_CTRL_DIR = 60,
	MB_IRA_STPR_CTRL_POS_PERCENT,
	MB_IRA_STPR_CTRL_POS_DEGREE,
	MB_IRA_STPR_CTRL_MAX_POS_DEGREE,
//	MB_IRA_MOTR_CTRL_LEARNING_SATAUS,
	MB_IRA_STPR_CTRL_FAULT_STATUS,




	MB_IRA_MAX,
}MB_IRAddres;

/*Holding Resister address ----------------------------------*/
typedef enum {
/*Utilities----------------*/


/*Encoder-------------------*/
	MB_HRA_ENC0_RESOLUTION = 10, //HB
	MB_HRA_ENC1_RESOLUTION = 15, //STP

	/*Stepper motor-------------*/
	MB_HRA_STEPPER_MOTOR_STEPANGLE = 20,
	MB_HRA_STEPPER_MOTOR_MAX_RPM,

/*DM542T Driver-------------*/
	MB_HRA_DM542T_MICROSTEP = 30,

/*HB Motor Controller----------*/
	MB_HRA_HB_CTRL_CALIBRATION_SPEED = 40,
	MB_HRA_HB_CTRL_WAITING_TIMEOUT,
	MB_HRA_HB_CTRL_PATH,  //noraml = 1 , reverse = 2

	/*Stepper Motor Controller----------*/
//	MB_HRA_MOTR_CTRL_SPEED,
//	MB_HRA_MOTR_CTRL_SET_POS_PERCENT,
//	MB_HRA_STPR_CTRL_TYPE = 40,
	MB_HRA_STPR_CTRL_CALIBRATION_SPEED,
	MB_HRA_STPR_CTRL_WAITING_TIMEOUT,
//	MB_HRA_STPR_CTRL_WAITING_TIMEOUT_MSB,
	MB_HRA_STPR_CTRL_PATH,  //noraml = 1 , reverse = 2


	/*Stepper Motr Safety-----------------*/
	MB_HRA_MOTR_SAFETY_HOLDING_DUTY,
	MB_HRA_MOTR_SAFETY_RUNNING_DUTY,
	MB_HRA_MOTR_SAFETY_WAITING_TIME,

	/*PID 0 HB---------------------------*/
	MB_HRA_PID0_SETPOINT = 50,
	MB_HRA_PID0_KP,
	MB_HRA_PID0_KI,
	MB_HRA_PID0_KD,
	MB_HRA_PID0_I_OUT_LIMIT,

	/*PID 1 STEPPER---------------------*/
	MB_HRA_PID1_SETPOINT = 60,
	MB_HRA_PID1_KP,
	MB_HRA_PID1_KI,
	MB_HRA_PID1_KD,
	MB_HRA_PID1_I_OUT_LIMIT,

	/*Motor Controller 2-----------*/
	//It will start from 60


	/*Modbus TCP Net info--------*/
	MB_HRA_MB_TCPS_PORT = 70,
	MB_HRA_MB_TCPS_MAC1,
	MB_HRA_MB_TCPS_MAC2,
	MB_HRA_MB_TCPS_MAC3,
	MB_HRA_MB_TCPS_MAC4,
	MB_HRA_MB_TCPS_MAC5,
	MB_HRA_MB_TCPS_MAC6,

	MB_HRA_MB_TCPS_IP1,
	MB_HRA_MB_TCPS_IP2,
	MB_HRA_MB_TCPS_IP3,
	MB_HRA_MB_TCPS_IP4,

	MB_HRA_MB_TCPS_SN1,
	MB_HRA_MB_TCPS_SN2,
	MB_HRA_MB_TCPS_SN3,
	MB_HRA_MB_TCPS_SN4,

	MB_HRA_MB_TCPS_GW1,
	MB_HRA_MB_TCPS_GW2,
	MB_HRA_MB_TCPS_GW3,
	MB_HRA_MB_TCPS_GW4,

	MB_HRA_MB_TCPS_APPLY_CHANGE=94,


	MB_HRA_MB_RTU_BAUD_LSB = 100 , //least 16 bits
	MB_HRA_MB_RTU_BAUD_MSB,
	MB_HRA_MB_RTU_DATA_BIT,
	MB_HRA_MB_RTU_STOP_BIT,
	MB_HRA_MB_RTU_PARITY,

	MB_HRA_MB_RTU_APPLY_CHANGE,


	MB_HRA_MAX,
}MB_HRAddres;



typedef enum {
	MB_REG_ERR_OK,
	MB_REG_ERR_INVALID_ADDRESS,
	MB_REG_ERR_INVALID_VALUE,
	MB_REG_ERR_NULL_POINTER,
}MB_REG_Error;


typedef struct {
	uint8_t isChanged;
	uint16_t startAddr;
	uint16_t numOfReg;
}modbus_reg_temp;

typedef struct modbus_reg{
	modbus_reg_temp co;
	modbus_reg_temp hr;
	modbus_mapping_t *reg;
}modbus_reg_ts;

MB_REG_Error modbus_reg_init(modbus_reg_ts *reg);
modbus_mapping_t* modbus_reg_map(void);

/*Discrete input Resister*/
MB_REG_Error modbus_reg_setDI(uint16_t address, uint8_t value);
MB_REG_Error modbus_reg_getDI(uint16_t address, uint8_t *value);

/*Coil output Resister*/
MB_REG_Error modbus_reg_setCO(uint16_t address, uint8_t value);
MB_REG_Error modbus_reg_getCO(uint16_t address, uint8_t *value);

/*Input Resister*/
MB_REG_Error modbus_reg_setIR(uint16_t address, uint16_t value);
MB_REG_Error modbus_reg_getIR(uint16_t address, uint16_t *value);

/*Input Resister*/
MB_REG_Error modbus_reg_setHR(uint16_t address, uint16_t value);
MB_REG_Error modbus_reg_getHR(uint16_t address, uint16_t *value);



MB_REG_Error modbus_reg_readDI_cb(void *ctx, uint8_t *pData, uint16_t address, uint16_t numOfReg);
MB_REG_Error modbus_reg_readCO_cb(void *ctx, uint8_t *pData, uint16_t address, uint16_t numOfReg);
MB_REG_Error modbus_reg_writeCO_cb(void *ctx, uint8_t *pData, uint16_t address, uint16_t numOfReg);
MB_REG_Error modbus_reg_readIR_cb(void *ctx, uint16_t *pData, uint16_t address, uint16_t numOfReg);
MB_REG_Error modbus_reg_readHR_cb(void *ctx, uint16_t *pData, uint16_t address, uint16_t numOfReg);
MB_REG_Error modbus_reg_writeHR_cb(void *ctx, uint16_t *pData, uint16_t address, uint16_t numOfReg);

void modbus_tcp_update_netIfs(void);
void modbus_rtu_update_serialConfig(void);
#endif /* INC_REGISTERS_H_ */

