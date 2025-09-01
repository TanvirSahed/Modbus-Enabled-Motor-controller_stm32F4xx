/*
 * Settings.c
 *
 *  Created on: Dec 9, 2024
 *      Author: RusselPC
 */

#include "Settings.h"
#include "debug.h"
#include "Define.h"
#include "Config.h"
#include "app_main.h"
#include "Encoder.h"
#include "DM542T.h"
#include "DM542T_Interface.h"
#include "Timer.h"
#include "TimeStamp.h"
#include "DM542TDriverIf.h"
#include "MotorController.h"
#include "BTS7960.h"
#include "BTS7960_Interface.h"
#include "BTS7960DriverIf.h"
#include "flash_addrs.h"
#include "w25qxx_flash.h"

#include "w5500_spi_handler.h"
#include "wizchip_conf.h"
#include "w5500.h"
#include "datalink.h"
#include "rs485_handler.h"
#include "modbus.h"
#include "modbus_tcp_driver.h"
#include "Registers.h"
#include "pwm_driver.h"


extern GlobalVar_ts gv;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim10;


extern DM542T_ts dm542t;
extern BTS7960_ts bts7960;
extern MotorCtrl_DriverInterface_ts dm542tDrvIf;
extern MotorCtrl_DriverInterface_ts bts7960DrvIf;
extern MotorCtrl_LimitSenInterface_ts	limitSenInterface;
extern MotorCtrl_Callback_ts motoCtrlCallback;

//motor safety system
//extern PWM_Instance pwm;


void Settings_PreInit(void){
	// loads required info from flash at first

	dbg_enable();
	dbg_print("\r\n---Started---\r\n");

	gv.sysClk = HAL_RCC_GetSysClockFreq();
	/*Time Stamp timer----------------*/
	TS_Init();

	/*W25QXX Flash storage------------*/
	if (flash_init()) {
		 dbg_print("Flash: Init Failed!\r\n");
	}else{
		 dbg_print("Flash: Init Success!\r\n");
	}
	//flash_cleanup();

	/*Modbus RTU Serial Comms DEF*/
	//	115200, 'N', 8, 1
	gv.mbrtu.serial.baudRate = CONF_DEF_MB_RTU_BAUD;
	gv.mbrtu.serial.dataBit = CONF_DEF_MB_RTU_DATA_BIT;
	gv.mbrtu.serial.stopBit = CONF_DEF_MB_RTU_STOP_BIT;
	gv.mbrtu.serial.parity = CONF_DEF_MB_RTU_PARITY;
	//gv.enc0 = gv.enc1;

}


void Settings_Init(void){

	/*Network configuration------------------------*/
	/*W5500 pin configuration*/
	W5500_GPIO_Config_t w5500_config = {
	   .cs_port = W5500_SPI2_CS_GPIO_Port,      // Replace with your CS GPIO port
	   .cs_pin = W5500_SPI2_CS_Pin,   			// Replace with your CS GPIO pin
	   .reset_port = W5500_RST_GPIO_Port,       // Replace with your Reset GPIO port
	   .reset_pin = W5500_RST_Pin, 				// Replace with your Reset GPIO pin
	   .int_port = W5500_IT5_GPIO_Port,        	// Replace with your Interrupt GPIO port
	   .int_pin = W5500_IT5_Pin,   				// Replace with your Interrupt GPIO pin
	   .spi_handle = &hspi2      				// Replace with your SPI handle
	};

	/*pwm part*/


	PWM_Config_t pwm_config = {
	        .htim = &htim10,
	        .channel = TIM_CHANNEL_1,
	        .gpio_port = CONF_DEF_PWM_PORT,
	        .gpio_pin = CONF_DEF_PWM_PIN,
	        .gpio_af = GPIO_AF3_TIM10,
	        .timer_clk_hz = 168000000U,
	        .default_freq_hz = 20000U,
	        .default_duty_percent = 50U
	    };

	gv.safety.pwm.config = pwm_config;
	(void)PWM_Init(&gv.safety.pwm);
	(void)PWM_Start(&gv.safety.pwm);

//	/*Net info for W5500*/
//	wiz_NetInfo netInfo_default = {
//		   .mac = CONF_DEF_MB_TCPS_MAC,
//		   .ip = CONF_DEF_MB_TCPS_IP,
//		   .sn = CONF_DEF_MB_TCPS_SN,
//		   .gw = CONF_DEF_MB_TCPS_GW,
//		   .dns = CONF_DEF_MB_TCPS_DNS,
//		   .dhcp = CONF_DEF_MB_TCPS_DHCP
//		};

	//Initialize Wiznet w5500 with GPIOs, SPI and NetInfo
	//dbg_print("%d\n",gv.mbtcps.netif.ip[0]);

	W5500Init(&w5500_config, &gv.mbtcps.netif);




	/*Modbus RTU initialization-------------*/
	if(modbus_reg_init(&gv.mbReg) == MB_REG_ERR_OK){
		dbg_print("Modbus: Register Init Success!\r\n");
	}else{
		dbg_print("Modbus: Register Init Failed!\r\n");
	}
	Initialize_ModbusRTU(&huart1, &gv.mbrtu.serial,
			RS485_DIR_GPIO_Port, RS485_DIR_Pin, 1, CONF_DEF_MB_RTU_UART_IDX); // UART1, CTRL GPIO, Slave ID 1, index 0
	//   Initialize_ModbusRTU(&huart2, UART2_CTRL_GPIO_Port, UART2_CTRL_Pin, 2, 1); // UART2, CTRL GPIO, Slave ID 2, index 1
	//   Initialize_ModbusRTU(&huart4, UART4_CTRL_GPIO_Port, UART4_CTRL_Pin, 3, 2); // UART2, CTRL GPIO, Slave ID 3, index 2

	/*Modbus TCP initialization-------------*/
	//change the ip according to loaded value from flash
	Modbus_TCP_Init((char*)gv.mbtcps.netif.ip, gv.mbtcps.port);
	//    run_modbus_server_blocking("192.168.0.14", MODBUS_TCP_DEFAULT_PORT); //ID must be matched with the netinfo
	//    run_modbus_server_noneblocking("192.168.0.14", MODBUS_TCP_DEFAULT_PORT); //ID must be matched with the wiz_NetInfo netinfo //Single request



	/*Encoder init-------------------------*/
	if(Encoder_Init(&gv.enc1, &htim3, CONF_ENC1_RESOLUTION) == ENCODER_ERR_NONE){
		dbg_print("ENC1: Init Success!\r\n");
	}else{
		dbg_print("ENC1: Init Failed!\r\n");
	}
	gv.enc1.chAInCap.refClk = (float)((float)HAL_RCC_GetPCLK1Freq()*2.0/((float)htim2.Init.Prescaler+1.0));
	/*Input capture for encoder channel A*/
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);



	/*Motor Control Driver init---------------*/
//	uint32_t data = 0xFFFFFFFF;
//	flash_read(FLS_ADDR_MOTR_CTRL_TYPE, &data);
//	if(!(data == MOTOR_CTRL_TYPE_STEPPER || data == MOTOR_CTRL_TYPE_HBRIDGE)){
//		data = CONF_DEF_MOTOCTRL_TYPE;
//	}
	//gv.motoCtrlStpr.type = (MotorCtrl_Type_te)data;

	//if(gv.motoCtrlStpr.type == MOTOR_CTRL_TYPE_STEPPER){
		/*DM542T Driver----------------------*/
		gv.dm542tDrv = &dm542t;
		DM542T_InitInterface(gv.dm542tDrv);
		if(DM542T_Init(gv.dm542tDrv) == DM542T_ERR_OK){
			dbg_print("DM542T: Init Success!\r\n");
		}else{
			dbg_print("DM542T: Init Failed!\r\n");
		}
		DM542T_SetDriverMicrostep(gv.dm542tDrv, CONF_DEF_DM542T_MICROSTEP);
		DM542T_SetRPM(gv.dm542tDrv, 0);

		/*Set stepper motor step angle*/
		DM542T_SetMotorStepAngle(gv.dm542tDrv, CONF_DEF_STEPPER_MOTOR_STEPANGLE);
		DM452TDrvIf_Init();

		MotorCtrl_SetMotorDriverInterface(&gv.motoCtrlStpr,&dm542tDrvIf);

		MotorCtrl_SetLimitSensorInterface(&gv.motoCtrlStpr, &limitSenInterface);
		dbg_print("---------------------The value of path of motor is : %u\n",gv.motoCtrlStpr.path);
		MotorCtrl_Init(&gv.motoCtrlStpr);
		MotorCtrl_AttachCallback(&gv.motoCtrlStpr, &motoCtrlCallback);
		gv.motoCtrlStpr.enable = CONF_DEF_STPRCTRL_ENABLE;
		gv.motoCtrlStpr.maxPos = CONF_DEF_STPRCTRL_MAX_ANGLE;	//TODO: for testing
		gv.motoCtrlStpr.setPos = 0;
		gv.motoCtrlStpr.setPosLast = 0;
		gv.motoCtrlStpr.maxSpeed = CONF_DEF_STPRCTRL_MAX_SPEED;
		gv.motoCtrlStpr.learningSpeed = CONF_DEF_STPRCTRL_CALIBRATION_SPEED;

		Timer_AttachTimeSource(&gv.motoCtrlStpr.timer, TS_GetUS);
		Timer_SetTimeout(&gv.motoCtrlStpr.timer, CONF_DEF_STPRCTRL_WAITING_TIMEOUT); // in us



	//}else{
		/*BTS7960 HBridge Motor driver--------*/
		gv.bts7960Drv = &bts7960;
		BTS7960_Init_If(gv.bts7960Drv);
		if(BTS7960_Init(gv.bts7960Drv) == BTS7960_ERR_OK){
			dbg_print("BTS7960: Init Success!\r\n");
		}else{
			dbg_print("BTS7960: Init Failed!\r\n");
		}
	//	BTS7960_Enable(&gv.bts7960Drv);
		BTS7960DrvIf_Init();
		gv.bts7960Drv->dutyOffset = CONF_DEF_BTS79600_DUTY_OFFSET;
	//}

	/*Motor controller HB-------------------*/
	MotorCtrl_SetMotorDriverInterface(&gv.motoCtrlHB,&bts7960DrvIf);

	MotorCtrl_SetLimitSensorInterface(&gv.motoCtrlHB, &limitSenInterface);
	dbg_print("---------------------The value of path of motor is : %u\n",gv.motoCtrlHB.path);
	MotorCtrl_Init(&gv.motoCtrlHB);
	MotorCtrl_AttachCallback(&gv.motoCtrlHB, &motoCtrlCallback);
	gv.motoCtrlHB.enable = CONF_DEF_HBCTRL_ENABLE;
	gv.motoCtrlHB.maxPos = CONF_DEF_HBCTRL_MAX_ANGLE;	//TODO: for testing
	gv.motoCtrlHB.setPos = 0;
	gv.motoCtrlHB.setPosLast = 0;
	gv.motoCtrlHB.maxSpeed = CONF_DEF_HBCTRL_MAX_SPEED;
	gv.motoCtrlHB.learningSpeed = CONF_DEF_HBCTRL_CALIBRATION_SPEED;

	Timer_AttachTimeSource(&gv.motoCtrlHB.timer, TS_GetUS);
	Timer_SetTimeout(&gv.motoCtrlHB.timer, 10000); // in us


	/*PID 1 & 2 ---------------------*/
	gv.pid1.enable = CONF_DEF_PID1_ENABLE;
	gv.pid1.setpoint = 0;
	gv.pid1.maxSetpoint = 0;
	gv.pid1.measurement = 0;
	gv.pid1.fbSource = 0;
	gv.pid1.hystValue = 0;
	gv.pid1.kp = CONF_DEF_PID1_KP;
	gv.pid1.ki = CONF_DEF_PID1_KI;
	gv.pid1.kd = CONF_DEF_PID1_KD;
	gv.pid1.error = 0;
	gv.pid1.last_error = 0;
	gv.pid1.last_error_time = 0;
	gv.pid1.outputMode = CONF_DEF_PID1_OUT_MODE;
	gv.pid1.output_p = 0;
	gv.pid1.output_i = 0;
	gv.pid1.output_d = 0;
	gv.pid1.output_i_max = CONF_DEF_PID1_I_OUT_MAX;
	gv.pid1.output_pid = 0;
}


void Settings_Update(void){


	for(uint16_t address = 0 ; address < FLS_ADDR_MAX; address++){
		uint32_t value = 0xFFFFFFFF;

		if(flash_read(address, &value)){
			dbg_print("Flash: Read Failed! (Address=%d)",address);
			continue;
		}
		switch (address) {
			case FLS_ADDR_MEM_FIRST_RUN_CHECK:{
					if(value == 0U){
						if(flash_cleanup()){
							dbg_print("Error: Flash erase failed!");
						}
					}

				}
				break;
				/*Encoder-------------------*/
			case FLS_ADDR_ENC1_RESOLUTION:
				if(!(value <= UINT16_MAX)){
					gv.enc1.resolution = CONF_ENC1_RESOLUTION;
				}else{
					gv.enc1.resolution = value;
				}
				break;
				/*Stepper motor-------------*/
			case FLS_ADDR_STEPPER_MOTOR_STEPANGLE:{
					float stepangle = 0.0;
					if(flash_read_float(address, &stepangle) == 0){
						if(!(stepangle > 0.0 && stepangle <= (float)ANGLE_MAX)){
							stepangle = CONF_DEF_STEPPER_MOTOR_STEPANGLE;
						}
						DM542T_SetMotorStepAngle(gv.dm542tDrv, stepangle);
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}break;
			case FLS_ADDR_STEPPER_MOTOR_MAX_RPM:
				if(!(value <= UINT16_MAX)){
					gv.dm542tDrv->motorConfig.rpmMax = CONF_DEF_STEPPER_MOTOR_MAX_RPM;
				}else{
					gv.dm542tDrv->motorConfig.rpmMax = value;
				}
				break;

				/*DM542T Driver-------------*/
			case FLS_ADDR_DM542T_MICROSTEP :
				if(!(value <= UINT16_MAX)){
					value = CONF_DEF_DM542T_MICROSTEP;
				}
				DM542T_SetDriverMicrostep(gv.dm542tDrv, value);
				break;

				/*Motor Controller----------*/
//			case FLS_ADDR_MOTR_CTRL_TYPE:
//				if(!(value <= UINT16_MAX)){
//					gv.motoCtrlStpr.type = CONF_DEF_MOTOCTRL_TYPE;
//				}else{
//					gv.motoCtrlStpr.type = value;
//				}
//				break;
			case FLS_ADDR_STPR_CTRL_CALIBRATION_SPEED:{
					float speed = 0.0;
					if(flash_read_float(address, &speed) == 0){
						if(!(speed >= MC_LEARNING_SPEED_MIN && speed <= MC_LEARNING_SPEED_MAX)){
							speed = CONF_DEF_STPRCTRL_CALIBRATION_SPEED;
						}
						dbg_print("CALIBRATION_SPEED: %0.2f",speed);
						gv.motoCtrlStpr.learningSpeed = speed;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_STPR_CTRL_WAITING_TIMEOUT:
				if(!((value <= CONF_DEF_STPRCTRL_WAITING_TIMEOUT_MAX) && (value >= CONF_DEF_STPRCTRL_WAITING_TIMEOUT_MIN) )){
					dbg_print("Flash: Read float Failed! (Address=%d)",address);
					value = CONF_DEF_STPRCTRL_WAITING_TIMEOUT;
				}
				dbg_print("read Timeout Value: %u\n",value);
				gv.motoCtrlStpr.timer.timeout = value;
				break;

			case FLS_ADDR_STPR_CTRL_PATH:
				if(!((value == MOTR_CTRL_PATH_CW) || (value == MOTR_CTRL_PATH_CCW) )){
					dbg_print("Flash: Motor Path Invalid (Address=%d)\n",address);
					value = CONF_DEF_STPRCTRL_PATH;
					}
				gv.motoCtrlStpr.path = value;
				break;

				/*Motor Safety---------------------------*/
			case FLS_ADDR_MOTR_SAFETY_HOLDING_DUTY:
				if(value<=10000){
					gv.safety.holdingDuty = value;
				}else{
					gv.safety.holdingDuty = CONF_DEF_MOTOSFTY_HOLDING_DUTY;
				}
				break;

			case FLS_ADDR_MOTR_SAFETY_RUNNING_DUTY:
				if(value<=10000){
					gv.safety.runningDuty = value;
				}else{
					gv.safety.runningDuty = CONF_DEF_MOTOSFTY_RUNNING_DUTY;
				}
				break;

			case FLS_ADDR_MOTR_SAFETY_WAITING_TIME:
				if(value >= 100 && value <=1500){
					gv.safety.timer.waitingTime = value;

				}else{
					gv.safety.timer.waitingTime = CONF_DEF_MOTOSFTY_WAITING_TIME_MS;
					}
				break;


				/*PID 0---------------------*/
			case FLS_ADDR_PID0_KP:{
					float kp = 0.0;
					if(flash_read_float(address, &kp) == 0){
						if(kp<0.0){
							kp =CONF_DEF_PID0_KP;
						}
						gv.pid0.kp = kp;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_PID0_KI:{
					float ki = 0.0;
					if(flash_read_float(address, &ki) == 0){
						if(ki<0.0){
							ki = CONF_DEF_PID0_KI;
						}
						gv.pid0.ki = ki;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_PID0_KD:{
					float kd = 0.0;
					if(flash_read_float(address, &kd) == 0){
						if(kd<0.0){
							kd =CONF_DEF_PID0_KD;
						}
						gv.pid0.kp = kd;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_PID0_I_OUT_LIMIT:{
					float iLimt = 0.0;
					if(flash_read_float(address, &iLimt) == 0){
						if(!(iLimt>=0.00 && iLimt <= 100.00)){
							iLimt = CONF_DEF_PID0_I_OUT_MAX;
						}
						gv.pid0.kp = iLimt;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}

				break;

				/*PID 1---------------------*/
//			case FLS_ADDR_PID1_SETPOINT :{
//					float setpoint = 0.0;
//					if(flash_read_float(address, &setpoint) == 0){
//						if(!(setpoint >= 0.0 && setpoint <= 100.0)){
//							setpoint = CONF_DEF_PID1_SETPOINT;
//						}
//						gv.motoCtrlStpr.setPos = setpoint;
//						gv.pid1.setpoint = setpoint;
//					}else{
//						dbg_print("Flash: Read float Failed! (Address=%d)",address);
//					}
//				}
//				break;
			case FLS_ADDR_PID1_KP:{
					float kp = 0.0;
					if(flash_read_float(address, &kp) == 0){
						if(kp<0.0){
							kp =CONF_DEF_PID1_KP;
						}
						gv.pid1.kp = kp;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_PID1_KI:{
					float ki = 0.0;
					if(flash_read_float(address, &ki) == 0){
						if(ki<0.0){
							ki = CONF_DEF_PID1_KI;
						}
						gv.pid1.ki = ki;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_PID1_KD:{
					float kd = 0.0;
					if(flash_read_float(address, &kd) == 0){
						if(kd<0.0){
							kd =CONF_DEF_PID1_KD;
						}
						gv.pid1.kp = kd;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}
				break;
			case FLS_ADDR_PID1_I_OUT_LIMIT:{
					float iLimt = 0.0;
					if(flash_read_float(address, &iLimt) == 0){
						if(!(iLimt>=0.00 && iLimt <= 100.00)){
							iLimt = CONF_DEF_PID1_I_OUT_MAX;
						}
						gv.pid1.kp = iLimt;
					}else{
						dbg_print("Flash: Read float Failed! (Address=%d)",address);
					}
				}

				break;

				/*Modbus TCP Net info--------*/
			case FLS_ADDR_MB_TCPS_PORT :
				if(!(value <= UINT16_MAX)){
					value = CONF_DEF_MB_TCPS_PORT;
				}
				gv.mbtcps.port = value;
				break;
			case FLS_ADDR_MB_TCPS_MAC1:
				for( uint8_t i = 0; i < 6; i++){
					flash_read(address+i, &value);
					if(!(value >= 0 && value <= UINT8_MAX)){
						uint8_t data[6] = CONF_DEF_MB_TCPS_MAC;
						memcpy(gv.mbtcps.netif.mac, data, 6);
						address = FLS_ADDR_MB_TCPS_MAC6;
						break;
					}else{
						gv.mbtcps.netif.mac[i] = (uint8_t)value;// (uint8_t)FSExt_ReadIntNum(address+i);
					}
				}
				address = FLS_ADDR_MB_TCPS_MAC6;
				break;
//			case FLS_ADDR_MB_TCPS_MAC2:
//
//				break;
//			case FLS_ADDR_MB_TCPS_MAC3:
//				break;
//			case FLS_ADDR_MB_TCPS_MAC4:
//				break;
//			case FLS_ADDR_MB_TCPS_MAC5:
//				break;
//			case FLS_ADDR_MB_TCPS_MAC6:
//				break;

			case FLS_ADDR_MB_TCPS_IP1:
				for( uint8_t i = 0; i < 4; i++){
					flash_read(address+i, &value);
					if(!(value >= 0 && value <= UINT8_MAX)){
						uint8_t data[4] = CONF_DEF_MB_TCPS_IP;
						memcpy(gv.mbtcps.netif.ip, data, 4);
						address = FLS_ADDR_MB_TCPS_IP4;
						break;
					}else{
						gv.mbtcps.netif.ip[i] = (uint8_t)value;// (uint8_t)FSExt_ReadIntNum(address+i);
					}
				}
				address = FLS_ADDR_MB_TCPS_IP4;
				//dbg_print("%d",gv.mbtcps.netif.ip[0]);
				break;
//			case FLS_ADDR_MB_TCPS_IP2:
//				break;
//			case FLS_ADDR_MB_TCPS_IP3:
//				break;
//			case FLS_ADDR_MB_TCPS_IP4:
//				break;

			case FLS_ADDR_MB_TCPS_SN1:
				for( uint8_t i = 0; i < 4; i++){
					flash_read(address+i, &value);
					if(!(value >= 0 && value <= UINT8_MAX)){
						uint8_t data[4] = CONF_DEF_MB_TCPS_SN;
						memcpy(gv.mbtcps.netif.sn, data, 4);
						address = FLS_ADDR_MB_TCPS_SN4;
						break;
					}else{
						gv.mbtcps.netif.sn[i] = (uint8_t)value;// (uint8_t)FSExt_ReadIntNum(address+i);
					}
				}
				address = FLS_ADDR_MB_TCPS_SN4;
				break;
//			case FLS_ADDR_MB_TCPS_SN2:
//				break;
//			case FLS_ADDR_MB_TCPS_SN3:
//				break;
//			case FLS_ADDR_MB_TCPS_SN4:
//				break;

			case FLS_ADDR_MB_TCPS_GW1:
				for( uint8_t i = 0; i < 4; i++){
					flash_read(address+i, &value);
					if(!(value >= 0 && value <= UINT8_MAX)){
						uint8_t data[4] = CONF_DEF_MB_TCPS_GW;
						memcpy(gv.mbtcps.netif.gw, data, 4);
						address = FLS_ADDR_MB_TCPS_GW4;
						break;
					}else{
						gv.mbtcps.netif.gw[i] = (uint8_t)value;// (uint8_t)FSExt_ReadIntNum(address+i);
					}
				}
				address = FLS_ADDR_MB_TCPS_GW4;
				break;
//			case FLS_ADDR_MB_TCPS_GW2:
//				break;
//			case FLS_ADDR_MB_TCPS_GW3:
//				break;
//			case FLS_ADDR_MB_TCPS_GW4:
//				break;

			case FLS_ADDR_MB_TCPS_APPLY_CHANGE:
				if(!(value <= UINT8_MAX)){
					value = CONF_DEF_MB_TCPS_APPLY_CHANGE;
					}
				gv.mbtcps.applyChange = value;
//				modbus_update_netIfs();
				break;


			case FLS_ADDR_MB_RTU_BAUD:
				//CONF_DEF_MB_RTU_BAUD;
				flash_read(FLS_ADDR_MB_RTU_BAUD, &value);
				if(value!=0 && value < UINT32_MAX){
					gv.mbrtu.serial.baudRate = value;
				}else{
					gv.mbrtu.serial.baudRate = CONF_DEF_MB_RTU_BAUD;
				}
				break;

			case FLS_ADDR_MB_RTU_APPLY_CHANGE:
				if(!(value <= UINT8_MAX)){
					value = CONF_DEF_MB_RTU_APPLY_CHANGE;
				}
				   gv.mbtcps.applyChange = value;

			break;



				/*Utilities--------------*/
			case FLS_ADDR_DBUG_ENABLE:
				if(!(value == 0 || value == 1)){
					value  = CONF_DEBUG_ENABLE;
				}
				gv.debugEnabe = value;
				break;
			default:
				break;
		}
	}
}
