/*
 * IRQ_Handler.c
 *
 *  Created on: Dec 5, 2024
 *      Author: RusselPC
 */

#include <stm32f4xx_it.h>
#include <stdint.h>
#include "main.h"
#include "app_main.h"
#include "Encoder.h"
#include "debug.h"
#include "BTS7960.h"
#include "TimeStamp/TimeStamp.h"
#include "modbus_tcp_manager.h"
#include "uart_callback.h"

extern GlobalVar_ts gv;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart2;

/*IRQ For GPIOs---------------------------*/
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	/*Encoder 1  index z interrupt*/
//	dbg_print("IRQ\r\n");
	if(GPIO_Pin == ENC1_CHZ_IT15_Pin){
		uint32_t count =  gv.enc1.count;//__HAL_TIM_GET_COUNTER(&htim2);//
		Encoder_IndexZCapture(&gv.enc1);

		gv.motoCtrlStpr.downSen.isTriggered = 1;
//		gv.smd.downSenTrig = 1;
		dbg_print("LowerLimit: IRQ, COunt:%u\r\n",count);
//		DM542TStpDrv_SetRPM(&gv.smd.drv, 0);
	}if(GPIO_Pin == STP_M_LIMIT_SW_IT10_Pin){

		gv.motoCtrlStpr.upSen.isTriggered = 1;
		uint32_t count =  gv.enc1.count;//__HAL_TIM_GET_COUNTER(&htim2);//gv.enc1.count;//
		//		gv.smd.upSenTrig = 1;
		dbg_print("UpperLmit: IRQ, COunt:%u\r\n",count);

	}
	handle_exti_Callback(GPIO_Pin);

}


/*IRQ For Timers---------------------------*/
/**
 * @brief Callback function for Input Capture interrupt.
 *
 * This function is called when a capture event occurs on the input capture channel of TIM2. It reads
 * the current timer counter and processes it using the `Encoder_ChACapture` function.
 *
 * @param htim Pointer to the TIM_HandleTypeDef structure.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim2){

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)	{

			uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
			Encoder_ChACapture(&gv.enc1, count);
//			dbg_print("Enc:%ld\r\n", count);
		}
	}
}

/**
  * @brief  Period elapsed callback
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim14){
		TS_CatchOVF();
//		HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin);
	}
}





/*IRQ For UARTs---------------------------*/
/**
 * @brief Callback function for UART receive event.
 *
 * This function is called when data is received over UART (via `huart2`). It echoes the received data
 * back to the sender, processes the received command, and performs corresponding actions based on the command.
 *
 * Commands supported:
 * - 'e': Enable or disable the stepper motor driver.
 * - 's': Start or stop the stepper motor driver.
 * - 'd': Set the direction of the stepper motor driver.
 * - 'r': Set the speed of the stepper motor driver.
 * - 'p': Set the position of the stepper motor driver.
 *
 * @param huart Pointer to the UART_HandleTypeDef structure.
 * @param Size  The size of the received data.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == &huart2 && Size>0){
		dbg_print("\r\nEcho: ");
		for(uint8_t i = 0; i < Size; i++){
			dbg_print("%c",gv.dbgRxBuf[i]);
		}
		dbg_print("\r\n");
		char cmd = gv.dbgRxBuf[0];
		uint16_t value = 0xFFFF;
		switch (cmd) {
			case 'e':
				if(gv.dbgRxBuf[1] == '0'){
					gv.motoCtrlStpr.enable = 0;
//					SMD_Disable(&gv.smd);
//					DM542T_Disable(&gv.smd.drv);
				}else{
					gv.motoCtrlStpr.enable = 1;
//					SMD_Enable(&gv.smd);
//					DM542T_Enable(&gv.smd.drv);
				}
				break;
			case 's':
				if(gv.dbgRxBuf[1] == '0'){
					gv.motoCtrlStpr.start = 0;
//					SMD_Stop(&gv.smd);
//					BTS7960_Stop(&gv.bts7960);
//					DM542T_StopPulse(&gv.smd.drv);
					dbg_print("stop\r\n");
					gv.start = 0;
				}else{
					gv.motoCtrlStpr.start = 1;
//					SMD_Start(&gv.smd);
//					BTS7960_Start(&gv.bts7960);
//					DM542T_StartPulse(&gv.smd.drv);
					dbg_print("start\r\n");
					gv.start = 1;
				}
				break;
			case 'd':
				if(gv.dbgRxBuf[1] == 'u'){
					gv.motoCtrlStpr.dir = MOTOR_CTRL_DIR_UP;

//					SMD_SetDir(&gv.smd, SMD_DIR_DOWN);
//					BTS7960_SetDir(&gv.bts7960, BTS7960_DIR_RIGHT);
//					DM542T_SetDir(&gv.smd.drv, DM542T_DIR_CW);
					dbg_print("up\r\n");
					gv.dir = 0;
				}else if(gv.dbgRxBuf[1] == 'd'){
					gv.motoCtrlStpr.dir = MOTOR_CTRL_DIR_DOWN;
//					SMD_SetDir(&gv.smd, SMD_DIR_UP);
//					BTS7960_SetDir(&gv.bts7960, BTS7960_DIR_LEFT);
//					DM542T_SetDir(&gv.smd.drv, DM542T_DIR_CCW);
					dbg_print("down\r\n");
					gv.dir = 1;
				}
				break;
			case 'r':
			case 'p':
				if(Size == 2){
					value = gv.dbgRxBuf[1]&0x0F;
				}else if(Size == 3){
					value = (gv.dbgRxBuf[1]&0x0F)*10 + (gv.dbgRxBuf[2]&0x0F);
				}else if(Size == 4){
					value = (gv.dbgRxBuf[1]&0x0F)*100 + (gv.dbgRxBuf[2]&0x0F)*10 + (gv.dbgRxBuf[3]&0x0F);
				}
//				dbg_print("size: %d\r\n", Size);
				if(value == 0xFFFF){
					break;
				}

				if(cmd == 'r'){
					gv.motoCtrlStpr.speed = value;
//					SMD_SetSpeed(&gv.smd, value);
//					BTS7960_SetDuty(&gv.bts7960, value);
//					DM542T_SetRPM(&gv.smd.drv, value);
//					gv.speed = value;
//					dbg_print("speed: %d\r\n", gv.motoCtrlStpr.speed);
				}else{
					if(value <= gv.motoCtrlStpr.maxPos ){
						gv.motoCtrlStpr.setPos = value;
						gv.pid1.setpoint = value;
					}else{
						dbg_print("Position shouldn't be greater than %d !\r\n", gv.smd.maxPos );
					}

				}
				dbg_print("value: %d\r\n",value);
				break;
			default:
				dbg_print("Invalid command!\r\n");
				break;
		}


		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, gv.dbgRxBuf, DBG_RX_BUFF_SIZE);
	}
	Modbus_UARTE_RxEventCallback(huart, Size);

}
