/*
 * BTS7960.h
 *
 *  Created on: Feb 8, 2025
 *      Author: RusselPC
 */

#ifndef SRC_BTS7960_BTS7960_H_
#define SRC_BTS7960_BTS7960_H_
#include <stdint.h>

/*TODO: to set resolution based on timer*/
#define BTS7960_PWM_RESOLUTION 		100.0F // in percent
//#define BTS7960_DUTYCYCLE_OFFSET	5.0F // in percent

/*BTS7960 left, right enable pin status*/
#define BTS7960_LOW 	0
#define BTS7960_HIGH	1

/* BTS7960 error status*/
typedef enum BTS7960_Error{
	BTS7960_ERR_OK = 0,
	BTS7960_ERR_NULL_PTR = -1,
	BTS7960_ERR_INVLD_DIR = -2,
	BTS7960_ERR_INVLD_SPEED = -3,
	BTS7960_ERR_INVLD_VALUE = -4,
	BTS7960_ERR_START_PWM = -5,
	BTS7960_ERR_STOP_PWM = -6,
	BTS7960_ERR_SET_DIR_FAIELD = -7,
	BTS7960_ERR_PWM_CH_CONFIG_FAILED = -8,
}BTS7960_Error_te;

/* BTS7960 error status*/
typedef enum BTS7960_Status{
	BTS7960_STATUS_OK = 0,
	BTS7960_STATUS_ERROR,
	BTS7960_STATUS_DISABLE,
	BTS7960_STATUS_ENABLE,
	BTS7960_STATUS_RUNNING,
	BTS7960_STATUS_BUSY,

}BTS7960_Status_te;



/*BTS7960 Direction*/
typedef enum BTS7960_Direction{
	BTS7960_DIR_NONE = 0,
	BTS7960_DIR_RIGHT,
	BTS7960_DIR_LEFT,
}BTS7960_Direction_te;



/*BTS7960 Handler*/
typedef struct BTS7960{
	uint8_t enable;
	uint8_t start;
	BTS7960_Direction_te dir;
	float rpm;
	float duty;
	float dutyOffset;
	uint32_t period;
	BTS7960_Error_te error;
	BTS7960_Error_te (*init)(struct BTS7960 *drv);
	void (*leftPin)(uint8_t status);
	void (*rightPin)(uint8_t status);
	BTS7960_Error_te (*timerInit)(void);
	BTS7960_Error_te (*startPWM)(BTS7960_Direction_te dir);
	BTS7960_Error_te (*stopPWM)(BTS7960_Direction_te dir);
	BTS7960_Error_te (*setDuty)(struct BTS7960 *drv, float duty);

	void (*delay_us)(uint32_t us);
}BTS7960_ts;


BTS7960_Error_te BTS7960_Init(BTS7960_ts *drv);
BTS7960_Error_te BTS7960_Enable(BTS7960_ts *drv);
BTS7960_Error_te BTS7960_Disable(BTS7960_ts *drv);
BTS7960_Error_te BTS7960_Start(BTS7960_ts *drv);
BTS7960_Error_te BTS7960_Stop(BTS7960_ts *drv);
BTS7960_Error_te BTS7960_SetDir(BTS7960_ts *drv, BTS7960_Direction_te dir);
BTS7960_Error_te BTS7960_SetDuty(BTS7960_ts *drv, float duty);
#endif /* SRC_BTS7960_BTS7960_H_ */
