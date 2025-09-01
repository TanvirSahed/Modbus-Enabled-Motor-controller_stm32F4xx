/*
 * MotorController.c
 *
 *  Created on: Mar 4, 2025
 *      Author: wsrra
 */
#include "MotorController.h"
#include "debug.h"
#include <stdio.h>
#include "MotorController_Config.h"

#define TRACE_ENABLE 		DEBUG_ENABLE


static void ErrorHandler(MotorController_ts *mc, MotorCtrl_Error_te error);

void processEvent(MotorController_ts *mc);
void setNextState(MotorController_ts *mc, MotorCtrl_State_te state);
void setFault(MotorController_ts *mc,  MotorCtrl_Fault_te fault);
void clearFault(MotorController_ts *mc);

void actOnEnable(MotorController_ts *mc);
void actOnDisable(MotorController_ts *mc);
void actOnStop(MotorController_ts *mc);
void actOnStart(MotorController_ts *mc);
void actOnDownDirFromGoingUp(MotorController_ts *mc);
void actOnUpDirFromGoingDowm(MotorController_ts *mc);
void actOnDir(MotorController_ts *mc);
void actOnStartUpAndDownDir(MotorController_ts *mc);
void actOnWitingTimeout(MotorController_ts *mc);
void actOnDownSen(MotorController_ts *mc);
void actOnUpSen(MotorController_ts *mc);
void actOnPosMatched(MotorController_ts *mc);
void actOnUpAndDownSenNotTrigd(MotorController_ts *mc);
void actOnFault(MotorController_ts *mc);
void actOnFaultClear(MotorController_ts *mc);


void printInfo(MotorController_ts *mc);

MotorCtrl_Error_te MotorCtrl_Init(MotorController_ts *mc){
    if (mc == NULL){
    	dbg_print("MC: Init failed.\r\n");
    	ErrorHandler(mc, MOTOR_CTRL_ERR_NULL_PTR);
    }

}

MotorCtrl_Error_te MotorCtrl_MoveToPos(MotorController_ts *mc, uint16_t pos,  //TODO: to add two params named set and cur
		float speed, uint8_t start)
{
	/*Is enabled*/
	if(!mc->enable || mc->isStartUp){
		return;
	}
//	if(mc->path == MOTOR_CTRL_PATH_REVERSE){
//		mc->setPos = mc->maxPos -
//	}

	if(mc->setPos == mc->setPosLast){return;}

	mc->setPosLast = mc->setPos;

	/*Ditect direction*/
	if(mc->pos > mc->setPos){
		/*Down direction*/
		mc->dir = MOTOR_CTRL_DIR_DOWN;
	}else if(mc->pos < mc->setPos){
		/*up direction*/
		mc->dir = MOTOR_CTRL_DIR_UP;
	}else{
		return;
	}

	/*Set position*/


	/*start*/
	mc->start = start;

}

MotorCtrl_Error_te MotorCtrl_SetMotorDriverInterface(MotorController_ts *mc,
		MotorCtrl_DriverInterface_ts *drv)
{
	if (mc == NULL || drv == NULL){
		dbg_print("MC: Set motor driver interface failed!\r\n");
		ErrorHandler(mc, MOTOR_CTRL_ERR_NULL_PTR);
		return MOTOR_CTRL_ERR_NULL_PTR;
	}

	mc->motorIf = drv;
	return MOTOR_CTRL_ERR_OK;
}

MotorCtrl_Error_te MotorCtrl_SetLimitSensorInterface(MotorController_ts *mc,
		MotorCtrl_LimitSenInterface_ts *sen)
{
	if (mc == NULL || sen == NULL){
		dbg_print("MC: Set sensor interface failed!\r\n");
		ErrorHandler(mc, MOTOR_CTRL_ERR_NULL_PTR);
	}

	mc->limitSenIf = sen;

}

MotorCtrl_Error_te MotorCtrl_AttachCallback(MotorController_ts *mc,
		MotorCtrl_Callback_ts *callback)
{
	if (mc == NULL || callback == NULL){
		dbg_print("MC: Attach callback failed!\r\n");
		ErrorHandler(mc, MOTOR_CTRL_ERR_NULL_PTR);
	}

	mc->callback = *callback;
}

// State machine function
MotorCtrl_Error_te MotorCtrl_Run(MotorController_ts *mc) {
    if (mc == NULL){
        ErrorHandler(mc, MOTOR_CTRL_ERR_NULL_PTR);
    }

    processEvent(mc);

    switch (mc->sm.state) {

        case MOTOR_CTRL_SM_STATE_INACTIVE_0:
            /* Motor is disabled, waiting for start command */
        	if(mc->enable){
        		/*T0: st 0 -> 1*/
        		printInfo(mc);
        		actOnEnable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_START_UP_1);
        	}

            break;

        case MOTOR_CTRL_SM_STATE_START_UP_1:
            /* Check motor position before moving*/
        	if(!mc->enable){
        		/*T1: st 1 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);

        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T5: st 1 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if(mc->downSen.isTriggered || mc->downSen.state){
        		/*T2: st 1 -> 3*/
        		printInfo(mc);
        		actOnDownSen(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_DOWN_POS_3);
        	}else if(mc->upSen.isTriggered || mc->upSen.state){
        		/*T4: st 1 -> 5*/
        		printInfo(mc);
        		actOnUpSen(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_UP_POS_5);

        	}else if(!mc->downSen.isTriggered && !mc->downSen.state &&
        			 !mc->upSen.isTriggered	&& !mc->upSen.state)
        	{
        		/*T4: st 1 -> 4*/
        		printInfo(mc);
        		actOnUpAndDownSenNotTrigd(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_UP_4);

        	}

            break;

        case MOTOR_CTRL_SM_STATE_GOING_DOWN_2:
        	/* Move motor down*/
        	if(!mc->enable){
        		/*T6: st 2 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T10: st 2 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if(mc->downSen.isTriggered || mc->downSen.state){
        		/*T7: st 2 -> 3*/
        		printInfo(mc);
        		actOnDownSen(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_DOWN_POS_3);
        	}else if(mc->dir == MOTOR_CTRL_DIR_UP && !mc->isStartUp){
        		/*T9: st 2 -> 6*/
        		printInfo(mc);
        		actOnUpDirFromGoingDowm(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_WAITING_6);
        	}else if(mc->pos <= mc->setPos && mc->setPos > 0.0){
        		/*T8: st 2 ->8 */
        		printInfo(mc);
        		actOnPosMatched(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_MID_POS_8);
        	}else if(!mc->start){
        		/*T29: st 2 ->8 */
        		printInfo(mc);
        		actOnStop(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_MID_POS_8);
        	}
            break;

        case MOTOR_CTRL_SM_STATE_DOWN_POS_3:
        	/* Motor reached the down position*/
        	 if(mc->isLearnOpenPos){
				mc->isLearnOpenPos = 0;
				mc->isStartUp = 0;
				dbg_print("learnStop\r\n");
			}else if(mc->isStartUp){

        		mc->isLearnOpenPos = 1;
        		mc->dir = MOTOR_CTRL_DIR_UP;
        		mc->speed = mc->learningSpeed;
        		dbg_print("learnStart\r\n");
        	}

        	if(!mc->enable){
        		/*T11: st 3 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T13: st 3 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if(mc->dir == MOTOR_CTRL_DIR_UP &&
        			 (mc->speed > 0.0F && mc->speed <= mc->maxSpeed))
        	{
        		/*T12: st 3 -> 4*/
        		printInfo(mc);
        		actOnDir(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_UP_4);
        	}
            break;

        case MOTOR_CTRL_SM_STATE_GOING_UP_4:
        {
        	uint8_t isStartUp = mc->isStartUp;

        	/* Move motor up*/
        	if(!mc->enable){
        		/*T14: st 4 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T19: st 4 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if((mc->downSen.isTriggered || mc->downSen.state)){
        		if(isStartUp && !mc->isLearnOpenPos){
				/*T18: st 4 -> 3*/
					printInfo(mc);
					actOnDownSen(mc);
					setNextState(mc, MOTOR_CTRL_SM_STATE_DOWN_POS_3);
        		}else{
        			/*False trigger, So clear the flags*/
        			mc->downSen.isTriggered = 0;
        			mc->downSen.state = 0;
        		}
				break;
			}else if(mc->upSen.isTriggered || mc->upSen.state){
				/*T15: st 4 -> 5*/
				printInfo(mc);
				actOnUpSen(mc);
				setNextState(mc, MOTOR_CTRL_SM_STATE_UP_POS_5);
				break;
			}
			if(isStartUp){
				break;
			}
			/* need to block the following condition on start up */
        	if(mc->dir == MOTOR_CTRL_DIR_DOWN){
        		/*T9: st 4 -> 6*/
        		printInfo(mc);
        		actOnDownDirFromGoingUp(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_WAITING_6);
        	}else if(mc->pos >= mc->setPos && mc->setPos > 0.0 &&
        			mc->setPos < mc->maxPos){
        		/*T8: st 4 ->8 */
        		printInfo(mc);
        		actOnPosMatched(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_MID_POS_8);
        	}else if(!mc->start){
        		/*T29: st 4 ->8 */
        		printInfo(mc);
        		actOnStop(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_MID_POS_8);
        	}

        }break;

        case MOTOR_CTRL_SM_STATE_UP_POS_5:
            /* Motor reached the up position*/

        	if(mc->isLearnOpenPos){
//        		mc->maxPos = mc->pos;
        		dbg_print("learnPOs:%0.2f\r\n",mc->maxPos);
        	}

        	if(!mc->enable){
        		/*T20: st 5 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T22: st 5 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if(mc->dir == MOTOR_CTRL_DIR_DOWN){
        		printInfo(mc);
        		if(mc->isStartUp){
        			actOnStartUpAndDownDir(mc);
					setNextState(mc, MOTOR_CTRL_SM_STATE_WAITING_6);
        		}else if(mc->speed > 0.0F && mc->speed <= mc->maxSpeed){
        		/*T21: st 5 -> 2*/
					actOnDir(mc);
					setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_DOWN_2);
        		}

        	}
            break;


        case MOTOR_CTRL_SM_STATE_WAITING_6:
            /* If direction changes, stop and wait before reversing*/
        	if(!mc->enable){
        		/*T23: st 6 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T26: st 6 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if(Timer_IsTimeout(&mc->timer)){
        		printInfo(mc);
        		actOnWitingTimeout(mc);
        		if(mc->dir == MOTOR_CTRL_DIR_DOWN){
        			/*T24: st 6 -> 7*/
            		setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_DOWN_2);
        		}else if(mc->dir == MOTOR_CTRL_DIR_UP){
        			/*T25: st 6 -> 7*/
            		setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_UP_4);
        		}else{
        			ErrorHandler(mc, MOTOR_CTRL_ERR_INVALID_DIR);
        		}
        	}
            break;
        case MOTOR_CTRL_SM_STATE_FAULT_7:
            /* Handle faults*/
        	if(!mc->enable){
        		/*T27: st 7 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->faultClear){
        		/*T26: st 7 -> 1*/
        		printInfo(mc);
        		actOnFaultClear(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_START_UP_1);
        	}
            break;
        case MOTOR_CTRL_SM_STATE_MID_POS_8:
            /* Motor in intermediate position*/
        	if(!mc->enable){
        		/*T31: st 8 -> 0*/
        		printInfo(mc);
        		actOnDisable(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_INACTIVE_0);
        	}else if(mc->fault != MOTOR_CTRL_FAULT_OK){
        		/*T34: st 8 -> 7*/
        		printInfo(mc);
        		actOnFault(mc);
        		setNextState(mc, MOTOR_CTRL_SM_STATE_FAULT_7);
        	}else if(mc->start){
        		printInfo(mc);
        		actOnStart(mc);
        		if(mc->dir == MOTOR_CTRL_DIR_DOWN){
        			/*T32: st 8 -> 7*/
        			setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_DOWN_2);
        		}else if(mc->dir == MOTOR_CTRL_DIR_UP){
        			/*T33: st 8 -> 7*/
        			setNextState(mc, MOTOR_CTRL_SM_STATE_GOING_UP_4);
        		}else{
        			ErrorHandler(mc, MOTOR_CTRL_ERR_INVALID_DIR);
        		}
        	}
            break;
        default:
            mc->sm.state = MOTOR_CTRL_SM_STATE_INACTIVE_0;
            break;
    }
}


void processEvent(MotorController_ts *mc){
	if(!mc->enable || mc->fault != MOTOR_CTRL_FAULT_OK) return;

	/*Read Down Sensor*/
	if(mc->limitSenIf->readDownSen != NULL){
		if(mc->limitSenIf->readDownSen(&mc->downSen) != MOTOR_CTRL_ERR_OK){
			setFault(mc, MOTOR_CTRL_FAULT_ON_READ_DOWN_SEN);
		}
	}

	/*Read Up Sensor*/
	if(mc->limitSenIf->readUpSen != NULL){
		if(mc->limitSenIf->readUpSen(&mc->upSen) != MOTOR_CTRL_ERR_OK){
			setFault(mc, MOTOR_CTRL_FAULT_ON_READ_UP_SEN);
		}
	}

	/*If the speed is changed*/
	if(mc->speed != mc->lastSpeed){
		/*2. Set speed*/
		if (mc->motorIf->setSpeed != NULL){
	    	if(mc->motorIf->setSpeed(mc->speed) != MOTOR_CTRL_ERR_OK){
	    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_SPEED);
	    	}
	    	mc->lastSpeed = mc->speed;
		}
	}
}


void printInfo(MotorController_ts *mc){
	dbg_print("MC:: {ST:%d, startUp:%d, EN:%d, Start:%d, "
			"Dir:%d, speed:%0.2f, pos:%0.2f, setPos:%0.2f, maxPos:%0.2f,"
			"downSen:{state:%d, isTrigd:%d}, upSen:{state:%d, isTrigd:%d}}\r\n",
			mc->sm.state, mc->isStartUp, mc->enable, mc->start,
			mc->dir, mc->speed, mc->pos, mc->setPos, mc->maxPos,
			mc->downSen.state, mc->downSen.isTriggered,
			mc->upSen.state, mc->upSen.isTriggered,
			mc->fault, mc->faultClear);
}

void setNextState(MotorController_ts *mc, MotorCtrl_State_te state){
	dbg_print("MC: ST %d ->%d\r\n",mc->sm.state, state);
	mc->sm.state = state;
}

void setFault(MotorController_ts *mc,  MotorCtrl_Fault_te fault){
	mc->fault = fault;
}

void clearFault(MotorController_ts *mc){
	mc->fault = MOTOR_CTRL_FAULT_OK;
}



/*Action on event--------------*/
/*Action on enable event*/
void actOnEnable(MotorController_ts *mc){
	mc->isStartUp = 1;
	mc->dir = MOTOR_CTRL_DIR_NONE;
//	mc->speed = 0;
	mc->start = 0;
    if (mc->motorIf->enable != NULL){
    	if(mc->motorIf->enable() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_ENABLE);
    	}
    }
}


/*Action on disable event*/
void actOnDisable(MotorController_ts *mc){
	mc->isStartUp = 0;
	clearFault(mc);
    if (mc->motorIf->disable != NULL){
    	if(mc->motorIf->disable() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_DISABLE);
    	}
    }
}

/*Action on stop event*/
void actOnStop(MotorController_ts *mc){
    if (mc->motorIf->stop != NULL){
    	if(mc->motorIf->stop() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
    	}
    }
}

/*Action on disable event*/
void actOnStart(MotorController_ts *mc){

	/*1. Set the direction*/
	if (mc->motorIf->setDir != NULL){

		MotorCtrl_Dir_te dir;
		if((mc->path==MOTOR_CTRL_PATH_NORMAL) || (mc->dir == MOTOR_CTRL_DIR_NONE)){
			dir = mc->dir;
		}else{

			dir = ((mc->dir== MOTOR_CTRL_DIR_DOWN)?MOTOR_CTRL_DIR_UP:MOTOR_CTRL_DIR_DOWN);
		}

    	if(mc->motorIf->setDir(dir) != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR);
    		return;
    	}
	}

    if (mc->motorIf->start != NULL){
    	if(mc->motorIf->start() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_START);
    	}
    }
}

/*Action on up direction change event*/
void actOnDownDirFromGoingUp(MotorController_ts *mc){
	/* T9: ST 2 -> 6
	 *  Stop the motor
	 *  start the waiting timer
	 * */


	/*Stop the motor*/
    if (mc->motorIf->stop != NULL){
    	if(mc->motorIf->stop() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
    		return;
    	}
    }

    if( mc->sm.state == MOTOR_CTRL_SM_STATE_GOING_DOWN_2 ||
    	mc->sm.state == MOTOR_CTRL_SM_STATE_GOING_UP_4	)
    {
		/*start the waiting timer*/
		Timer_Start(&mc->timer);
    }

}

/*Action on up direction change event*/
void actOnUpDirFromGoingDowm(MotorController_ts *mc){
	/* T9: ST 2 -> 6
	 *  Stop the motor
	 *  start the waiting timer
	 * */

	mc->downSen.isTriggered = 0;
	mc->downSen.state = 0;
	mc->upSen.isTriggered = 0;
	mc->upSen.state = 0;

	/*Stop the motor*/
    if (mc->motorIf->stop != NULL){
    	if(mc->motorIf->stop() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
    		return;
    	}
    }

    if( mc->sm.state == MOTOR_CTRL_SM_STATE_GOING_DOWN_2 ||
    	mc->sm.state == MOTOR_CTRL_SM_STATE_GOING_UP_4	)
    {
		/*start the waiting timer*/
		Timer_Start(&mc->timer);
    }

}

/*Action on down direction change event*/
void actOnDir(MotorController_ts *mc){
	/* T9: ST 2 -> 6
	 *  start the motor
	 *  start the waiting timer
	 * */

	mc->downSen.isTriggered = 0;
	mc->downSen.state = 0;
	mc->upSen.isTriggered = 0;
	mc->upSen.state = 0;

	/*1. Set the direction*/
	if (mc->motorIf->setDir != NULL){

		MotorCtrl_Dir_te dir;
				if((mc->path==MOTOR_CTRL_PATH_NORMAL) || (mc->dir == MOTOR_CTRL_DIR_NONE)){
					dir = mc->dir;
				}else{
					dir = ((mc->dir== MOTOR_CTRL_DIR_DOWN)?MOTOR_CTRL_DIR_UP:MOTOR_CTRL_DIR_DOWN);
				}


    	if(mc->motorIf->setDir(dir) != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR);
    		return;
    	}
	}

	/*start the motor*/
    if (mc->motorIf->start != NULL){

    	if(mc->motorIf->start() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_START);
    		return;
    	}
    	mc->start = 1;
    }
}


/*Action on up direction change event*/
void actOnStartUpAndDownDir(MotorController_ts *mc){
	/* T21
	 *  start the motor
	 *  start the waiting timer
	 * */

	/*1. Set the direction*/
	if (mc->motorIf->setDir != NULL){

		MotorCtrl_Dir_te dir;
				if((mc->path==MOTOR_CTRL_PATH_NORMAL) || (mc->dir == MOTOR_CTRL_DIR_NONE)){
					dir = mc->dir;
				}else{
					dir = ((mc->dir== MOTOR_CTRL_DIR_DOWN)?MOTOR_CTRL_DIR_UP:MOTOR_CTRL_DIR_DOWN);
				}

    	if(mc->motorIf->setDir(dir) != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR);
    		return;
    	}
	}

	/*start the waiting timer*/
	Timer_Start(&mc->timer);
}



/*Action on up direction change event*/
void actOnWitingTimeout(MotorController_ts *mc){
	/* T9: ST 2 -> 6
	 *  Stop the motor
	 *  start the waiting timer
	 * */


	/*Stop the waiting timer*/
	Timer_Stop(&mc->timer);

	/*1. Set the direction*/
	if (mc->motorIf->setDir != NULL){

		MotorCtrl_Dir_te dir;
				if((mc->path==MOTOR_CTRL_PATH_NORMAL) || (mc->dir == MOTOR_CTRL_DIR_NONE)){
					dir = mc->dir;
				}else{
					dir = ((mc->dir== MOTOR_CTRL_DIR_DOWN)?MOTOR_CTRL_DIR_UP:MOTOR_CTRL_DIR_DOWN);
				}


    	if(mc->motorIf->setDir(dir) != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR);
    		return;
    	}
	}

	/*Start the motor*/
    if (mc->motorIf->start != NULL){

    	if(mc->motorIf->start() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_START);
    		return;
    	}
    	mc->start = 1;
    }




}

/*Action on down sensor trigger event*/
void actOnDownSen(MotorController_ts *mc){
//	mc->isStartUp = 0;
	mc->downSen.isTriggered = 0;
	mc->downSen.state = 0;
	mc->upSen.isTriggered = 0;
	mc->upSen.state = 0;
	mc->dir = MOTOR_CTRL_DIR_NONE;
	/*Stop the motor*/
    if (mc->motorIf->stop != NULL){
    	if(mc->motorIf->stop() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
    		return;
    	}
    	mc->start = 0;
    }
    if(mc->callback.downPosRechdCB != NULL){
    	mc->callback.downPosRechdCB((void*)mc);
    }
}


/*Action on up sensor trigger event*/
void actOnUpSen(MotorController_ts *mc){
	mc->upSen.isTriggered = 0;
	mc->upSen.state = 0;
	mc->downSen.isTriggered = 0;
	mc->downSen.state = 0;
	/*Stop the motor*/
	if (mc->motorIf->stop != NULL){
		if(mc->motorIf->stop() != MOTOR_CTRL_ERR_OK){
			setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
			return;
		}
		mc->start = 0;
	}
	if(mc->isStartUp){
		mc->dir = MOTOR_CTRL_DIR_DOWN;
		/*1. Set the direction*/
		if (mc->motorIf->setDir != NULL){

			MotorCtrl_Dir_te dir;
					if((mc->path==MOTOR_CTRL_PATH_NORMAL) || (mc->dir == MOTOR_CTRL_DIR_NONE)){
						dir = mc->dir;
					}else{
						dir = ((mc->dir== MOTOR_CTRL_DIR_DOWN)?MOTOR_CTRL_DIR_UP:MOTOR_CTRL_DIR_DOWN);
					}


	    	if(mc->motorIf->setDir(dir) != MOTOR_CTRL_ERR_OK){
	    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR);
	    		return;
	    	}
		}

		/*2. Set speed*/
		if (mc->motorIf->setSpeed != NULL){
	    	if(mc->motorIf->setSpeed(mc->learningSpeed) != MOTOR_CTRL_ERR_OK){
	    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_SPEED);
	    		return;
	    	}
		}
	}

    if(mc->callback.upPosRechdCB != NULL){
    	mc->callback.upPosRechdCB((void*)mc);
    }

}


/*Action on down sensor trigger event*/
void actOnPosMatched(MotorController_ts *mc){

    if (mc->motorIf->stop != NULL){
    	if (mc->motorIf->stop()!= MOTOR_CTRL_ERR_OK){
			setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
			return;
		}
    	mc->start = 0;
    }

    if(mc->callback.midPosRechdCB != NULL){
    	mc->callback.midPosRechdCB((void*)mc);
    }
}

/*Action on up sensor trigger event*/
void actOnUpAndDownSenNotTrigd(MotorController_ts *mc){
	/* T3:
	 * 1. Set the up direction
	 * 2. Set speed
	 * 3. Start the motor
	 * 4. set the start up flag
	 * */

	/*4. set the start up flag*/
	mc->isStartUp = 1;
	/*1. Set the up direction*/
	if (mc->motorIf->setDir != NULL){

		MotorCtrl_Dir_te dir;
				if(mc->path==MOTOR_CTRL_PATH_NORMAL){
					dir = MOTOR_CTRL_DIR_UP;
				}else{
					dir = MOTOR_CTRL_DIR_DOWN;//((mc->dir== MOTOR_CTRL_DIR_DOWN)?MOTOR_CTRL_DIR_UP:MOTOR_CTRL_DIR_DOWN);
				}


    	if(mc->motorIf->setDir(dir) != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_DOWN_DIR);
    		return;
    	}
    	mc->dir = dir;
	}

	/*2. Set speed*/
	if (mc->motorIf->setSpeed != NULL){
    	if(mc->motorIf->setSpeed(mc->learningSpeed) != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_SET_SPEED);
    		return;
    	}
	}

	/*3. Start the motor*/
    if (mc->motorIf->start != NULL){
    	if(mc->motorIf->start() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_START);
    		return;
    	}
    }


}

/*Action on fault event*/
void actOnFault(MotorController_ts *mc){
	mc->isStartUp = 0;
    if (mc->motorIf->stop != NULL){
    	if (mc->motorIf->stop()!= MOTOR_CTRL_ERR_OK){
			setFault(mc, MOTOR_CTRL_FAULT_ON_STOP);
			return;
		}
    	mc->start = 0;
    }

    if (mc->motorIf->disable != NULL){
    	if(mc->motorIf->disable() != MOTOR_CTRL_ERR_OK){
    		setFault(mc, MOTOR_CTRL_FAULT_ON_DISABLE);
    	}
    }
}

/*Action on fault clear event*/
void actOnFaultClear(MotorController_ts *mc){
	mc->isStartUp = 0;
	mc->start = 0;
	mc->dir = MOTOR_CTRL_DIR_NONE;
	mc->fault = 0;
	mc->faultClear = MOTOR_CTRL_FAULT_OK;
}

static void ErrorHandler(MotorController_ts *mc, MotorCtrl_Error_te error){
	dbg_trace(TRACE_ENABLE, "MC Error: ");
	switch (error) {
	    case MOTOR_CTRL_ERR:
	    	dbg_trace(TRACE_ENABLE,"General error occurred.\n");
	        break;

	    case MOTOR_CTRL_ERR_NULL_PTR:
	    	dbg_trace(TRACE_ENABLE, "Null pointer\n");
	        break;

	    case MOTOR_CTRL_ERR_INVALID_VALUE:
	    	dbg_trace(TRACE_ENABLE,"Invalid value\n");
	        break;
	    default:
	        break;
	}
}

