/*
 * StepperMotor.c
 *
 *  Created on: Dec 17, 2024
 *      Author: RusselPC
 */

#include "StepperMotor.h"
#include "debug.h"
#include "Encoder.h"
#include "app_main.h"
extern GlobalVar_ts gv;

/*SM Private Variable----------------------------------*/

/*SMD Private Variable---------------------------------*/
/*SMD Private Function prototype-----------------------*/
static void onEnable(void *arg);
static void onDisable(void *arg);
static void onDownSenFalse(void *arg);
static void onDownSenTrue(void *arg);
static void onUpSen(void *arg);
static void onDirDown(void *arg);
static void onDirUP(void *arg);
void onDownPosMatched(void *arg);
void onUpPosMatched(void *arg);
static void onFault(void *arg);
static void onFaultClear(void *arg);
static void onUpSenTrue(void *arg);
static void onUpSenTrue(void *arg);


/**
 * @brief State machine transitions for the SMD (Stepper Motor Driver).
 *
 * This array defines the state transitions for the SMD, including the current state, trigger event,
 * associated action, and the next state.
 */
static const SMD_SMTrans_ts smTrnsList[] = {
    /*State Inactive*/
    {SMD_SM_STAT_NONE, SMD_SM_EVENT_NONE, 0, SMD_SM_STAT_INACTIVE},  // T0
    /*State Inactive*/
    {SMD_SM_STAT_INACTIVE, SMD_SM_TRIG_ENABLE, onEnable, SMD_SM_STAT_STARTUP},  // T0

    /*State Startup*/
    {SMD_SM_STAT_STARTUP, SMD_SM_TRIG_FAULT_TRUE, onFault, SMD_SM_STAT_FAULT},
    {SMD_SM_STAT_STARTUP, SMD_SM_TRIG_DISABLE, onDisable, SMD_SM_STAT_INACTIVE},  // T1
    {SMD_SM_STAT_STARTUP, SMD_SM_TRIG_DOWN_SEN_TRUE, onDownSenTrue, SMD_SM_STAT_DOWN_POS},  // T2
    {SMD_SM_STAT_STARTUP, SMD_SM_TRIG_DOWN_SEN_FALSE, onDownSenFalse, SMD_SM_STAT_GOING_UP},  // T
    {SMD_SM_STAT_STARTUP, SMD_SM_TRIG_UP_SEN_TRUE, onUpSenTrue, SMD_SM_STAT_UP_POS},

    /*State GoingDown*/
    {SMD_SM_STAT_STARTUP, SMD_SM_TRIG_FAULT_TRUE, onFault, SMD_SM_STAT_FAULT},
    {SMD_SM_STAT_GOING_DOWN, SMD_SM_TRIG_DISABLE, onDisable, SMD_SM_STAT_INACTIVE},
    {SMD_SM_STAT_GOING_DOWN, SMD_SM_TRIG_DOWN_SEN_TRUE, onDownSenTrue, SMD_SM_STAT_DOWN_POS},
    {SMD_SM_STAT_GOING_DOWN, SMD_SM_TRIG_DOWN_POS_MATCHED, onDownPosMatched, SMD_SM_STAT_DOWN_POS},
    {SMD_SM_STAT_GOING_DOWN, SMD_SM_TRIG_DIR_UP, onDirUP, SMD_SM_STAT_GOING_UP},

    /*State DownPos*/
    {SMD_SM_STAT_DOWN_POS, SMD_SM_TRIG_FAULT_TRUE, onFault, SMD_SM_STAT_FAULT},
    {SMD_SM_STAT_DOWN_POS, SMD_SM_TRIG_DISABLE, onDisable, SMD_SM_STAT_INACTIVE},
    {SMD_SM_STAT_DOWN_POS, SMD_SM_TRIG_DIR_UP, onDirUP, SMD_SM_STAT_GOING_UP},

    /*State GoingUp*/
    {SMD_SM_STAT_GOING_UP, SMD_SM_TRIG_FAULT_TRUE, onFault, SMD_SM_STAT_FAULT},
    {SMD_SM_STAT_GOING_UP, SMD_SM_TRIG_DISABLE, onDisable, SMD_SM_STAT_INACTIVE},
    {SMD_SM_STAT_GOING_UP, SMD_SM_TRIG_UP_SEN_TRUE, onUpSenTrue, SMD_SM_STAT_UP_POS},
    {SMD_SM_STAT_GOING_UP, SMD_SM_TRIG_UP_POS_MATCHED, onUpPosMatched, SMD_SM_STAT_UP_POS},
    {SMD_SM_STAT_GOING_UP, SMD_SM_TRIG_DIR_DOWN, onDirDown, SMD_SM_STAT_GOING_DOWN},
    {SMD_SM_STAT_GOING_UP, SMD_SM_TRIG_DOWN_SEN_TRUE, onDownSenTrue, SMD_SM_STAT_DOWN_POS},

    /*State UpPos*/
    {SMD_SM_STAT_UP_POS, SMD_SM_TRIG_FAULT_TRUE, onFault, SMD_SM_STAT_FAULT},
    {SMD_SM_STAT_UP_POS, SMD_SM_TRIG_DISABLE, onDisable, SMD_SM_STAT_INACTIVE},
    {SMD_SM_STAT_UP_POS, SMD_SM_TRIG_DIR_DOWN, onDirDown, SMD_SM_STAT_GOING_DOWN},

    /*State Fault*/
    {SMD_SM_STAT_FAULT, SMD_SM_TRIG_DISABLE, onDisable, SMD_SM_STAT_INACTIVE},
    {SMD_SM_STAT_FAULT, SMD_SM_TRIG_DISABLE, onFaultClear, SMD_SM_STAT_INACTIVE},
};

/**
 * @brief Enable the stepper motor driver.
 *
 * This function enables and starts the stepper motor driver. The driver is initialized
 * to start and the enable flag is set to true.
 *
 * @param arg A pointer to the stepper motor driver structure.
 */
static void onEnable(void *arg){
    StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
    smd->drv.enable = 1;
    smd->drv.start = 1;
    dbg_print("SMD: Enabled!\r\n");
}

/**
 * @brief Disable the stepper motor driver.
 *
 * This function disables the stepper motor driver. The driver is stopped and the enable flag is cleared.
 *
 * @param arg A pointer to the stepper motor driver structure.
 */
static void onDisable(void *arg){
    StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
    smd->drv.enable = 0;
    smd->drv.start = 0;
    dbg_print("SMD: Disabled!\r\n");
}

/**
 * @brief Handle the condition when the down sensor is false.
 *
 * This function is called when the down sensor indicates a false state during startup.
 * It sets the motor to the upward direction with a small RPM value to start the motor.
 *
 * @param arg A pointer to the stepper motor driver structure.
 */
static void onDownSenFalse(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	if(smd->isStartingUp){
		smd->drv.dir = DM542T_DIR_UP;
		DM542T_SetRPM(&smd->drv, 5);
//		DM542T_SetDir(&smd->drv, DM542T_DIR_UP);
//		DM542T_StartPulse(&smd->drv);
		smd->drv.start = 1;
		dbg_print("SMD: Unknown Pos! [StartUp]\r\n");
	}


}


/**
 * @brief Handles the event when the Down Sensor becomes true during startup.
 *
 * If the system is starting up, the motor's direction is set to down, and the motor starts pulsing.
 * If the system is not starting up, it disables the motor start.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onDownSenTrue(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	if(smd->isStartingUp){
		smd->isStartingUp = 0;
		smd->start = 0;
		smd->drv.start = 0;
		smd->drv.dir = DM542T_DIR_DOWN;
		smd->dir = SMD_DIR_DOWN;
	//	DM542T_StopPulse(&smd->drv);
	//	DM542T_SetDir(&smd->drv, DM542T_DIR_DOWN);
		dbg_print("SMD: Down Pos![Startup]\r\n");
	}else{
		smd->start = 0;
		smd->drv.start = 0;
		dbg_print("SMD: Down Pos!\r\n");
	}

}

/**
 * @brief Handles the event when the Up Sensor becomes false.
 *
 * This function is called when the up sensor is triggered as false. Additional actions can be added.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onUpSenFalse(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;

		//TDOD: write code here
		dbg_print("SMD: Unknown Pos!\r\n");

}

/**
 * @brief Handles the event when the Up Sensor becomes true.
 *
 * If the system is starting up, the motor's direction is set to down and the motor starts pulsing.
 * If the system is not starting up, it disables the motor start.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onUpSenTrue(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	if(smd->isStartingUp){
//		smd->drv.dir = DM542T_DIR_DOWN;
		smd->upSenTrig = 0;
		smd->dir = SMD_DIR_DOWN;
//		DM542T_StopPulse(&smd->drv);
//		DM542T_SetDir(&smd->drv, DM542T_DIR_DOWN);
//		DM542T_StartPulse(&smd->drv);
		dbg_print("SMD: Up Pos![StartsUp]\r\n");
	}else{
		smd->start = 0;
		smd->drv.start = 0;
//		smd->drv.dir = DM542T_DIR_DOWN;
//		DM542T_StopPulse(&smd->drv);
//		DM542T_SetDir(&smd->drv, DM542T_DIR_DOWN);
		dbg_print("SMD: Up Pos!\r\n");
	}



}

/**
 * @brief Sets the motor's direction to down and handles the "Going Down" state.
 *
 * This function is used when the direction is set to go down and handles the relevant state transitions.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onDirDown(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	smd->drv.dir = DM542T_DIR_DOWN;
	smd->downSenTrig = 0;
//	DM542T_StopPulse(&smd->drv);
//	DM542T_SetDir(&smd->drv, DM542T_DIR_DOWN);
//	DM542T_StartPulse(&smd->drv);
	if(smd->isStartingUp){
		dbg_print("SMD: Going Down [StartUp]\r\n");
	}else{

		dbg_print("SMD: Going Down!\r\n");
	}
}

/**
 * @brief Sets the motor's direction to up and handles the "Going Up" state.
 *
 * This function is used when the direction is set to go up and handles the relevant state transitions.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onDirUP(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	smd->drv.dir = DM542T_DIR_UP;
	smd->upSenTrig = 0;
//
//	DM542T_StopPulse(&smd->drv);
//	DM542T_SetDir(&smd->drv, DM542T_DIR_UP);
//	DM542T_StartPulse(&smd->drv);
	dbg_print("SMD: Going Up!\r\n");
}

/**
 * @brief Handles the event when the down position is matched.
 *
 * This function disables the motor start, resets the encoder count, and logs the matched position.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
void onDownPosMatched(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	smd->drv.start = 0;
	smd->start = 0;
//	DM542T_StopPulse(&smd->drv);
//	DM542T_SetDir(&smd->drv, DM542T_DIR_UP);
	smd->drv.start = 0;
	Encoder_SetCount(&gv.enc1, 0);
	dbg_print("SMD: Pos>%d\r\n",smd->pos);
}

/**
 * @brief Handles the event when the up position is matched.
 *
 * This function disables the motor start and logs the matched position.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
void onUpPosMatched(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	if(smd->isStartingUp){
//		smd->drv.dir = DM542T_DIR_DOWN;
		smd->upSenTrig = 0;

//		DM542T_StopPulse(&smd->drv);
//		DM542T_SetDir(&smd->drv, DM542T_DIR_DOWN);
//		DM542T_StartPulse(&smd->drv);
	}else{
		smd->drv.start = 0;
		smd->start = 0;
//		DM542T_StopPulse(&smd->drv);
	}
	dbg_print("SMD: Pos>%d\r\n",smd->pos);
}

/**
 * @brief Handles the event when a fault is triggered.
 *
 * This function disables the motor start and logs the fault.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onFault(void *arg){
	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	smd->drv.start = 0;
	//	DM542T_StopPulse(&smd->drv);
	dbg_print("SMD: Fault!\r\n");
}

/**
 * @brief Clears the fault condition and logs the fault clearance.
 *
 * @param arg A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
static void onFaultClear(void *arg){
//	StpMotDrv_ts *smd = (StpMotDrv_ts*)arg;
	//TDOD: write code here
	dbg_print("SMD: Fault Clear!\r\n");
}

/*SM Public functions----------------------------------*/
/**
 * @brief Moves the stepper motor to the desired position.
 *
 * This function controls the movement of the stepper motor towards the target position (`setPos`).
 * It determines the direction of movement based on the current position (`pos`) and target position (`setPos`).
 * If the motor is enabled and not in startup mode, it sets the motor direction and starts the movement.
 *
 * @param smd A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
void SMD_MoveToPos(StpMotDrv_ts *smd){
	/*Is enabled*/
	if(!smd->enable || smd->isStartingUp == 1){
		return;
	}


	/*Ditect direction*/
	if(smd->pos > smd->setPos){
		/*Down direction*/
		smd->dir = SMD_DIR_DOWN;
	}else if(smd->pos < smd->setPos){
		/*up direction*/
		smd->dir = SMD_DIR_UP;
	}else{

		return;
	}

	/*Set position*/

	/*start*/
	smd->start = 1;

}

/**
 * @brief Sets the trigger for the state machine.
 *
 * This function sets the trigger for the state machine. It ensures that the trigger is within valid bounds.
 *
 * @param sm A pointer to the SMD_SM_ts structure that holds the state machine information.
 * @param trigger The trigger value to set for the state machine.
 */
void SMD_SM_SetTriger(SMD_SM_ts *sm, SMD_SMTrigger_te trigger){
	if(trigger > SMD_SM_TRIG_MAX){return;}
	sm->trigger = trigger;
}

/**
 * @brief Processes the input data for the stepper motor driver state machine.
 *
 * This function processes the input data for the motor's state machine. It checks for changes in position,
 * updates the motor direction, and manages the motor's movement state. It also handles sensor states
 * and adjusts motor speed accordingly.
 *
 * @param smd A pointer to the StpMotDrv_ts structure that holds the motor driver information.
 */
void SMD_SM_ProcessInputData(StpMotDrv_ts *smd){

//	SMD_SetPos(smd, 100);
	SMD_MoveToPos(smd);

	/* Enable
	 * do nothing
	 * */
	/*Start*/
	/*Direction*/


	/*position*/

	if(smd->sm.state == SMD_SM_STAT_DOWN_POS){
		Encoder_SetCount(&gv.enc1, 0);
		gv.enc1.posAngle = 0;
	}

	if(smd->setPos != smd->setPosLast){
		smd->setPosLast = smd->setPos;
		smd->start = 1;
		smd->drv.start = 1;
//		dbg_print("smd: start on pos change\r\n");
	}

	if(smd->start == SMD_SM_STAT_GOING_DOWN){
		if(smd->dir == SMD_DIR_DOWN&& smd->isStartingUp == 0){
			if(smd->pos <= smd->setPos && smd->setPos > 0){
				smd->isPosMatched = 1;
			}
		}
	}else if(smd->sm.state == SMD_SM_STAT_GOING_UP){
		if(smd->dir == SMD_DIR_UP && smd->isStartingUp == 0){
				if(smd->pos >= smd->setPos && smd->setPos > 0){
					smd->isPosMatched = 1;
				}
			}
	}




	/*Reference Speed*/
	DM542T_SetRPM(&smd->drv, smd->inputRpm);

	/*control(PID) Speed*/

	/*Down Sensor*/
	smd->downSenState = smd->downSen();
	/*Up sensor*/
	smd->upSenState = smd->upSen();

}

/**
 * @brief Processes the state machine transitions for the stepper motor driver (SMD).
 *
 * This function handles various states and events for controlling the stepper motor.
 * It processes the state transitions based on the conditions such as enable status,
 * fault status, sensor triggers, and other parameters. The state machine transitions
 * include startup, moving up/down, fault handling, and position matching.
 *
 * @param smd A pointer to the stepper motor driver structure containing the current state
 *            and configuration of the motor driver and associated parameters.
 */
void SMD_SM_ProcessTrigger(StpMotDrv_ts *smd){

	switch (smd->sm.state) {
		case SMD_SM_STAT_NONE:
		break;
		case SMD_SM_STAT_INACTIVE:
			if(smd->enable){
				/*T0*/
//				dbg_print("st: %d to %d\r\n",smd->sm.state,SMD_SM_STAT_STARTUP);
				smd->drv.enable = 1;
				smd->drv.start = 0;
				smd->sm.state = SMD_SM_STAT_STARTUP;

			}
		break;
		case SMD_SM_STAT_STARTUP:
			if(!smd->enable){
				/*T1*/
				smd->drv.enable = 0;
				smd->drv.start = 0;
				smd->sm.state = SMD_SM_STAT_INACTIVE;
			}else if(smd->isFault){
				smd->drv.enable = 0;
				smd->drv.start = 0;
				smd->sm.state = SMD_SM_STAT_FAULT;
			}else if(smd->downSenTrig>0 || smd->downSen()>0){
				/*T2*/
				smd->downSenTrig = 0;
				smd->drv.start = 0;
//				dbg_print("SU1 st: %d to %d\r\n",smd->sm.state,SMD_SM_STAT_DOWN_POS);
				smd->sm.state = SMD_SM_STAT_DOWN_POS;
			}else if(smd->upSenTrig >0 || smd->upSen() >0){
				smd->isStartingUp = 1;
				smd->upSenTrig =0;
				smd->downSenTrig = 0;
				smd->drv.enable = 1;
				smd->drv.start = 1;
				smd->dir = SMD_DIR_DOWN;
				smd->drv.dir = DM542T_DIR_DOWN;
				DM542T_SetRPM(&smd->drv, 5);
//				DM542T_Enable(&smd->drv);
//				DM542T_SetDir(&smd->drv, DM542T_DIR_DOWN);
//				DM542T_StartPulse(&smd->drv);
				SMD_SetSpeed(smd, 5);
//				dbg_print("Su2 st: %d to %d\r\n",smd->sm.state,SMD_SM_STAT_GOING_DOWN);
				smd->sm.state = SMD_SM_STAT_GOING_DOWN;
			}else{
				/*T3*/
				smd->isStartingUp = 1;
				smd->upSenTrig = 0;
				smd->drv.enable = 1;
				smd->drv.start = 1;
				smd->dir = SMD_DIR_UP;
				smd->drv.dir = DM542T_DIR_UP;
				DM542T_SetRPM(&smd->drv, 5);
//				DM542T_Enable(&smd->drv);
//				DM542T_SetDir(&smd->drv, DM542T_DIR_UP);
//				DM542T_StartPulse(&smd->drv);
				SMD_SetSpeed(smd, 5);
//				dbg_print("SU3 st: %d to %d, dir:%d\r\n",
//						smd->sm.state,SMD_SM_STAT_GOING_UP,smd->drv.dir);
				smd->sm.state = SMD_SM_STAT_GOING_UP;
			}
			break;
		case SMD_SM_STAT_GOING_DOWN:
			if(smd->isStartingUp){

			}
			if(!smd->enable){
				/*T5*/
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_INACTIVE;
			}else if(smd->isFault){
				/*T9*/
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_FAULT;
			}else if(smd->downSenTrig>0 || smd->downSen()>0){
				smd->downSenTrig = 0;
				/*T6*/

				smd->drv.start = 0;
				smd->downSenTrig = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_DOWN_POS;
//				dbg_print("GD3\r\n");
			}else if( smd->isPosMatched){
				/*T7*/
//				dbg_print("down pos matched!\r\n");
				smd->drv.start = 0;
				smd->start = 0;
				smd->isPosMatched = 0;
				smd->sm.state = SMD_SM_STAT_DOWN_POS;
//				dbg_print("GD2\r\n");
			}else if(smd->dir == SMD_DIR_UP){
				/*T8*/
				smd->drv.start = 1;
				smd->drv.dir = DM542T_DIR_UP;
				smd->upSenTrig = 0;
				smd->sm.state = SMD_SM_STAT_GOING_UP;
//				dbg_print("GD3\r\n");
			}
		break;
		case SMD_SM_STAT_DOWN_POS:
			if(smd->isStartingUp>0){
				smd->isStartingUp = 0;
				smd->drv.start = 0;
				smd->start = 0;
			}
			if(!smd->enable){
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_INACTIVE;
			}else if(smd->isFault){
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_FAULT;
			}else
				if(smd->dir == SMD_DIR_UP){
				/*T11*/
					smd->upSenTrig = 0;
					smd->drv.start = 1;
					smd->drv.dir = DM542T_DIR_UP;
					smd->sm.state = SMD_SM_STAT_GOING_UP;
//					dbg_print("DP1\r\n");
			}

		break;
		case SMD_SM_STAT_GOING_UP:
			if(!smd->enable){
				/*T13*/
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_INACTIVE;
			}else if(smd->isFault){
				/*T17*/
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_FAULT;
			}else if(smd->upSenTrig>0 || smd->upSen()){
				/*T14*/
				smd->upSenTrig = 0;
				smd->drv.start = 0;
				smd->sm.state = SMD_SM_STAT_UP_POS;
//				dbg_print("GU1\r\n");
			}else if( smd->isPosMatched){
				/*T15*/
//				dbg_print("up pos matched!\r\n");
				smd->drv.start = 0;
				smd->isPosMatched = 0;
				smd->sm.state = SMD_SM_STAT_UP_POS;
//				dbg_print("GU2r\n");
			}else if(smd->dir == SMD_DIR_DOWN){
				/*T16*/
				smd->downSenTrig = 0;
				smd->drv.start = 1;
				smd->drv.dir = DM542T_DIR_DOWN;
				smd->sm.state = SMD_SM_STAT_GOING_DOWN;
//				dbg_print("GU3\r\n");
			}

			if(smd->isStartingUp){
				if(smd->downSenTrig>0 || smd->downSen()>0){
					/*T6*/
					smd->downSenTrig=0;
					smd->start = 0;
					smd->drv.start = 0;
					smd->drv.dir = DM542T_DIR_DOWN;
					smd->dir = SMD_DIR_DOWN;
					smd->sm.state = SMD_SM_STAT_DOWN_POS;
//					dbg_print("GU4\r\n");
				}
			}

		break;
		case SMD_SM_STAT_UP_POS:
			if(!smd->enable){
				/*T18*/
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_INACTIVE;
			}else if(smd->isFault){
				/*T20*/
				smd->drv.enable = 0;
				smd->start = 0;
				smd->sm.state = SMD_SM_STAT_FAULT;
			}else if(smd->dir == SMD_DIR_DOWN){
				/*T19*/
				smd->drv.start = 1;
				smd->drv.dir = DM542T_DIR_DOWN;
				smd->downSenTrig = 0;
				smd->sm.state = SMD_SM_STAT_GOING_DOWN;
//				dbg_print("UP1\r\n");
			}
			if(smd->isStartingUp){
				smd->drv.dir = DM542T_DIR_DOWN;
				smd->drv.start = 1;
				smd->downSenTrig = 0;
				smd->start = 1;
				smd->dir = SMD_DIR_DOWN;
				smd->sm.state = SMD_SM_STAT_GOING_DOWN;
//				dbg_print("UP2\r\n");
			}
		break;
		case SMD_SM_STAT_FAULT:
			if(!smd->enable){
				/*T21*/
				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
			}else if(smd->isFaultClear){
				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_CLR_TRUE);
			}
		break;
		default:
			break;
	}

//	switch (smd->sm.state) {
//		case SMD_SM_STAT_NONE:
//		break;
//		case SMD_SM_STAT_INACTIVE:
//			if(smd->enable){
//				/*T0*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_ENABLE);
//			}
//		break;
//		case SMD_SM_STAT_STARTUP:
//			if(!smd->enable){
//				/*T1*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
//			}else if(smd->isFault){
//				/*T4*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_TRUE);
//			}else if(smd->downSenTrig>0 || smd->downSen()>0){
//				/*T2*/
//				smd->upSenTrig = 0;
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DOWN_SEN_TRUE);
//			}else if(smd->upSenState >0 || smd->upSen() >0){
//				smd->isStartingUp = 1;
//				SMD_SetSpeed(smd, 5);
//				SMD_Enable(smd);
////				SMD_SetDir(smd, SMD_DIR_UP);
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_UP_SEN_TRUE);
//			}else{
//				/*T3*/
//				smd->isStartingUp = 1;
//				SMD_SetSpeed(smd, 5);
//				SMD_Enable(smd);
//				SMD_SetDir(smd, SMD_DIR_UP);
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DOWN_SEN_FALSE);
//
//			}
//			break;
//		case SMD_SM_STAT_GOING_DOWN:
//			if(!smd->enable){
//				/*T5*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
//			}else if(smd->isFault){
//				/*T9*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_TRUE);
//			}else if(smd->downSenTrig || smd->downSenState){
//				/*T6*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DOWN_SEN_TRUE);
//			}else if( smd->isPosMatched){
//				/*T7*/
//				dbg_print("down pos matched!\r\n");
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DOWN_POS_MATCHED);
//			}else if(smd->dir == SMD_DIR_UP){
//				/*T8*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DIR_UP);
//			}
//		break;
//		case SMD_SM_STAT_DOWN_POS:
//			if(!smd->enable){
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
//			}else if(smd->isFault){
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_TRUE);
//			}else //if(smd->start){
//				if(smd->dir == SMD_DIR_UP){
//				/*T11*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DIR_UP);
//				}
//			//}
//		break;
//		case SMD_SM_STAT_GOING_UP:
//			if(!smd->enable){
//				/*T13*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
//			}else if(smd->isFault){
//				/*T17*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_TRUE);
//			}else if(smd->upSenTrig || smd->upSen()){
//				/*T14*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_UP_SEN_TRUE);
//			}else if( smd->isPosMatched){
//				/*T15*/
//				dbg_print("up pos matched!\r\n");
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_UP_POS_MATCHED);
//			}else if(smd->dir == SMD_DIR_DOWN){
//				/*T16*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DIR_DOWN);
//			}
//
//			if(smd->isStartingUp){
//				if(smd->downSenTrig || smd->downSen()){
//					/*T6*/
//					SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DOWN_SEN_TRUE);
//				}
//			}
//
//		break;
//		case SMD_SM_STAT_UP_POS:
//			if(!smd->enable){
//				/*T18*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
//			}else if(smd->isFault){
//				/*T20*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_TRUE);
//			}else if(smd->dir == SMD_DIR_DOWN){
//				/*T19*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DIR_DOWN);
//			}
//			if(smd->isStartingUp){
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DIR_DOWN);
//			}
//		break;
//		case SMD_SM_STAT_FAULT:
//			if(!smd->enable){
//				/*T21*/
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
//			}else if(smd->isFaultClear){
//				SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_FAULT_CLR_TRUE);
//			}
//		break;
//		default:
//			break;
//	}
}

/*SMD Public functions---------------------------------*/

/**
 * @brief Initializes the stepper motor driver structure.
 *
 * This function initializes all the parameters of the stepper motor driver structure
 * to their default values, including motor position, speed, sensor states, and state machine settings.
 * It also initializes the stepper motor driver with the `DM542T_Init` function.
 *
 * @param smd Pointer to the stepper motor driver structure to initialize.
 */
void SMD_Init(StpMotDrv_ts *smd){
	smd->id = 0;
	smd->enable = 0;
	smd->pos = 0;
	smd->setPos = 0;
	smd->setPosLast = 0;
	smd->refRpm = 0;
	smd->inputRpm = 0;
	smd->downSenState = 0;
	smd->upSenState = 0;
	smd->downSenTrig = 0;
	smd->upSenTrig = 0;
	smd->sm.event = 0;
	smd->sm.state = SMD_SM_STAT_INACTIVE;
	smd->sm.trigger = 0;
	smd->sm.trans = smTrnsList;
	smd->maxPos = SMD_MAX_POS_ANGLE;
	smd->sm.maxTrans = (sizeof(smTrnsList)/sizeof(smTrnsList[0]));

	smd->downSen = SMD_ReadDownSen;
	smd->upSen = SMD_ReadUpSen;
	DM542T_Init(&smd->drv);
}

/**
 * @brief Runs the state machine and processes the trigger events.
 *
 * This function processes the input data and triggers for the stepper motor state machine.
 * It calls the `SMD_SM_ProcessInputData` and `SMD_SM_ProcessTrigger` functions to handle the events
 * and triggers based on the current state of the stepper motor driver.
 *
 * @param smd Pointer to the stepper motor driver structure.
 */
void SMD_Run(StpMotDrv_ts *smd){
	SMD_SM_ProcessInputData(smd);
	SMD_SM_ProcessTrigger(smd);

//	smd->drv.enable =1;
//	smd->drv.start = 1;
//	smd->drv.dir =DM542T_DIR_DOWN;
//	DM542T_SetRPM(&smd->drv, 5);


//	/*Check state & events*/
//	for(uint8_t i = SMD_SM_STAT_INACTIVE; i < smd->sm.maxTrans; i++){
//		if(	smd->sm.state == smTrnsList[i].fromState &&
//				smd->sm.trigger == smTrnsList[i].trigger)
//		{
//			if(smTrnsList[i].trigger == SMD_SM_TRIG_NONE){continue;}
//
//			dbg_print("SMD_%d:{T%d => St:%d->%d, ev:%d, trig:%d, "
//					"en:%d, start: %d, dir:%d isStartUp:%d,"
//					"dowTrig:%d, DownStat:%d, UpTrig:%d, UpStat:%d}\r\n",
//					smd->id, i, smTrnsList[i].fromState, smTrnsList[i].toState, smd->sm.event, smd->sm.trigger,
//					smd->enable,smd->start,smd->dir,
//					smd->isStartingUp,smd->downSenState, smd->downSenTrig,smd->upSenState, smd->upSenTrig);
//			smTrnsList[i].action(smd);
//			smd->sm.state = smTrnsList[i].toState;
//			smd->sm.trigger = SMD_SM_TRIG_NONE;
//			return;
//		}
//	}

	DM542T_Run(&smd->drv);
}

/**
 * @brief Enables the stepper motor driver.
 *
 * This function sets the `enable` flag of the stepper motor driver to 1, allowing the motor to operate.
 *
 * @param smd Pointer to the stepper motor driver structure.
 */
void SMD_Enable(StpMotDrv_ts *smd){
	smd->enable = 1;
//	SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_ENABLE);
}

/**
 * @brief Disables the stepper motor driver.
 *
 * This function sets the `enable` flag of the stepper motor driver to 0, disabling the motor.
 *
 * @param smd Pointer to the stepper motor driver structure.
 */
void SMD_Disable(StpMotDrv_ts *smd){
	smd->enable = 0;
//	SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_DISABLE);
}

/**
 * @brief Starts the stepper motor driver.
 *
 * This function sets the `start` flag to 1, indicating that the stepper motor should start moving.
 *
 * @param smd Pointer to the stepper motor driver structure.
 */
void SMD_Start(StpMotDrv_ts *smd){
	smd->start = 1;
//	SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_ENABLE);
}

/**
 * @brief Stops the stepper motor driver.
 *
 * This function sets the `start` flag to 0, indicating that the stepper motor should stop.
 *
 * @param smd Pointer to the stepper motor driver structure.
 */
void SMD_Stop(StpMotDrv_ts *smd){
	smd->start = 0;
//	SMD_SM_SetTriger(&smd->sm, SMD_SM_TRIG_ENABLE);
}


/**
 * @brief Sets the direction of the stepper motor.
 *
 * This function sets the direction of the stepper motor based on the `dir` parameter. It can be either
 * upwards or downwards and adjusts the direction accordingly.
 *
 * @param smd Pointer to the stepper motor driver structure.
 * @param dir The direction to set for the stepper motor (SMD_Dir_te).
 */
void SMD_SetDir(StpMotDrv_ts *smd, SMD_Dir_te dir){
	smd->dir = dir;//(dir == SMD_DIR_DOWN? DM542T_DIR_DOWN : DM542T_DIR_UP);
//	SMD_SM_SetTriger(&smd->sm,
//					(dir == SMD_DIR_DOWN?
//					 SMD_SM_TRIG_DIR_DOWN :
//					 SMD_SM_TRIG_DIR_UP));
}

/**
 * @brief Sets the speed of the stepper motor.
 *
 * This function sets the speed (RPM) of the stepper motor by adjusting the `inputRpm` parameter.
 *
 * @param smd Pointer to the stepper motor driver structure.
 * @param rpm The speed (RPM) to set for the stepper motor.
 */
void SMD_SetSpeed(StpMotDrv_ts *smd, float rpm){
	smd->inputRpm = rpm;
//	DM542T_SetRPM(&smd->drv, rpm);
}

/**
 * @brief Sets the position of the stepper motor.
 *
 * This function updates the current position of the stepper motor driver.
 * The actual implementation for setting the position can be added in the future.
 *
 * @param smd Pointer to the stepper motor driver structure.
 * @param pos The position to set for the stepper motor.
 */
void SMD_SetPos(StpMotDrv_ts *smd, uint16_t pos){
	smd->pos = pos;

	/*TODO: to write code*/
}
