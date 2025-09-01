/**
 * @file DM542T.c
 * @brief Implementation of functions to control the DM542T stepper driver.
 *
 * This file contains initialization, configuration, and operation functions
 * for the DM542T stepper motor driver. It handles enabling, disabling,
 * setting direction, and pulse control for the driver.
 *
 * @author RusselPC
 * @date Dec 11, 2024
 */

#include "DM542T.h"
#include <stdio.h>
#include "debug.h"

#define PIN_RESET  	0
#define PIN_SET  	1

/**
 * @brief Creates a delay of at least 5 microseconds.
 */
DM542T_Error_te DM542T_CalcTimerPeriod(DM542T_ts *drv, float rpm);

/**
 * @brief Initializes the DM542T stepper driver structure.
 *
 * @param[in,out] drv Pointer to the DM542T stepper driver structure.
 * @return Error code indicating success or failure of initialization.
 *         - DM542T_ERR_OK: Initialization successful.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 */
DM542T_Error_te DM542T_Init(DM542T_ts *drv){
	if(drv == NULL){
		return DM542T_ERR_NULL_PTR;
	}

	drv->enable = 0;
	drv->fault = 0;
	drv->faultClear = 0;
	drv->state = DM542T_SM_STATE_NONE_0;
	drv->posAngle = 0.0;
	drv->pps = 0;
	drv->rpm = 0;
	drv->isChanged = 0;
	if(drv->getSysFreq != NULL){
		uint32_t freq = drv->getSysFreq();
		if(freq == 0U){
			return DM542T_ERR_INVALID_VALUE;
		}
		drv->timerConfig.sysFreq = freq;
	}else{
		return DM542T_ERR_NULL_PTR;
	}
	return DM542T_ERR_OK;
}




/**
 * @brief Enables the stepper driver.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @return DM542T_Error_te Error status.
 *         - DM542T_ERR_OK: Successfully disabled.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 */
DM542T_Error_te DM542T_Enable(DM542T_ts *drv){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    drv->enable = 1;

    if (drv->enPin != NULL){
        drv->enPin(PIN_SET);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    return DM542T_ERR_OK;
}

/**
 * @brief Disables the stepper driver.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @return DM542T_Error_te Error status.
 *         - DM542T_ERR_OK: Successfully disabled.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 */
DM542T_Error_te DM542T_Disable(DM542T_ts *drv){

    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    drv->enable = 0;

    if (drv->enPin != NULL){
        drv->enPin(PIN_RESET);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    return DM542T_ERR_OK;
}

/**
 * @brief Starts generating PWM pulses for the stepper motor.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @return DM542T_Error_te Error status.
 *         - DM542T_ERR_OK: PWM started successfully.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 *         - DM542T_ERR_PWM_START_FAILED: Failed to start PWM.
 */
DM542T_Error_te DM542T_StartPulse(DM542T_ts *drv){

    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    drv->start = 1;

    if (drv->startPWM != NULL){
        if(drv->startPWM() != DM542T_ERR_OK){
        	return DM542T_ERR_PWM_START_FAILED;
        }
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    return DM542T_ERR_OK;
}


/**
 * @brief Stops generating PWM pulses for the stepper motor.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @return DM542T_Error_te Error status.
 *         - DM542T_ERR_OK: PWM stopped successfully.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 *         - DM542T_ERR_PWM_STOP_FAILED: Failed to stop PWM.
 */
DM542T_Error_te DM542T_StopPulse(DM542T_ts *drv){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    drv->start = 0;

    if (drv->stopPWM != NULL){
        if(drv->stopPWM() != DM542T_ERR_OK){
        	return DM542T_ERR_PWM_STOP_FAILED;
        }
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    return DM542T_ERR_OK;
}





/**
 * @brief Sets the direction of the stepper motor.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @param[in] dir Desired direction (CW or CCW).
 * @return DM542T_Error_te Error status.
 *         - DM542T_ERR_OK: Direction set successfully.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 *         - DM542T_ERR_INVALID_DIR: Invalid direction error.
 */
DM542T_Error_te DM542T_SetDir(DM542T_ts *drv, DM542T_Dir_te dir){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    if (dir != DM542T_DIR_CW && dir != DM542T_DIR_CCW){
        return DM542T_ERR_INVALID_VALUE;
    }

    drv->dir = dir;

    if (drv->dirPin != NULL){
        drv->dirPin(dir);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    return DM542T_ERR_OK;
}


/**
 * @brief Sets the RPM of the stepper motor.
 *
 * @param[in,out] drv Pointer to the DM542T stepper driver structure.
 * @param[in] rpm Desired speed in rotations per minute.
 * @return DM542T_Error_te Error status.
 * 		   - DM542T_ERR:
 *         - DM542T_ERR_OK: RPM set successfully.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 *         - DM542T_ERR_INVALID_VALUE: Invalid RPM value.
 */
DM542T_Error_te DM542T_SetRPM(DM542T_ts *drv, float rpm){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    if (rpm < 0.0F){
        return DM542T_ERR_INVALID_VALUE;
    }

//    /*If value doesn't change, it will return immediately*/
//    if (drv->rpm == rpm){
//        return DM542T_ERR_OK;
//    }

    /* Set RPM */
    drv->rpm = rpm;
    drv->isChanged = 1;

//    /* Calculate period */
//    DM542T_CalcTimerPeriod(drv, rpm);
//
//    /* Ensure period is within a valid range */
//    if (drv->timerConfig.period < 65356U){
//        if (drv->setTimerPeriod != NULL){
//            if(drv->setTimerPeriod(drv, drv->timerConfig.period) != DM542T_ERR_OK){
//            	return DM542T_ERR;
//            }
//        }else{
//            return DM542T_ERR_NULL_PTR;
//        }
//    }

//    DM542T_Update(drv);
    return DM542T_ERR_OK;
}


/**
 * @brief   Sets the motor step angle for the DM542T driver.
 * @param   drv    Pointer to the DM542T driver instance.
 * @param   angle  Step angle value to be set.
 * @return  DM542T_Error_te
 *          - DM542T_ERR_OK: Operation successful.
 *          - DM542T_ERR_NULL_PTR: drv pointer is NULL.
 *          - DM542T_ERR_INVALID_VALUE: Invalid angle value (must be > 0.0F).
 */
DM542T_Error_te DM542T_SetMotorStepAngle(DM542T_ts *drv, float angle){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    if (angle <= 0.0F){
        return DM542T_ERR_INVALID_VALUE;
    }

    drv->motorConfig.stepAngle = angle;
    drv->motorConfig.isChanged = 1;

    return DM542T_ERR_OK;
}

DM542T_Error_te DM542T_SetDriverMicrostep(DM542T_ts *drv, uint16_t microsteps){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    if (microsteps == 0){
        return DM542T_ERR_INVALID_VALUE;
    }

    drv->drvConfig.microsteps = microsteps;
    drv->drvConfig.isChanged = 1;

    return DM542T_ERR_OK;
}

/**
 * @brief   Calculates the number of motor steps per revolution based on the step angle.
 * @param   stepsPerRev Pointer to store the calculated steps per revolution.
 * @param   angle       Step angle in degrees.
 * @return  DM542T_Error_te
 *          - DM542T_ERR_OK: Operation successful.
 *          - DM542T_ERR_NULL_PTR: stepsPerRev pointer is NULL.
 *          - DM542T_ERR_INVALID_VALUE: Invalid angle value (must be > 0.0F).
 */
DM542T_Error_te DM542T_CalcMotorStepsPerRev(uint32_t *stepsPerRev, float angle){
    if (stepsPerRev == NULL){
        return DM542T_ERR_NULL_PTR;
    }
    if (angle <= 0.0F){
        return DM542T_ERR_INVALID_VALUE;
    }
    /*Calculate steps per revolution*/
    *stepsPerRev = ANGLE_MAX/angle;

    return DM542T_ERR_OK;
}


/**
 * @brief   Calculates the motor pulses per second (PPS) based on RPM and steps per revolution.
 * @param   pps          Pointer to store the calculated pulses per second.
 * @param   rpm          Motor speed in revolutions per minute (RPM).
 * @param   stepsPerRev  Number of steps per revolution.
 * @return  uint16_t
 *          - DM542T_ERR_OK: Operation successful.
 *          - DM542T_ERR_NULL_PTR: pps pointer is NULL.
 *          - DM542T_ERR_INVALID_VALUE: Invalid RPM value (must be >= 0.0F).
 * @note    This function calculates PPS using the formula:
 *          \f[ PPS = \frac{RPM \times StepsPerRev}{60} \f]
 */
DM542T_Error_te DM542T_CalcMotorPPS(uint32_t *pps, float rpm, uint32_t stepsPerRev){
    if (pps == NULL){
        return DM542T_ERR_NULL_PTR;
    }
    if (rpm < 0.0f){
        return DM542T_ERR_INVALID_VALUE;  // Invalid input, return 0
    }
    *pps =  rpm * stepsPerRev / 60;

    return DM542T_ERR_OK;
}

/**
 * @brief   Calculates the driver pulses per revolution (PPR).
 * @param   ppr          Pointer to store the calculated pulses per revolution.
 * @param   stepsPerRev  Number of steps per revolution.
 * @param   microsteps   Microstepping setting of the driver.
 * @return  uint16_t
 *          - DM542T_ERR_OK: Operation successful.
 *          - DM542T_ERR_NULL_PTR: ppr pointer is NULL.
 * @note    This function computes the PPR using the formula:
 *          \f[ PPR = StepsPerRev \times Microsteps \f]
 */
DM542T_Error_te DM542T_CalcDriverPPR(uint32_t *ppr, uint32_t stepsPerRev, uint16_t microsteps){
    if (ppr == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    *ppr =  stepsPerRev * microsteps;

    return DM542T_ERR_OK;
}





/**
 * @brief Calculates and updates the timer period based on RPM.
 *
 * @param[in,out] drv Pointer to the DM542T stepper driver structure.
 * @param[in] rpm Desired speed in rotations per minute.
 * @return DM542T_Error_te Error code indicating success or failure of calculation.
 *         - DM542T_ERR_OK: Calculation successful.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 *         - DM542T_ERR_INVALID_VALUE: Invalid RPM or PPR value.
 */
DM542T_Error_te DM542T_CalcTimerPeriod(DM542T_ts *drv, float rpm){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->getTimerPrescaler == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    drv->timerConfig.prescaler = drv->getTimerPrescaler();

    /* Check for invalid conditions (prescaler, RPM, and PPR) */
    if (drv->timerConfig.prescaler == 0U ||
    	rpm < 0.0f || drv->drvConfig.ppr == 0U)
    {
        return DM542T_ERR_INVALID_VALUE;
    }

    /* Calculate frequency */
    float freq = (rpm * drv->drvConfig.ppr) / 60.0f;

    /* Prevent division by zero */
    if (freq > 0.0f){
        drv->timerConfig.period = drv->timerConfig.sysFreq / ((drv->timerConfig.prescaler + 1.0f) * freq);
    }else{
    	drv->timerConfig.period = 0;
    }

    drv->timerConfig.isChanged = 1;
    return DM542T_ERR_OK;
}



/**
 * @brief Moves the stepper motor in the specified direction.
 *
 * Stops the pulse, sets the direction, and then restarts the pulse.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @param[in] dir Desired direction (CW or CCW).
 */
DM542T_Error_te DM542T_Move(DM542T_ts *drv, DM542T_Dir_te dir){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->stopPWM != NULL){
        drv->stopPWM();
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->dirPin != NULL){
        drv->dirPin(dir);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->delay_us != NULL){
        drv->delay_us(DM542T_DELAY_US);
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    if (drv->stopPWM != NULL){
        drv->stopPWM();
    }else{
        return DM542T_ERR_NULL_PTR;
    }

    return DM542T_ERR_OK;
}






/**
 * @brief Updates the DM542T stepper driver configuration if changes are detected.
 *
 * @param[in] drv Pointer to the DM542T stepper driver structure.
 * @return DM542T_Error_te Error code indicating success or failure of the update.
 *         - DM542T_ERR_OK: Update successful.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 *         - DM542T_ERR_INVALID_VALUE: Invalid RPM or period value.
 */
DM542T_Error_te DM542T_Update(DM542T_ts *drv){
    if (drv == NULL){
        return DM542T_ERR_NULL_PTR;  // Check for null pointer
    }
    DM542T_Error_te err = DM542T_ERR_OK;

    /*If the motor configuration is changed*/
    if(drv->motorConfig.isChanged){


		/*Calculate motor steps per revolution*/
		err = DM542T_CalcMotorStepsPerRev(&drv->motorConfig.stepsPerRev,
				drv->motorConfig.stepAngle);
		if(err != DM542T_ERR_OK)
		{
			return err;
		}
		drv->motorConfig.isChanged = 0;
    }

    /*If the driver configuration is changed*/
    if(drv->drvConfig.isChanged){


		/*Calculate ppr*/
    	err = DM542T_CalcDriverPPR(&drv->drvConfig.ppr,
    			drv->motorConfig.stepsPerRev,
				drv->drvConfig.microsteps);
		if(err != DM542T_ERR_OK)
		{
			return err;
		}
		// Mark that changes have been applied
		drv->drvConfig.isChanged = 0;
    }

    /*if motor rpm is changed*/
    if (drv->isChanged){



		// Try to calculate the timer period based on RPM
		DM542T_Error_te err = DM542T_CalcTimerPeriod(drv, drv->rpm);
		if (err != DM542T_ERR_OK){
			dbg_print("DM54T Error: PWM Period Calculation failed, in file:%s, at line:%u\r\n", __FILE__, __LINE__);
			return err;  // Return error if calculation failed
		}

		// Validate the calculated period
		if (drv->timerConfig.period >= 65356){
			dbg_print("DM54T Error: Invalid Period, in file:%s, at line:%u\r\n", __FILE__, __LINE__);
			return DM542T_ERR_INVALID_VALUE;  // Invalid period value
		}

		// Set the new timer period
		if(drv->setTimerPeriod != NULL){
			if(drv->setTimerPeriod((struct DM542T*)drv, drv->timerConfig.period) != DM542T_ERR_OK){
				dbg_print("DM54T Error: Set Period Failed, in file:%s, at line:%u\r\n", __FILE__, __LINE__);
				return DM542T_ERR;
			}
		}else{
			dbg_print("DM54T Error: setTimerPeriod() null pointer function, in file:%s, at line:%u\r\n", __FILE__, __LINE__);
			return DM542T_ERR_NULL_PTR;
		}
		// Mark that changes have been applied
		drv->isChanged = 0;
    }
    return DM542T_ERR_OK;  // Update successful
}

/**
 * @brief Runs the state machine for the stepper driver.
 *
 * Handles transitions between various states like initialization, idle,
 * movement, and error states.
 *
 * @param[in,out] drv Pointer to the DM542T stepper driver structure.
 * @return DM542T_Error_te Error code indicating success or failure.
 *         - DM542T_ERR_OK: No errors encountered.
 *         - DM542T_ERR_NULL_PTR: Null pointer error.
 */
DM542T_Error_te DM542T_Run(DM542T_ts *drv)
{
    if (drv == NULL) {
        return DM542T_ERR_NULL_PTR;  // Null pointer check
    }

    DM542T_Update(drv);

    switch (drv->state) {
        case DM542T_SM_STATE_INITIAL_1:
            if (drv->enable) {
                /* T1: S1 to S2 */
                dbg_print("DM542T: Driver Enabled\r\n");
                drv->state = DM542T_SM_STATE_IDLE_2;
            }
            break;

        case DM542T_SM_STATE_IDLE_2:
            if (!drv->enable) {
                /* T1: S2 to S1 */
                /* Stop pulse */
                DM542T_StopPulse(drv);
                /* Disable the driver */
                DM542T_Disable(drv);
                drv->start = 0;
                drv->state = DM542T_SM_STATE_INITIAL_1;
            } else if (drv->fault) {
                /* T7: S2 to S5 (Go to error state) */
                dbg_print("DM542T: Driver Fault!, File: %s, Line:%d ", __FILE__, __LINE__);
                drv->state = DM542T_SM_STATE_ERROR_5;
            }

            if (drv->start && drv->rpm > 0) {
                /* Enable the driver */
                DM542T_Enable(drv);

                if (drv->dir == DM542T_DIR_CW) {
                    /* T3: S2 to S3 */
                    /* Set the direction */
                    DM542T_SetDir(drv, DM542T_DIR_CW);
                    /* Start pulse */
                    DM542T_StartPulse(drv);
                    drv->state = DM542T_SM_STATE_MOVE_CW_3;
                } else if (drv->dir == DM542T_DIR_CCW) {
                    /* T5: S2 to S4 */
                    /* Set the direction */
                    DM542T_SetDir(drv, DM542T_DIR_CCW);
                    /* Start pulse */
                    DM542T_StartPulse(drv);
                    drv->state = DM542T_SM_STATE_MOVE_CCW_4;
                } else {
                    dbg_print("DM542T: Unknown Direction!, File: %s, Line: %d ", __FILE__, __LINE__);
                    drv->state = DM542T_SM_STATE_ERROR_5;
                }
            }
            break;

        case DM542T_SM_STATE_MOVE_CW_3:
            if (drv->fault) {
                /* T7: S3 to S5 (Go to error state) */
                DM542T_StopPulse(drv);
                DM542T_Disable(drv);
                drv->start = 0;
                dbg_print("DM542T: Driver Fault!, File: %s, Line:%d ", __FILE__, __LINE__);
                drv->state = DM542T_SM_STATE_ERROR_5;
            }
            if (!drv->enable || !drv->start || drv->dir != DM542T_DIR_CW) {
                /* T4: S3 to S2 */
                DM542T_StopPulse(drv);
                DM542T_Disable(drv);
                drv->state = DM542T_SM_STATE_IDLE_2;
            }
            break;

        case DM542T_SM_STATE_MOVE_CCW_4:
            if (drv->fault) {
                /* T7: S3 to S5 (Go to error state) */
                DM542T_StopPulse(drv);
                DM542T_Disable(drv);
                drv->start = 0;
                dbg_print("DM542T: Driver Fault!, File: %s, Line:%d ", __FILE__, __LINE__);
                drv->state = DM542T_SM_STATE_ERROR_5;
            }
            if (!drv->enable || !drv->start || drv->dir != DM542T_DIR_CCW) {
                /* T4: S3 to S2 */
                DM542T_StopPulse(drv);
                DM542T_Disable(drv);
                drv->state = DM542T_SM_STATE_IDLE_2;
            }
            break;

        case DM542T_SM_STATE_ERROR_5:
            if (drv->faultClear) {
                drv->faultClear = 0;
                drv->fault = 0;
                drv->start = 0;
                drv->state = DM542T_SM_STATE_IDLE_2;
            }
            break;

        default:
            /* Invalid state, go to initial state */
            dbg_print("DM542T: Invalid State! Defaulting to Initial state, File: %s, Line: %d\n", __FILE__, __LINE__);
            drv->state = DM542T_SM_STATE_INITIAL_1;
            return DM542T_ERR_INVALID_VALUE;  // Return error for invalid state
    }

    return DM542T_ERR_OK;  // Successfully ran state machine
}
