/*
 * PID_Controller.c
 *
 *  Created on: Mar 22, 2022
 *      Author: RASEL_EEE
 */

#include "PID_Controller.h"
#include <math.h>
#include "Debug.h"


/**
  * @brief PID controller
  * @param pid: PID_Controller, for the PID parameters
  */
void PID_Controller(PID *pid){
	if (pid->outputMode == PID_OM_PERCENT){
		pid->error = pid->setpoint - pid->measurement;						// calculates error term
		pid->error_time = HAL_GetTick();
		PID_Proportional(pid);												// calculates proportional output
		PID_Integral(pid);													// calculates integral output
		PID_Derivative(pid);												// calculates derivative output


		pid->output_i = __PID_Min(pid->output_i, pid->output_i_max);		// makes the integral output limited to set value of the max integral output
		pid->output_i = __PID_Max(pid->output_i, pid->output_i_max*(-1.0f));		// makes the integral output limited to set value of the max integral output
		pid->last_error_time = pid->error_time;								// sets the error time to last error time
		pid->last_error = pid->error;									// sets the error value to last error value
		pid->output_pid = pid->output_p + pid->output_i + pid->output_d;	// sum of proportional, integral and derivative output
		pid->output_pid = pid->output_pid;
		if(pid->output_pid > 100.0 || pid->output_pid < -100.0){
			pid->output_pid = 100.0;
		}
		else if(pid->output_pid < 0.0 && pid->output_pid >= -100.0){
			pid->output_pid = pid->output_pid*(-1);
		}
	}else if (pid->outputMode == PID_OM_DISCRETE){							// discrete output mode

		if(pid->measurement >= (float)pid->setpoint){
			pid->output_pid = 0;
		}else if(pid->measurement < (float)(pid->setpoint - pid->hystValue)){
			pid->output_pid = 1;
		}
	}
}


//void PID_Controller(PID *pid) {
//    // Get current error and time
//    pid->error = pid->setpoint - pid->measurement;
//    pid->error_time = HAL_GetTick();
//
//    // Calculate proportional, integral, and derivative terms
//    PID_Proportional(pid);
//    PID_Integral(pid);
//    PID_Derivative(pid);
//
//    // Sum up the PID components
//    pid->output_pid = pid->output_p + pid->output_i + pid->output_d;
//
//    // Apply back-calculation anti-windup
//    if (pid->output_pid > 100.0f) {
//        pid->output_pid = 100.0f;
//        pid->output_i -= (pid->output_pid - 100.0f);  // Reduce integral term
//    } else if (pid->output_pid < -100.0f) {
//        pid->output_pid = -100.0f;
//        pid->output_i -= (pid->output_pid + 100.0f);  // Reduce integral term
//    }
//
//    // Prevent the integral term from exceeding defined bounds
//    pid->output_i = __PID_Min(pid->output_i, pid->output_i_max);
//    pid->output_i = __PID_Max(pid->output_i, -pid->output_i_max);
//
//    // Convert output to integer and handle sign appropriately
//    pid->output_pid = (int16_t)round(pid->output_pid);
//
//    // Discrete output mode
//    if (pid->outputMode == PID_OM_DISCRETE) {
//        if (pid->measurement >= (float)pid->setpoint) {
//            pid->output_pid = 0;
//        } else if (pid->measurement < (float)(pid->setpoint - pid->hystValue)) {
//            pid->output_pid = 1;
//        }
//    }
//
//    // Update last error and time for the next iteration
//    pid->last_error = pid->error;
//    pid->last_error_time = pid->error_time;
//}




/**
  * @brief proportional term for the PID controller
  * @param pid: PID_Controller, for the PID parameters
  */
inline void PID_Proportional(PID *pid){
	pid->output_p = pid->kp * pid->error;
}

/**
  * @brief Integral term for the PID controller
  * @param pid: PID_Controller, for the PID parameters
  */
inline void PID_Integral(PID *pid){
	pid->output_i = pid->output_i + (pid->ki * pid->error);
}

/**
  * @brief Derivative term for the PID controller
  * @param pid: PID_Controller, for the PID parameters
  */
inline void PID_Derivative(PID *pid){

	float delta_error = pid->error - pid->last_error;
	float delta_error_time = pid->error_time - pid->last_error_time;
	if(delta_error_time != 0.0f) pid->output_d = ((delta_error* pid->kd)/delta_error_time) ;

}


/*
 * update the set point resolution means max set point
 * cause of there are to type of input source, the range of one is 0 to 1000 & another one is 0 to 4095
 * so we need to update the max allowable set point according to feedback source
 * */
void PID_UpdateSpReso(PID *pid){
	if(pid->fbSource == PID_FB_SRC_KTHERMO_C1 || pid->fbSource == PID_FB_SRC_KTHERMO_C2){
		pid->maxSetpoint = PID_MAX_SP_KTC;
	}else if(pid->fbSource == PID_FB_SRC_ANAINP_1 || pid->fbSource == PID_FB_SRC_ANAINP_2){
		pid->maxSetpoint = PID_MAX_SP_ANAINP;
	}else{
		pid->maxSetpoint = PID_MAX_SP_ANAINP;
	}
	pid->setpoint = 0U;
}
