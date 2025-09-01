/*
 * PID_Controller.h
 *
 *  Created on: Mar 22, 2022
 *      Author: RASEL_EEE
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include "main.h"

#define PID_MAX_SP	 		65535U		// max setpoint for others
#define PID_MAX_SP_KTC 		1000U		// max setpoint for thermocouple
#define PID_MAX_SP_ANAINP	4095U		// max setpoint for analog input
#define PID_MAX_HYST_VALUE 	1000U
#define PID_MAX_KP 			10000.0f
#define PID_MAX_KI			10000.0f
#define PID_MAX_KD			10000.0f
#define PID_MAX_I_MAX_OUT	10000.0f

#define PID_MIN_KP 			0.0f
#define PID_MIN_KI			0.0f
#define PID_MIN_KD			0.0f
#define PID_MAX_I_MIN_OUT	0.0f


#define __PID_Min(a,b)									(a<b ? a : b)				// to get the minimum value between two values
#define __PID_Max(a,b)									(a>b ? a : b)				// to get the minimum value between two values


typedef enum PID_FeedbackSource{
	PID_FB_SRC_NONE = 0U,
	PID_FB_SRC_KTHERMO_C1,
	PID_FB_SRC_KTHERMO_C2,
	PID_FB_SRC_ANAINP_1,
	PID_FB_SRC_ANAINP_2,
	PID_FB_SRC_MAX,
}PID_FeedbackSource;

/*
 * PID output modes.
 * 		1. Percent Output mode: It is used when the PID output is tied with the analog & PWM outputs
 * 		2. Discrete output mode: It is used when the PID output is tied with the digital outputs & relay outputs
 * */
typedef enum PID_OutputMode{
	PID_OM_NONE = 0U,
	PID_OM_PERCENT,
	PID_OM_DISCRETE,
	PID_OM_MAX,
}PID_OutputMode;


/*PID controller parameters
 * hystValue: Hysteresis value explanation with example
 * 		Ex: Set Point 500 C, hysteresis value 2deg C. If temperature was at or above 500 PID will turn off,
 * 		PID would not turn back on until value below the hysteresis value so heat would	not turn back on
 * 		until get felt below 498 C
 * */
typedef struct PID{
	uint8_t enable;
	PID_FeedbackSource fbSource;		// pid feedback source
	PID_OutputMode outputMode;			// output mode
	uint16_t hystValue;					// Hysteresis
	float setpoint;					// Target value
	float maxSetpoint;				// max allowable setpoint
	float measurement;					// measured value using sensor
	float kp;							// proportional gain
	float ki;							// integral gain
	float kd;							// derivative gain
	float error;						// error term
	float last_error;					// error term
	float output_p;						// proportional output
	float output_i;						// integral output
	float output_d;						// derivative output
	float output_pid;					// PID output
	float output_i_max;					// max integral output
	float error_time;					// current error time
	float last_error_time;				// last error time
}PID;

void PID_Controller(PID *pid);
void PID_Proportional(PID *pid);
void PID_Integral(PID *pid);
void PID_Derivative(PID *pid);
void PID_UpdateSpReso(PID *pid);

#endif /* INC_PID_CONTROLLER_H_ */
