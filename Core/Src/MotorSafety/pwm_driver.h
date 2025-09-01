/*
 * pwm_driver.h
 *
 *  Created on: Jul 21, 2025
 *      Author: Tanvir Sahed
 */

#ifndef SRC_MOTORSAFETY_PWM_DRIVER_H_
#define SRC_MOTORSAFETY_PWM_DRIVER_H_

#include "pwmIF.h"

PWM_StatusCode_t PWM_HW_Init(PWM_t *pwm);
PWM_StatusCode_t PWM_HW_Start(PWM_t *pwm);
PWM_StatusCode_t PWM_HW_Stop(PWM_t *pwm);
PWM_StatusCode_t PWM_HW_SetDuty(PWM_t *pwm, uint16_t duty_x100);
PWM_StatusCode_t PWM_HW_SetFrequency(PWM_t *pwm, uint32_t freq_hz);


#endif /* SRC_MOTORSAFETY_PWM_DRIVER_H_ */
