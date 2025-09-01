/*
 * pwm_driver.h
 *
 *  Created on: Jul 21, 2025
 *      Author: Tanvir Sahed
 */
#include "pwmIF.h"
#include "pwm_driver.h"

PWM_StatusCode_t PWM_Init(PWM_t *pwm)
{
    if (pwm == NULL) {
        return PWM_STATUS_ERROR;
    }

    pwm->state.currentFreq = pwm->config.default_freq_hz;
    pwm->state.currentDuty = pwm->config.default_duty_percent;

    return PWM_HW_Init(pwm);
}

PWM_StatusCode_t PWM_Start(PWM_t *pwm)
{
    return PWM_HW_Start(pwm);
}

PWM_StatusCode_t PWM_Stop(PWM_t *pwm)
{
    return PWM_HW_Stop(pwm);
}

PWM_StatusCode_t PWM_SetDutyCycle(PWM_t *pwm, uint16_t duty_x100)
{
    PWM_StatusCode_t status = PWM_HW_SetDuty(pwm, duty_x100);
    if (status == PWM_STATUS_OK) {
        pwm->state.currentDuty = (uint32_t)duty_x100;
    }
    return status;
}


PWM_StatusCode_t PWM_SetFrequency(PWM_t *pwm, uint32_t freq_hz)
{
    PWM_StatusCode_t status = PWM_HW_SetFrequency(pwm, freq_hz);
    if (status == PWM_STATUS_OK) {
        pwm->state.currentFreq = freq_hz;
    }
    return status;
}

