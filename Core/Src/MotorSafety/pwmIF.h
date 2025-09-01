#ifndef SRC_MOTORSAFETY_PWMIF_H_
#define SRC_MOTORSAFETY_PWMIF_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

typedef enum {
    PWM_STATUS_OK = 0,
    PWM_STATUS_ERROR = 1
} PWM_StatusCode_t;

typedef struct {
    uint32_t currentFreq;
    uint32_t currentDuty;
} PWM_Status_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    uint8_t gpio_af;
    uint32_t timer_clk_hz;
    uint32_t default_freq_hz;
    uint8_t default_duty_percent;
} PWM_Config_t;

typedef struct {
    PWM_Config_t config;
    PWM_Status_t state;
} PWM_t;

PWM_StatusCode_t PWM_Init(PWM_t *pwm);
PWM_StatusCode_t PWM_Start(PWM_t *pwm);
PWM_StatusCode_t PWM_Stop(PWM_t *pwm);
PWM_StatusCode_t PWM_SetDutyCycle(PWM_t *pwm, uint16_t duty_x100);
PWM_StatusCode_t PWM_SetFrequency(PWM_t *pwm, uint32_t freq_hz);

#endif /* SRC_MOTORSAFETY_PWMIF_H_ */
