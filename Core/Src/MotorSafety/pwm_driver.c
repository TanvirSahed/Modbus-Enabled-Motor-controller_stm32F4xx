#include "pwm_driver.h"

PWM_StatusCode_t PWM_HW_Init(PWM_t *pwm)
{
    GPIO_InitTypeDef gpio = {0};

    if ((pwm == NULL) || (pwm->config.htim == NULL)) {
        return PWM_STATUS_ERROR;
    }

    // GPIO Configuration
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Replace with actual port clock if needed

    gpio.Pin = pwm->config.gpio_pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = pwm->config.gpio_af;
    HAL_GPIO_Init(pwm->config.gpio_port, &gpio);

    pwm->config.htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm->config.htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm->config.htim->Init.Prescaler = 0U;
    pwm->config.htim->Init.Period = 0U;

    if (HAL_TIM_PWM_Init(pwm->config.htim) != HAL_OK) {
        return PWM_STATUS_ERROR;
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = 0U;

    if (HAL_TIM_PWM_ConfigChannel(pwm->config.htim, &sConfigOC, pwm->config.channel) != HAL_OK) {
        return PWM_STATUS_ERROR;
    }

    (void)PWM_HW_SetFrequency(pwm, pwm->config.default_freq_hz);
    (void)PWM_HW_SetDuty(pwm, pwm->config.default_duty_percent);

    return PWM_STATUS_OK;
}

PWM_StatusCode_t PWM_HW_Start(PWM_t *pwm)
{
    if (pwm == NULL) {
        return PWM_STATUS_ERROR;
    }
    return (HAL_TIM_PWM_Start(pwm->config.htim, pwm->config.channel) == HAL_OK) ? PWM_STATUS_OK : PWM_STATUS_ERROR;
}

PWM_StatusCode_t PWM_HW_Stop(PWM_t *pwm)
{
    if (pwm == NULL) {
        return PWM_STATUS_ERROR;
    }
    return (HAL_TIM_PWM_Stop(pwm->config.htim, pwm->config.channel) == HAL_OK) ? PWM_STATUS_OK : PWM_STATUS_ERROR;
}

PWM_StatusCode_t PWM_HW_SetDuty(PWM_t *pwm, uint16_t duty_x100)
{
    if ((pwm == NULL) || (duty_x100 > 10000U)) {
        return PWM_STATUS_ERROR;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(pwm->config.htim);
    uint32_t pulse = (arr * (uint32_t)duty_x100) / 10000U;
    __HAL_TIM_SET_COMPARE(pwm->config.htim, pwm->config.channel, pulse);

    return PWM_STATUS_OK;
}


PWM_StatusCode_t PWM_HW_SetFrequency(PWM_t *pwm, uint32_t freq)
{
    if ((pwm == NULL) || (freq == 0U)) {
        return PWM_STATUS_ERROR;
    }

    uint32_t prescaler = 0U;
    uint32_t period = (pwm->config.timer_clk_hz / freq) - 1U;

    while (period > 0xFFFFU) {
        prescaler++;
        period = (pwm->config.timer_clk_hz / (freq * (prescaler + 1U))) - 1U;
    }

    pwm->config.htim->Init.Prescaler = prescaler;
    pwm->config.htim->Init.Period = period;

    if (HAL_TIM_PWM_Init(pwm->config.htim) != HAL_OK) {
        return PWM_STATUS_ERROR;
    }

    return PWM_STATUS_OK;
}
