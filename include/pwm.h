#ifndef PWM_CTRL_H
#define PWM_CTRL_H

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

/* Inicijalizacija PWM uređaja */
int pwm_ctrl_init(void);

/* Kanal 0 (PWM0/CH0 → P0.31, 10 kHz @ 40%) */
void pwm_ch0_start(void);
void pwm_ch0_stop(void);

/* Kanal 1 (PWM1/CH0 → P0.28, 8 kHz @ 1%) */
void pwm_ch1_start(void);
void pwm_ch1_stop(void);

#endif /* PWM_CTRL_H */
