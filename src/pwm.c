#include "pwm.h"
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <stdbool.h>

/* === PWM uređaji === */
#define PWM0_NODE DT_NODELABEL(pwm0)
#define PWM1_NODE DT_NODELABEL(pwm1)

static const struct device *pwm0_dev = DEVICE_DT_GET(PWM0_NODE); // CH0 -> P0.31
static const struct device *pwm1_dev = DEVICE_DT_GET(PWM1_NODE); // CH0 -> P0.28

/* --- Prosti flegovi i poslednji parametri (bez mutexa) --- */
static volatile bool     ch0_ready = false, ch1_ready = false;
static volatile bool     ch0_running = false, ch1_running = false;
static volatile uint32_t ch0_freq_hz = 0,   ch1_freq_hz = 0;
static volatile uint8_t  ch0_duty_pc = 0,   ch1_duty_pc = 0;

/* --- Pomoćna rutinska primena podešavanja --- */
static int pwm_apply(const struct device *dev, uint8_t hw_ch,
                     uint32_t freq_hz, uint8_t duty_pc)
{
    if (!dev || !device_is_ready(dev)) return -ENODEV;
    if (freq_hz == 0 || duty_pc > 100) return -EINVAL;

    uint32_t period_us = 1000000U / freq_hz;
    if (period_us == 0) period_us = 1U;

    uint32_t pulse_us = (period_us * duty_pc) / 100U;
    if (duty_pc > 0 && pulse_us == 0) pulse_us = 1U; /* clamp na 1 µs */

    return pwm_set(dev, hw_ch, PWM_USEC(period_us), PWM_USEC(pulse_us), 0);
}

int pwm_ctrl_init(void)
{
    if (!device_is_ready(pwm0_dev)) {
        printk("PWM0 device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(pwm1_dev)) {
        printk("PWM1 device not ready\n");
        return -ENODEV;
    }

    /* Osiguraj stop stanje (duty=0) na oba kanala */
    (void)pwm_apply(pwm0_dev, 0, 1000, 0);
    (void)pwm_apply(pwm1_dev, 0, 1000, 0);

    ch0_ready = true;   ch1_ready = true;
    ch0_running = false; ch1_running = false;
    ch0_freq_hz = 0;    ch1_freq_hz = 0;
    ch0_duty_pc = 0;    ch1_duty_pc = 0;

    printk("PWM devices ready\n");
    return 0;
}

/* ===================== Kanal 0: P0.31 → 10 kHz, 40% ===================== */
void pwm_ch0_start(void)
{
    if (!ch0_ready) { printk("PWM0 not ready\n"); return; }

    const uint32_t freq_hz   = 10000; /* 10 kHz */
    const uint8_t  duty_pc   = 40;    /* 40% */

    /* Idempotentno: već radi sa istim parametrima */
    if (ch0_running && ch0_freq_hz == freq_hz && ch0_duty_pc == duty_pc) {
        return;
    }

    int ret = pwm_apply(pwm0_dev, 0, freq_hz, duty_pc);
    if (ret) {
        printk("PWM0 CH0 start failed (%d)\n", ret);
        return;
    }

    ch0_running = true;
    ch0_freq_hz = freq_hz;
    ch0_duty_pc = duty_pc;
    printk("PWM0 CH0 ON (10 kHz, 40%%)\n");
}

void pwm_ch0_stop(void)
{
    if (!ch0_ready) { printk("PWM0 not ready\n"); return; }

    /* Idempotentno: već ugašen */
    if (!ch0_running) {
        return;
    }

    /* Stop = duty 0; frekvencija nebitna (uzmi zadnju ili 1 kHz fallback) */
    uint32_t f = ch0_freq_hz ? ch0_freq_hz : 1000U;
    int ret = pwm_apply(pwm0_dev, 0, f, 0);
    if (ret) {
        printk("PWM0 CH0 stop failed (%d)\n", ret);
        return;
    }

    ch0_running = false;
    ch0_duty_pc = 0;
    printk("PWM0 CH0 OFF\n");
}

/* ===================== Kanal 1: P0.28 → 8 kHz, 1% ===================== */
void pwm_ch1_start(void)
{
    if (!ch1_ready) { printk("PWM1 not ready\n"); return; }

    const uint32_t freq_hz   = 8000; /* 8 kHz */
    const uint8_t  duty_pc   = 1;    /* 1% */

    if (ch1_running && ch1_freq_hz == freq_hz && ch1_duty_pc == duty_pc) {
        return;
    }

    int ret = pwm_apply(pwm1_dev, 0, freq_hz, duty_pc);
    if (ret) {
        printk("PWM1 CH0 start failed (%d)\n", ret);
        return;
    }

    ch1_running = true;
    ch1_freq_hz = freq_hz;
    ch1_duty_pc = duty_pc;
    printk("PWM1 CH0 ON (8 kHz, 1%%)\n");
}

void pwm_ch1_stop(void)
{
    if (!ch1_ready) { printk("PWM1 not ready\n"); return; }

    if (!ch1_running) {
        return;
    }

    uint32_t f = ch1_freq_hz ? ch1_freq_hz : 1000U;
    int ret = pwm_apply(pwm1_dev, 0, f, 0);
    if (ret) {
        printk("PWM1 CH0 stop failed (%d)\n", ret);
        return;
    }

    ch1_running = false;
    ch1_duty_pc = 0;
    printk("PWM1 CH0 OFF\n");
}
