/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2023 Intel Corporation
 * and Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include "impulse.h"
#include "ble_nus.h"
#include "dac.h"
#include "mux.h"
#include "fuel_gauge.h"
#include "nrfx_adc.h"
#include "pwm.h"  /* <<--- NOVO: naš PWM modul */

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* ===================== MUX config ===================== */
struct mux_config stim_mux_config = {
    .spi_dev      = STIM_MUX_SPI_DEV,
    .gpio_dev     = STIM_MUX_GPIO_DEV,
    .le_pin       = STIM_MUX_LE_PIN,
    .clr_pin      = STIM_MUX_CLR_PIN,
    .num_channels = STIM_MUX_NUM_CHANNELS
};

/* ===================== GPIO DT refs ===================== */
static const struct gpio_dt_spec led0   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec        kill   = GPIO_DT_SPEC_GET(DT_ALIAS(kill), gpios);
const struct gpio_dt_spec        pb_mcu = GPIO_DT_SPEC_GET(DT_ALIAS(pb_mcu), gpios);

/* Ovi pinovi su deklarisani u drugim modulima */
extern const struct gpio_dt_spec pulse_cathode;
extern const struct gpio_dt_spec pulse_anode;
extern const struct gpio_dt_spec dc_dc_en;

/* ===================== GPIO init ===================== */
static int init_gpios(void)
{
    int ret;

    if (!device_is_ready(pulse_cathode.port) ||
        !device_is_ready(pulse_anode.port)   ||
        !device_is_ready(dc_dc_en.port)) {
        printk("One of pulse GPIO devices not ready\n");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&pulse_cathode, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&pulse_anode, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    return 0;
}

#include <hal/nrf_power.h>
static void log_reset_reason(void)
{
    uint32_t reas = nrf_power_resetreas_get(NRF_POWER);

    if (reas & POWER_RESETREAS_RESETPIN_Msk) LOG_INF("Reset: pin reset");
    if (reas & POWER_RESETREAS_DOG_Msk)      LOG_INF("Reset: watchdog");
    if (reas & POWER_RESETREAS_SREQ_Msk)     LOG_INF("Reset: soft reset");
    if (reas & POWER_RESETREAS_LOCKUP_Msk)   LOG_INF("Reset: CPU lockup");
    if (reas & POWER_RESETREAS_OFF_Msk)      LOG_INF("Reset: wakeup from OFF");
    if (reas & POWER_RESETREAS_LPCOMP_Msk)   LOG_INF("Reset: LPCOMP event");

    nrf_power_resetreas_clear(NRF_POWER, reas);
}

/* ===================== main ===================== */
void main(void)
{
    int err;

    log_reset_reason();

    /* --- LED/KILL/PB inicijalizacija pre upotrebe --- */
    if (!device_is_ready(led0.port)) {
        printk("LED0 device not ready\n");
        return;
    }
    if (!device_is_ready(kill.port) || !device_is_ready(pb_mcu.port)) {
        printk("KILL/PB_MCU device not ready\n");
        return;
    }

    (void)gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    (void)gpio_pin_configure_dt(&kill, GPIO_OUTPUT_INACTIVE);
    (void)gpio_pin_configure_dt(&pb_mcu, GPIO_OUTPUT_INACTIVE);

    gpio_pin_set_dt(&kill, 0);   /* enable kroz LTC2950 (active-low linija) */
    gpio_pin_set_dt(&pb_mcu, 0); /* PB_MCU neaktivan */

    /* --- Ostali GPIO pinovi (stim) --- */
    if (init_gpios() != 0) {
        printk("GPIO initialization failed!\n");
        return;
    }

    /* Signal da je sistem "živ" */
    gpio_pin_set_dt(&led0, 1);

    /* --- DC-DC --- */
    gpio_pin_set_dt(&dc_dc_en, 1);
    printk("DC-DC converter enabled\n");
    k_sleep(K_MSEC(100));

    /* --- BLE --- */
    err = ble_nus_init();
    if (err) {
        printk("BLE initialization failed (%d)\n", err);
        return;
    }

    /* --- DAC --- */
    dac_init();
    dac_set_value(80); /* Default */

    /* --- MUX --- */
    mux_init(&stim_mux_config);

    /* --- Fuel Gauge --- */
    bq27220_init();

    /* --- (Opcionalno) SAADC lanac --- */
    // start_nrfx_adc();

    /* --- PWM modul (pwm.c) --- */
    if (pwm_ctrl_init() != 0) {
        printk("PWM init failed\n");
        return;
    }

    // /* Demo: upali oba kanala na 5 s */
    pwm_ch0_start();  /* PWM0/CH0 → P0.31, 10 kHz @ 40% (definisano u pwm.c) */
    // pwm_ch1_start();  /* PWM1/CH0 → P0.28, 8 kHz @ 1%   (definisano u pwm.c) */

    k_sleep(K_SECONDS(5));

    pwm_ch0_stop();
    // pwm_ch1_stop();

    /* --- Glavna petlja --- */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
