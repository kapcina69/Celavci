/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2023 Intel Corporation and Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "impulse.h"
#include "ble_nus.h"
#include "dac.h"
#include "mux.h"
#include "fuel_gauge.h"
#include "nrfx_adc.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>



struct mux_config stim_mux_config = {
        .spi_dev = STIM_MUX_SPI_DEV,
        .gpio_dev = STIM_MUX_GPIO_DEV,
        .le_pin = STIM_MUX_LE_PIN,
        .clr_pin = STIM_MUX_CLR_PIN,
        .num_channels = STIM_MUX_NUM_CHANNELS
    };

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct gpio_dt_spec kill= GPIO_DT_SPEC_GET(DT_ALIAS(kill), gpios); 
const struct gpio_dt_spec pb_mcu= GPIO_DT_SPEC_GET(DT_ALIAS(pb_mcu), gpios);
const struct gpio_dt_spec buzz= GPIO_DT_SPEC_GET(DT_ALIAS(buzz), gpios);









/* Initialize all GPIOs */
static int init_gpios(void)
{
    int ret;
    
    /* Check if devices are ready */
    if (!device_is_ready(pulse_cathode.port)) {
        printk("Pulse cathode device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(pulse_anode.port)) {
        printk("Pulse anode device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(dc_dc_en.port)) {
        printk("DC-DC enable device not ready\n");
        return -ENODEV;
    }

    /* Configure pins as outputs, starting inactive */
    ret = gpio_pin_configure_dt(&pulse_cathode, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure pulse cathode (err %d)\n", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&pulse_anode, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure pulse anode (err %d)\n", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure DC-DC enable (err %d)\n", ret);
        return ret;
    }

    return 0;
}

#include <hal/nrf_power.h>

static void log_reset_reason(void)
{
    uint32_t reas = nrf_power_resetreas_get(NRF_POWER);

    if (reas & POWER_RESETREAS_RESETPIN_Msk) {
        LOG_INF("Reset: pin reset");
    }
    if (reas & POWER_RESETREAS_DOG_Msk) {
        LOG_INF("Reset: watchdog");
    }
    if (reas & POWER_RESETREAS_SREQ_Msk) {
        LOG_INF("Reset: soft reset");
    }
    if (reas & POWER_RESETREAS_LOCKUP_Msk) {
        LOG_INF("Reset: CPU lockup");
    }
    if (reas & POWER_RESETREAS_OFF_Msk) {
        LOG_INF("Reset: wakeup from OFF");
    }
    if (reas & POWER_RESETREAS_LPCOMP_Msk) {
        LOG_INF("Reset: LPCOMP event");
    }

    /* Očisti zastavice da ne bi ostale za sledeći put */
    nrf_power_resetreas_clear(NRF_POWER, reas);
}





void main(void)
{
    int err;
    gpio_pin_set_dt(&kill, 0); // Set KILL pin low to enable the device
    gpio_pin_set_dt(&pb_mcu, 0); // Set PB_MCU pin high (not pressed)
    log_reset_reason();

    /* === LED initialization === */
    if (!device_is_ready(led0.port)) {
        printk("LED0 device not ready\n");
        return;
    }

    

    if (gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE) < 0 ||
        gpio_pin_configure_dt(&kill, GPIO_OUTPUT_INACTIVE) < 0) { //
        printk("Failed to configure LEDs\n");
        return;
    }

    /* === Additional GPIO pins initialization === */
    if (init_gpios() != 0) {
        printk("GPIO initialization failed!\n");
        return;
    }
    gpio_pin_set_dt(&led0, 1);  // Turn on LED0 when connected

    /* === DC-DC converter activation === */
    gpio_pin_set_dt(&dc_dc_en, 1);
    printk("DC-DC converter enabled\n");
    k_sleep(K_MSEC(100));

    /* === BLE initialization === */
    err = ble_nus_init();
    if (err) {
        printk("BLE initialization failed (%d)\n", err);
        return;
    }


    /* === DAC and timer initialization === */
    dac_init();
    dac_set_value(80); // Default value
    gpio_pin_set_dt(&buzz, 1); // Ensure buzzer is off
    k_sleep(K_MSEC(5000));
    gpio_pin_set_dt(&buzz, 0); // Buzz for 5 seconds
    // debug_i2c_probe();
    // rsens_init();
    uint32_t ntc_voltages;
    /* === MUX initialization and sending initial data === */
    mux_init(&stim_mux_config);

    bq27220_init();   

    // start_nrfx_adc(); // Start the ADC sampling chain (PPI + Timer + SAADC)



    /* === Main loop === */
    while (1) {


        k_sleep(K_FOREVER);

    
    }
}
