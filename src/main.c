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
#include "impulse.h"
#include "ble_nus.h"
#include "dac.h"
#include "mux.h"
#include "rsens.h"


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
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

// --- Deklaracije funkcija ---
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

// Bluetooth callback structure (implementation in ble_nus.c)
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

// --- BT events ---
void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connection failed (err %u)\n", err);
    } else {
        printk("BLE connected\n");
        gpio_pin_set_dt(&led1, 1);  // Turn on LED0 when connected
        
    }
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE disconnected (reason %u)\n", reason);
    gpio_pin_set_dt(&led1, 0);  // Turn off LED0 when disconnected
}




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





void main(void)
{
    int err;

    /* === LED initialization === */
    if (!device_is_ready(led0.port)) {
        printk("LED0 device not ready\n");
        return;
    }

    if (!device_is_ready(led1.port)) {
        printk("LED1 device not ready\n");
        return;
    }

    if (gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE) < 0 ||
        gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE) < 0) {
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

    bt_conn_cb_register(&conn_callbacks);

    /* === DAC and timer initialization === */
    dac_init();
    dac_set_value(80); // Default value
    k_sleep(K_MSEC(5000));

    rsens_init();
    uint32_t ntc_voltages;
    /* === MUX initialization and sending initial data === */
    mux_init(&stim_mux_config);
    // start_pulse_sequence(); 
    /* === Main loop === */
    while (1) {

        // izbaci: uint32_t ntc_voltages;
        k_sleep(K_MSEC(1000));
    
    }
}
