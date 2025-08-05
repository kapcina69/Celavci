#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <string.h>
#include <stdlib.h>  
#include "ble_nus.h"
#include "dac.h"



// --- Callback when receive data from NUS ---
static void nus_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    printk("Received BLE message:\n");

    printk("  HEX: ");
    for (uint16_t i = 0; i < len; i++) {
        printk("%02X ", data[i]);
    }

    printk("\n  ASCII: ");
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
            printk("%c", data[i]);
        } else {
            printk(".");
        }
    }
    printk("\n");

    process_command(data, len);
}



void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth was not started successfully (err %d)\n", err);
        return;
    }

    printk("Bluetooth ready, starting advertising...\n");

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        printk("Advertising not started (err %d)\n", err);
    } else {
        printk("BLE advertising active\n");
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static struct bt_nus_cb nus_callbacks = {
    .received = nus_cb,
};

int ble_nus_init(void)
{
    int err;

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
        return err;
    }

    bt_conn_cb_register(&conn_callbacks);

    err = bt_nus_init(&nus_callbacks);
    if (err) {
        printk("NUS service not initialized (err %d)\n", err);
        return err;
    }

    return 0;
}

#define OK_MSG  "OK\n"
#define ERR_MSG "ERR\n"

volatile uint8_t amplitude = 10;
uint8_t frequency = 20;
uint8_t pulse_width = 5;
uint8_t temperature = 38;
uint8_t stim_state = 0;

void send_response(const char *msg) {
    bt_nus_send(NULL, msg, strlen(msg));
}



static void process_command(const uint8_t *data, uint16_t len) {
    if (len < 4 || data[2] != ';') {
        send_response(ERR_MSG);
        return;
    }

    char cmd[3] = {data[0], data[1], '\0'};

    // Parse the value after ';' as a DECIMAL number (1 to 3 digits)
    char val_str[4] = {0};
    int val_len = len - 3;
    if (val_len > 3) val_len = 3;  
    memcpy(val_str, &data[3], val_len);
    val_str[val_len] = '\0';

    int value = strtol(val_str, NULL, 10);  // decimal

    if (strcmp(cmd, "ST") == 0) {
        if (value >= 0 && value <= 3) {
            stim_state = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }

    } else if (strcmp(cmd, "SA") == 0) { // RSENS resistor is 270 ohms, so the upper value is 0.81V, and the lower value is 0.027V
        if (value >= 1 && value <= 30) {
            amplitude = (uint8_t)value;
            send_response(OK_MSG);
            dac_set_value(amplitude * 8); // For value 8 you get 0.027V, and for 240 you get 0.81V
        } else {
            send_response(ERR_MSG);
        }

    } else if (strcmp(cmd, "SF") == 0) {
        if (value >= 1 && value <= 40) {
            frequency = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }

    } else if (strcmp(cmd, "SW") == 0) {
        if (value >= 1 && value <= 10) {
            printk("Pulse width set to %d\n", value);
            pulse_width = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            printk("Invalid pulse width value: %d\n", value);
            send_response(ERR_MSG);
        }

    } else if (strcmp(cmd, "HT") == 0) {
        if (value >= 25 && value <= 42) {
            printk("Temperature set to %d\n", value);
            temperature = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            printk("Invalid temperature value: %d\n", value);
            send_response(ERR_MSG);
        }

    } else {
        send_response(ERR_MSG);
    }
}
