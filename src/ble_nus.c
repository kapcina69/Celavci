#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "ble_nus.h"
#include "dac.h"

#define PATTERN_LEN   8
#define MAX_PATTERNS  16

/* Globalne promenljive iz impulse.c */
extern uint16_t pulse_patterns[MAX_PATTERNS][PATTERN_LEN];
extern volatile size_t patterns_count;
extern volatile size_t number_patter;

/* Helper: parsiraj heks token u uint16_t (dozvoljen prefiks 0x) */
static bool parse_hex16(const char *tok, uint16_t *out)
{
    if (!tok || !out) return false;
    while (*tok && isspace((unsigned char)*tok)) tok++;   // preskoči beline

    unsigned long v = 0;
    if ((tok[0] == '0') && (tok[1] == 'x' || tok[1] == 'X')) {
        tok += 2;
    }

    char *endp = NULL;
    v = strtoul(tok, &endp, 16);
    if (endp == tok)    return false;     // nema cifara
    if (v > 0xFFFFUL)   return false;     // izvan 16-bit
    *out = (uint16_t)v;
    return true;
}




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

void send_response(const char *msg) {
    bt_nus_send(NULL, msg, strlen(msg));
}


volatile uint8_t amplitude = 10;
uint8_t frequency = 20;
uint8_t pulse_width = 5;
uint8_t temperature = 38;
uint8_t stim_state = 0;

static void process_command(const uint8_t *data, uint16_t len)
{
    /* Napravi lokalni, nul-terminiran string od RX bajtova */
    char msg[160];
    size_t cpy = (len < sizeof(msg) - 1) ? len : (sizeof(msg) - 1);
    memcpy(msg, data, cpy);
    msg[cpy] = '\0';

    /* Trim trailing \r\n belina (opciono) */
    for (int i = (int)strlen(msg) - 1; i >= 0 && isspace((unsigned char)msg[i]); --i) {
        msg[i] = '\0';
    }

    /* Provera osnovnog formata "XX;..." */
    if (strlen(msg) < 3 || msg[2] != ';') {
        send_response(ERR_MSG);
        return;
    }

    char cmd[3] = { msg[0], msg[1], '\0' };
    const char *arg = msg + 3;  // deo posle ';'

    /* === Novi set komandi: SP i SM === */

    /* SP;0xAABB 0xCCDD ... (8 vrednosti) */
    if (strcmp(cmd, "SP") == 0) {
        uint16_t tmp[PATTERN_LEN];
        int found = 0;

        // Napravi kopiju argumenata jer ćemo tokenizovati po belinama
        char buf[128];
        size_t n = strnlen(arg, sizeof(buf) - 1);
        memcpy(buf, arg, n);
        buf[n] = '\0';

        for (char *p = strtok(buf, " \t"); p != NULL && found < PATTERN_LEN; p = strtok(NULL, " \t")) {
            uint16_t v;
            if (!parse_hex16(p, &v)) {
                send_response("ERR: bad hex in SP\n");
                return;
            }
            tmp[found++] = v;
        }

        if (found != PATTERN_LEN) {
            send_response("ERR: need exactly 8 values\n");
            return;
        }

        if (patterns_count >= MAX_PATTERNS) {
            send_response("ERR: patterns full\n");
            return;
        }

        /* Upis u kolekciju */
        for (int i = 0; i < PATTERN_LEN; ++i) {
            pulse_patterns[patterns_count][i] = tmp[i];
        }
        size_t idx_added = patterns_count;
        patterns_count++;

        char ok[48];
        snprintf(ok, sizeof(ok), "OK: SP stored as #%u\n", (unsigned)idx_added);
        send_response(ok);
        return;
    }

    /* SM;x  — set active pattern (x može biti dec ili hex "0x...") */
    if (strcmp(cmd, "SM") == 0) {
        // baza 0 -> prihvata 10/16 (npr. "7" ili "0x7")
        long v = strtol(arg, NULL, 0);
        if (v < 0 || (size_t)v >= patterns_count) {
            send_response("ERR: invalid pattern index\n");
            return;
        }
        number_patter = (size_t)v;

        char ok[40];
        snprintf(ok, sizeof(ok), "OK: active pattern=%ld\n", v);
        send_response(ok);
        return;
    }

    /* === Postojeće kratke komande oblika "XX;DECIMAL" === */
    /* Izvuci decimalnu vrednost posle ';' (do kraja) */
    int value = (int)strtol(arg, NULL, 10);

    if (strcmp(cmd, "ST") == 0) {
        if (value >= 0 && value <= 3) {
            stim_state = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }

    } else if (strcmp(cmd, "SA") == 0) {
        if (value >= 1 && value <= 30) {
            amplitude = (uint8_t)value;
            send_response(OK_MSG);
            dac_set_value(amplitude * 8); // 8->0.027V, 240->0.81V (kao komentar u tvom kodu)
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
            pulse_width = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }

    } else if (strcmp(cmd, "HT") == 0) {
        if (value >= 25 && value <= 42) {
            temperature = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }

    } else {
        send_response(ERR_MSG);
    }
}
