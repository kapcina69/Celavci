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
#include "impulse.h"

#define PATTERN_LEN   8
#define MAX_PATTERNS  16

/* Globalne promenljive iz impulse.c */
extern uint16_t pulse_patterns[MAX_PATTERNS][PATTERN_LEN];
extern volatile size_t patterns_count;
extern volatile size_t number_patter;

uint8_t RCE = 0;


bool stimulation_running = false;

static bool is_dec_uint(const char *s) {
    if (!s || !*s) return false;
    while (*s) {
        if (!isdigit((unsigned char)*s)) return false;
        s++;
    }
    return true;
}



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


volatile uint8_t amplitude = 100;
uint8_t frequency = 20;
uint8_t pulse_width = 25;
uint8_t temperature = 38;
uint8_t stim_state = 0;

uint8_t new_frequency = 0;

static void process_command(const uint8_t *data, uint16_t len)
{
    char msg[160];
    size_t cpy = (len < sizeof(msg) - 1) ? len : (sizeof(msg) - 1);
    memcpy(msg, data, cpy);
    msg[cpy] = '\0';

    /* --- Trim trailing \r\n belina --- */
    for (int i = (int)strlen(msg) - 1;
         i >= 0 && isspace((unsigned char)msg[i]);
         --i) {
        msg[i] = '\0';
    }
    /* SON – Start stimulation */
    if (strcmp(msg, ">SON<") == 0) {
        if (!stimulation_running) {
            stimulation_running = true;
            start_pulse_sequence();
            send_response("SONOK\r\n");
        } else {
            send_response("SONERR\r\n");
        }
        return;  
    }

    if (strcmp(msg, ">OFF<") == 0) {
        if (stimulation_running) {
            stimulation_running = false;
            stop_pulse_sequence();
            send_response("OFFOK\r\n");
        } else {
            send_response("OFFERR\r\n");
        }
        return;  
    }
        /* RCE – pokreni čitanje kontakata (bez argumenata) */
    if (strcmp(msg, ">RCE<") == 0) {
        if (RCE == 0) {
            RCE = 1;                   // omogući režim čitanja za sledećih 8 impulsa
            send_response(">RCE;OK<\r\n");
        } else {
            send_response(">RCE;BUSY<\r\n");
        }
        return;
    }
    if (strcmp(msg, ">RSC<") == 0) {
        uint16_t soc = 0;
        int ret = fuel_gauge_get_soc(&soc);   // wrapper funkcija u fuel_gauge.c
        if (ret == 0) {
            char resp[16];
            snprintk(resp, sizeof(resp), ">RSC;%u<", soc);
            send_response(resp);
        } else {
            send_response(">RSCERR<");
        }
        return;
    }

        /* RSTAT – vrati kompletno trenutno stanje (7 set + baterija + kontakti) */
    if (strcmp(msg, ">RSS<") == 0) {
        /* 1) SON/OFF */
        if (stimulation_running) {
            send_response(">SON<\r\n");
        } else {
            send_response(">OFF<\r\n");
        }

                /* 2) SA; v1 v2 ... v8 (decimal, kao u SA komandi) */
        {
            if (number_patter < patterns_count) {
                char line[160];
                char *p = line;
                p += snprintk(p, sizeof(line) - (p - line), ">SA;");
                for (int i = 0; i < PATTERN_LEN; i++) {
                    uint16_t v = pulse_patterns[number_patter][i];
                    p += snprintk(p, sizeof(line) - (p - line),
                                  (i < PATTERN_LEN - 1) ? "0x%04X " : "0x%04X",
                                  v);
                }
                p += snprintk(p, sizeof(line) - (p - line), "<\r\n");
                send_response(line);
            } else {
                send_response(">SA;ERR<\r\n");
            }
        }


        /* 3) SF;x */
        {
            char line[24];
            snprintk(line, sizeof(line), ">SF;%u<\r\n", (unsigned)frequency);
            send_response(line);
        }

        /* 4) PW;x */
        {
            char line[24];
            snprintk(line, sizeof(line), ">PW;%u<\r\n", (unsigned)pulse_width);
            send_response(line);
        }

        /* 5) SC;x  (koristimo aktivni pattern index / šemu) */
        // {
        //     char line[24];
        //     snprintk(line, sizeof(line), ">SC;%u<\r\n", (unsigned)number_patter);
        //     send_response(line);
        // }

        /* 6) ST;x  (trajanje stimulacije u sekundama) */
        {
            char line[24];
            snprintk(line, sizeof(line), ">ST;%u<\r\n", (unsigned)stim_duration_s);
            send_response(line);
        }

        /* 7) RSC;x  (State of Charge u %) */
        {
            uint16_t soc = 0;
            int ret = fuel_gauge_get_soc(&soc);
            if (ret == 0) {
                char line[24];
                snprintk(line, sizeof(line), ">RSC;%u<\r\n", (unsigned)soc);
                send_response(line);
            } else {
                send_response(">RSCERR<\r\n");
            }
        }

        /* 8) RCE;xx  (kontakt elektroda – poslednja poznata 8-bit maska) */
        {
            uint8_t mask = 0;
            /* Ako imaš util iz SAADC podsistema: */
            extern bool saadc_get_last_burst(uint8_t *out_mask);
            if (saadc_get_last_burst(&mask)) {
                char line[24];
                snprintk(line, sizeof(line), ">RCE;%02X<\r\n", mask);
                send_response(line);
            } else {
                /* Nema sveže maske – prijavi poslednju ili ERR */
                send_response(">RCE;NA<\r\n");
            }
        }

        return;
    }


    /* === Parsiranje komande u formatu >XX;ARGUMENTI< === */
    char cmd[4] = {0};          /* podržava 2–3 slova, npr. "SA", "RSH" */
    const char *arg = NULL;     /* pokazivač na argumente bez završnog '<' */
    char arg_local[128] = {0};  /* lokalni bafer za argumente (od ';' do '<') */

    const char *gt = strchr((const char *)msg, '>');
    const char *lt = gt ? strchr(gt, '<') : NULL;     /* kraj poruke */
    const char *sc = gt ? strchr(gt, ';') : NULL;     /* razdvajanje cmd/arg */

    if (!gt) {
        send_response(">ERR;NO_START<\r\n");
        return;
    }
    if (!lt) {
        send_response(">ERR;NO_END<\r\n");
        return;
    }

    /* Komanda je između '>' i prvog ';' ili '<' (šta prvo naiđe) */
    const char *cmd_end = (sc && sc < lt) ? sc : lt;
    size_t clen = (size_t)(cmd_end - (gt + 1));
    if (clen == 0 || clen > 3) {
        send_response(">ERR;BAD_CMD<\r\n");
        return;
    }
    memcpy(cmd, gt + 1, clen);
    cmd[clen] = '\0';

    /* Argumenti su između ';' i '<' (ako ima ';'), inače nema argumenata */
    if (sc && sc < lt) {
        size_t alen = (size_t)(lt - (sc + 1));
        if (alen >= sizeof(arg_local)) alen = sizeof(arg_local) - 1;
        memcpy(arg_local, sc + 1, alen);
        arg_local[alen] = '\0';
    }
    arg = arg_local;

    /* --- Dalje tvoj postojeći kod ostaje isti: koristi 'cmd' i 'arg' --- */


    /* ============================================================
    * Komanda SP – Set Pattern
    * Format: SP;0xAABB 0xCCDD ... (8 vrednosti)
    * ============================================================ */
    if (strcmp(cmd, "SA") == 0) { // (tvoja trenutna logika ostaje)
        uint16_t tmp[PATTERN_LEN];
        int found = 0;

        char buf[128];
        size_t n = strnlen(arg, sizeof(buf) - 1);
        memcpy(buf, arg, n);
        buf[n] = '\0';

        for (char *p = strtok(buf, " \t");
            p != NULL && found < PATTERN_LEN;
            p = strtok(NULL, " \t")) {
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

    /* ============================================================
    * Komanda SM – Set active pattern
    * Format: SM;x (decimal ili hex 0x..)
    * ============================================================ */
    if (strcmp(cmd, "SP") == 0) { // (tvoja trenutna logika ostaje)
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

    /* ============================================================
    * Komanda XC – Set current amplitude per each channel
    * Format: XC;x1 x2 x3 x4 x5 x6 x7 x8  (decimal 5–200)
    * ============================================================ */
    else if (strcmp(cmd, "XC") == 0) {
        char buf[128];

        size_t n = strnlen(arg, sizeof(buf) - 1);
        memcpy(buf, arg, n);
        buf[n] = '\0';

        for (char *s = buf; *s; ++s) {
            if (*s == ',') *s = ' ';
        }

        uint16_t tmp_vals[8];
        int count = 0;

        char *saveptr = NULL;
        for (char *tok = strtok_r(buf, " \t", &saveptr);
            tok && count < 8;
            tok = strtok_r(NULL, " \t", &saveptr)) {

            char *endp = NULL;
            long v = strtol(tok, &endp, 10);

            if (endp == tok || *endp != '\0' || v < 5 || v > 200) {
                send_response(">SC;ERR<\r\n");
                return;
            }
            tmp_vals[count++] = (uint16_t)v;
        }

        if (count != 8) {
            send_response(">SC;ERR<\r\n");
            return;
        }

        for (int i = 0; i < 8; ++i) {
            pair_amplitude_uA[i] = tmp_vals[i];
        }

        send_response(">SC;OK<\r\n");
        return;
    }


    /* ============================================================
    * Ostale kratke komande oblika "XX;DECIMAL"
    * ============================================================ */
    int value = (int)strtol(arg, NULL, 10);

    /* SF – Set frequency */
    if (strcmp(cmd, "SF") == 0) {
        if (value >= 1 && value <= 100) {
            frequency = (uint8_t)value;
            new_frequency = 1;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }
    }

    /* PW – Set pulse width */
    else if (strcmp(cmd, "PW") == 0) {
        if (value >= 5 && value <= 100) {
            pulse_width = (uint8_t)value;
            send_response(OK_MSG);
        } else {
            send_response(ERR_MSG);
        }
    }

    else if (strcmp(cmd, "ST") == 0) {
        long v = strtol(arg, NULL, 10);
        if (v >= 1 && v <= 36000) {
            stim_duration_s = (uint32_t)v;
            send_response(">ST;OK<\r\n");
        } else {
            send_response(">ST;ERR<\r\n");
        }
        return;
    }

    /* Nepoznata komanda */
    else {
        send_response(ERR_MSG);
    }

}