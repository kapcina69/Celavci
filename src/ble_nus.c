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


// Callback za aplikaciju

// // --- Callback when receive data from NUS ---
// static void nus_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
// {
//     printk("Received BLE message:\n");

//     /* HEX prikaz */
//     printk("  HEX: ");
//     for (uint16_t i = 0; i < len; i++) {
//         printk("%02X ", data[i]);
//     }
//     printk("\n");

//     /* ASCII prikaz sa specijalnim karakterima */
//     printk("  ASCII: ");
//     for (uint16_t i = 0; i < len; i++) {
//         uint8_t c = data[i];
        
//         // Prikaz specijalnih karaktera koji se koriste u komandama
//         if (c == '>') {
//             printk(">");
//         } else if (c == '<') {
//             printk("<");
//         } else if (c == ';') {
//             printk(";");
//         } else if (c == '\r') {
//             printk("\\r");
//         } else if (c == '\n') {
//             printk("\\n");
//         } else if (c == '\t') {
//             printk("\\t");
//         } else if (c >= 32 && c <= 126) {
//             printk("%c", (char)c);
//         } else {
//             printk("."); // Neštampajući karakteri
//         }
//     }
//     printk("\n");

//     // Proveri da li je komanda u formatu >XX;...<
//     if (len >= 5 && data[0] == '>' && data[len-1] == '<') {
//         printk("  Format: Valid command structure\n");
        
//         // Pronađi ; u komandi
//         const uint8_t *semicolon = memchr(data, ';', len);
//         if (semicolon && semicolon > data && semicolon < data + len - 1) {
//             // Izdvoji komandu (između > i ;)
//             uint8_t cmd_len = semicolon - data - 1;
//             if (cmd_len > 0 && cmd_len <= 3) {
//                 char cmd[4] = {0};
//                 memcpy(cmd, data + 1, cmd_len);
//                 printk("  Command: %s\n", cmd);
//             }
            
//             // Izdvoji argumente (između ; i <) - mogu biti binarni
//             uint8_t arg_len = (data + len - 1) - semicolon - 1;
//             if (arg_len > 0) {
//                 printk("  Arguments (hex): ");
//                 for (uint8_t i = 0; i < arg_len; i++) {
//                     printk("%02X ", semicolon[1 + i]);
//                 }
//                 printk("\n");
                
//                 // Konvertuj binarnu vrednost u decimalnu (BIG-ENDIAN)
//                 uint32_t decimal_value = 0;
                
//                 if (arg_len == 2) {
//                     // 16-bitna vrednost (BIG-ENDIAN: prvi bajt je MSB, drugi je LSB)
//                     decimal_value = (semicolon[1] << 8) | semicolon[2];
//                     printk("  Decimal value: %lu\n", decimal_value);
//                 }
//                 else if (arg_len == 1) {
//                     // 8-bitna vrednost
//                     decimal_value = semicolon[1];
//                     printk("  Decimal value: %lu\n", decimal_value);
//                 }
//                 else if (arg_len == 4) {
//                     // 32-bitna vrednost (BIG-ENDIAN)
//                     decimal_value = (semicolon[1] << 24) | (semicolon[2] << 16) | 
//                                    (semicolon[3] << 8) | semicolon[4];
//                     printk("  Decimal value: %lu\n", decimal_value);
//                 }
//                 else {
//                     // Tekstualni argumenti
//                     char args[128] = {0};
//                     if (arg_len < sizeof(args) - 1) {
//                         memcpy(args, semicolon + 1, arg_len);
//                         args[arg_len] = '\0';
//                         printk("  Text arguments: %s\n", args);
                        
//                         // Pokušaj da konvertuješ tekstualne brojeve
//                         char *endptr;
//                         long text_value = strtol(args, &endptr, 0);
//                         if (endptr != args) {
//                             printk("  Text numeric value: %ld\n", text_value);
//                         }
//                     }
//                 }
//             }
//         }
//     } else {
//         printk("  Format: Invalid command structure\n");
//     }

//     process_command(data, len);
// }

/* --- Konekcioni parametri (stabilniji protiv reason 0x08) --- */
static const struct bt_le_conn_param desired_param = {
    .interval_min = 24,   /* 30 ms */
    .interval_max = 40,   /* 50 ms */
    .latency      = 0,
    .timeout      = 400,  /* 4 s */
};

static struct bt_conn *current_conn;

static void request_conn_param(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!current_conn) return;

    int err = bt_conn_le_param_update(current_conn, &desired_param);
    if (err) {
        printk("bt_conn_le_param_update err=%d, retry in 3s\n", err);
        /* ako central odbije, probaj opet kasnije */
        static struct k_work_delayable *self;
        self = CONTAINER_OF(work, struct k_work_delayable, work);
        k_work_reschedule(self, K_SECONDS(3));
    } else {
        printk("Conn param update requested\n");
    }
}

/* Odloženi rad za traženje parametara posle povezivanja */
static K_WORK_DELAYABLE_DEFINE(conn_param_work, request_conn_param);

/* --- Deklaracije funkcija (ostaju) --- */
void bt_ready(int err);
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

/* --- LED (led1 alias) --- */
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* --- BT events --- */
void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connection failed (err %u)\n", err);
        return;
    }

    /* Zapamti konekciju za NUS TX */
    current_conn = bt_conn_ref(conn);

    /* Info o trenutnim parametrima */
    struct bt_conn_info info;
    if (!bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE) {
        printk("BLE connected: interval=%u*1.25ms latency=%u timeout=%u*10ms\n",
               info.le.interval, info.le.latency, info.le.timeout);
    } else {
        printk("BLE connected\n");
    }

    /* LED ON */
    gpio_pin_set_dt(&led1, 1);

    /* Posle kratkog kašnjenja zatraži „zdravije” parametre */
    k_work_schedule(&conn_param_work, K_SECONDS(2));
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE disconnected (reason 0x%02X)\n", reason);
    /* 0x08 = Connection Timeout */

    /* LED OFF */
    gpio_pin_set_dt(&led1, 0);

    /* Oslobodi i počisti state */
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    k_work_cancel_delayable(&conn_param_work);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static struct bt_nus_cb nus_callbacks = {
    .received = nus_cb,
};

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

int ble_nus_init(void)
{
    int err;

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
        return err;
    }

    if (!device_is_ready(led1.port)) {
        printk("LED1 device not ready\n");
        return -ENODEV;
    }
    /* Pravilno konfiguriši LED kao izlaz i ugasi je */
    err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    if (err) {
        printk("LED1 config failed (err %d)\n", err);
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

/* --- Slanje odgovora preko NUS-a (koristi aktivnu konekciju) --- */
#define OK_MSG  "OK\n"
#define ERR_MSG "ERR\n"

void send_response(const char *msg)
{
    if (!current_conn || !msg) {
        printk("NUS TX skipped (no conn or msg=NULL)\n");
        return;
    }
    int err = bt_nus_send(current_conn, (const uint8_t *)msg, strlen(msg));
    if (err) {
        printk("bt_nus_send err=%d\n", err);
    }
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


// process command za aplikaciju


// static void process_command(const uint8_t *data, uint16_t len)
// {
//     /* 0) Validacija okvira i pronalazak granica */
//     if (!data || len < 3 || data[0] != '>' || data[len-1] != '<') {
//         printk("[CMD] Bad frame: missing '>'/'<' (len=%u)\n", len);
//         send_response(">ERR;BAD_FRAME<\r\n");
//         return;
//     }

//     /* 1) Nađi ';' (ako ga ima) i izvuci CMD (2–3 slova) */
//     const uint8_t *semicolon = memchr(data, ';', len);
//     const uint8_t *cmd_start = data + 1;
//     const uint8_t *cmd_end   = (semicolon ? semicolon : (data + len - 1));
//     size_t cmd_len = (cmd_end > cmd_start) ? (size_t)(cmd_end - cmd_start) : 0;
//     if (cmd_len == 0 || cmd_len > 3) {
//         printk("[CMD] BAD_CMD: cmd_len=%u\n", (unsigned)cmd_len);
//         send_response(">ERR;BAD_CMD<\r\n");
//         return;
//     }

//     char cmd[4] = {0};
//     memcpy(cmd, cmd_start, cmd_len);
//     for (size_t i = 0; i < cmd_len; i++) cmd[i] = (char)toupper((unsigned char)cmd[i]);
//     printk("[CMD] Command='%s'\n", cmd);

//     /* 2) Bez-arg komande (čisto ASCII) */
//     if (!semicolon) {
//         if (strcmp(cmd, "SON") == 0) {
//             if (!stimulation_running) {
//                 stimulation_running = true;
//                 start_pulse_sequence();
//                 printk("[CMD] SON -> start OK\n");
//                 send_response("SONOK\r\n");
//             } else {
//                 printk("[CMD] SON -> already running (ERR)\n");
//                 send_response("SONERR\r\n");
//             }
//             return;
//         }
//         if (strcmp(cmd, "OFF") == 0) {
//             if (stimulation_running) {
//                 stimulation_running = false;
//                 stop_pulse_sequence();
//                 printk("[CMD] OFF -> stop OK\n");
//                 send_response("OFFOK\r\n");
//             } else {
//                 printk("[CMD] OFF -> already stopped (ERR)\n");
//                 send_response("OFFERR\r\n");
//             }
//             return;
//         }
//         if (strcmp(cmd, "RCE") == 0) {
//             if (RCE == 0) {
//                 RCE = 1;
//                 printk("[CMD] RCE -> enabled for next 8 pulses\n");
//                 send_response(">RCE;OK<\r\n");
//             } else {
//                 printk("[CMD] RCE -> BUSY already enabled\n");
//                 send_response(">RCE;BUSY<\r\n");
//             }
//             return;
//         }
//         if (strcmp(cmd, "RSC") == 0) {
//             uint16_t soc = 0;
//             int ret = fuel_gauge_get_soc(&soc);
//             if (ret == 0) {
//                 char resp[16];
//                 snprintk(resp, sizeof(resp), ">RSC;%u<", soc);
//                 printk("[CMD] RSC -> %u%%\n", soc);
//                 send_response(resp);
//             } else {
//                 printk("[CMD] RSC -> fuel_gauge_get_soc ERR=%d\n", ret);
//                 send_response(">RSCERR<");
//             }
//             return;
//         }
//         if (strcmp(cmd, "RSS") == 0) {
//             /* 1) SON/OFF */
//             if (stimulation_running) { printk("[CMD] RSS -> SON\n"); send_response(">SON<\r\n"); }
//             else                     { printk("[CMD] RSS -> OFF\n"); send_response(">OFF<\r\n"); }

//             /* 2) SA;... */
//             if (number_patter < patterns_count) {
//                 char line[160]; char *p = line;
//                 p += snprintk(p, sizeof(line)-(p-line), ">SA;");
//                 for (int i = 0; i < PATTERN_LEN; i++) {
//                     uint16_t v = pulse_patterns[number_patter][i];
//                     p += snprintk(p, sizeof(line)-(p-line),
//                                   (i < PATTERN_LEN-1) ? "0x%04X " : "0x%04X", v);
//                 }
//                 p += snprintk(p, sizeof(line)-(p-line), "<\r\n");
//                 printk("[CMD] RSS -> SA for pattern #%u\n", (unsigned)number_patter);
//                 send_response(line);
//             } else {
//                 printk("[CMD] RSS -> SA ERR (pattern idx=%u >= count=%u)\n",
//                        (unsigned)number_patter, (unsigned)patterns_count);
//                 send_response(">SA;ERR<\r\n");
//             }

//             /* 3) SF, 4) PW, 6) ST */
//             { char line[24]; snprintk(line, sizeof(line), ">SF;%u<\r\n", (unsigned)frequency); send_response(line); }
//             { char line[24]; snprintk(line, sizeof(line), ">PW;%u<\r\n", (unsigned)pulse_width); send_response(line); }
//             { char line[24]; snprintk(line, sizeof(line), ">ST;%u<\r\n", (unsigned)stim_duration_s); send_response(line); }

//             /* 7) RSC */
//             {   uint16_t soc = 0;
//                 int ret = fuel_gauge_get_soc(&soc);
//                 if (ret == 0) { char line[24]; snprintk(line, sizeof(line), ">RSC;%u<\r\n", (unsigned)soc); send_response(line); }
//                 else          { send_response(">RSCERR<\r\n"); }
//             }

//             /* 8) RCE */
//             {   uint8_t mask = 0;
//                 extern bool saadc_get_last_burst(uint8_t *out_mask);
//                 if (saadc_get_last_burst(&mask)) { char line[24]; snprintk(line, sizeof(line), ">RCE;%02X<\r\n", mask); send_response(line); }
//                 else                              { send_response(">RCE;NA<\r\n"); }
//             }
//             return;
//         }

//         printk("[CMD] Unknown no-arg command '%s'\n", cmd);
//         send_response(">ERR;UNKNOWN<\r\n");
//         return;
//     }

//     /* 3) Imamo argumente: između ';' i '<' (mogu biti binarni ili tekst) */
//     const uint8_t *arg_ptr = semicolon + 1;
//     uint8_t arg_len = (uint8_t)((data + len - 1) - arg_ptr); /* bez '<' */
//     printk("[CMD] Arg len=%u\n", arg_len);

//     /* 3a) Ako su binarni, ponašaj se kao u tvom callback-u:
//      *     - 1 bajt  => 8-bit, decimal
//      *     - 2 bajta => 16-bit BIG-ENDIAN, decimal
//      *     - 4 bajta => 32-bit BIG-ENDIAN, decimal
//      *     - ostalo  => pokušaj kao tekst (ASCII broj, base 0)
//      *
//      *  DODATNI ZAHTEV (po tvom insistiranju ranije):
//      *  Ako stigne više bajtova koji nisu čisti tekst, mi za komande sa jednom vrednošću
//      *  koristimo pre svega decimalnu vrednost iz ove konverzije.
//      */
//     bool have_numeric = false;
//     uint32_t dec_value = 0;

//     if (arg_len == 1) {
//         dec_value = arg_ptr[0];
//         have_numeric = true;
//         printk("[CMD] Numeric(8) = %lu\n", dec_value);
//     } else if (arg_len == 2) {
//         dec_value = ((uint32_t)arg_ptr[0] << 8) | (uint32_t)arg_ptr[1];
//         have_numeric = true;
//         printk("[CMD] Numeric(16 BE) = %lu\n", dec_value);
//     } else if (arg_len == 4) {
//         dec_value = ((uint32_t)arg_ptr[0] << 24) | ((uint32_t)arg_ptr[1] << 16) |
//                     ((uint32_t)arg_ptr[2] << 8)  |  (uint32_t)arg_ptr[3];
//         have_numeric = true;
//         printk("[CMD] Numeric(32 BE) = %lu\n", dec_value);
//     } else {
//         /* Proveri da li je čisto tekstualno (printable) pa probaj strtol */
//         bool printable = true;
//         for (uint8_t i = 0; i < arg_len; i++) {
//             uint8_t c = arg_ptr[i];
//             if (c == '\r' || c == '\n' || c == '\t') continue;
//             if (c < 32 || c > 126) { printable = false; break; }
//         }
//         if (printable && arg_len > 0) {
//             char args[128] = {0};
//             size_t copy = arg_len < sizeof(args)-1 ? arg_len : sizeof(args)-1;
//             memcpy(args, arg_ptr, copy);
//             args[copy] = '\0';
//             char *endp = NULL;
//             long tv = strtol(args, &endp, 0); /* dozvoljava 123, 0x1F, ... */
//             if (endp != args) {
//                 dec_value = (uint32_t)tv;
//                 have_numeric = true;
//                 printk("[CMD] Numeric(TEXT) = %ld\n", tv);
//             } else {
//                 printk("[CMD] TEXT args='%s' (non-numeric)\n", args);
//             }
//         } else {
//             printk("[CMD] Non-printable multi-byte args (len=%u) -> no numeric\n", arg_len);
//         }
//     }

//     /* 4) Jednopoljne komande koje koriste upravo ovu decimalnu vrednost */
//     if (strcmp(cmd, "SF") == 0) {
//         uint32_t sf_val = 0;
//         bool valid = false;

//         if (arg_len >= 1) {
//             /* Specijalno: uzimamo SAMO prvi bajt kao decimalnu vrednost */
//             sf_val = (uint32_t)arg_ptr[0];
//             valid = true;
//             printk("[CMD] SF special: using only first byte 0x%02X -> %lu\n",
//                 arg_ptr[0], sf_val);
//         } else {
//             printk("[CMD] SF: no argument bytes\n");
//         }

//         if (!valid) {
//             send_response(ERR_MSG);
//             return;
//         }

//         if (sf_val >= 1 && sf_val <= 100) {
//             frequency = (uint8_t)sf_val;
//             new_frequency = 1;
//             printk("[CMD] SF applied: %lu Hz\n", sf_val);
//             send_response(OK_MSG);
//         } else {
//             printk("[CMD] SF rejected: out-of-range %lu\n", sf_val);
//             send_response(ERR_MSG);
//         }
//         return;
//     }


//     if (strcmp(cmd, "PW") == 0) {
//         if (!have_numeric) { printk("[CMD] PW: no numeric value\n"); send_response(ERR_MSG); return; }
//         if (dec_value >= 50 && dec_value <= 1000) {
//             pulse_width = (uint8_t)dec_value;
//             printk("[CMD] PW applied: %lu\n", dec_value);
//             send_response(OK_MSG);
//         } else {
//             printk("[CMD] PW rejected: out-of-range %lu\n", dec_value);
//             send_response(ERR_MSG);
//         }
//         return;
//     }

//     if (strcmp(cmd, "SC") == 0) {
//         if (!have_numeric) { printk("[CMD] SC: no numeric value\n"); send_response(">SC;ERR<\r\n"); return; }
//         if ((size_t)dec_value < patterns_count) {
//             number_patter = (size_t)dec_value;
//             printk("[CMD] SC applied: active pattern=%lu\n", dec_value);
//             send_response(">SC;OK<\r\n");
//         } else {
//             printk("[CMD] SC rejected: idx=%lu >= count=%u\n", dec_value, (unsigned)patterns_count);
//             send_response(">SC;ERR<\r\n");
//         }
//         return;
//     }

//     if (strcmp(cmd, "ST") == 0) {
//         if (!have_numeric) { printk("[CMD] ST: no numeric value\n"); send_response(">ST;ERR<\r\n"); return; }
//         if (dec_value >= 1 && dec_value <= 36000) {
//             stim_duration_s = (uint32_t)dec_value;
//             printk("[CMD] ST applied: %lu s\n", dec_value);
//             send_response(">ST;OK<\r\n");
//         } else {
//             printk("[CMD] ST rejected: out-of-range %lu\n", dec_value);
//             send_response(">ST;ERR<\r\n");
//         }
//         return;
//     }

//     /* 5) Višeargumentne ASCII komande (SA/SP/XC) — očekuju tekstualne argumente */
//     if (strcmp(cmd, "SA") == 0) {
//         if (arg_len == 0) { printk("[CMD] SA: no args\n"); send_response("ERR: need 8 values\n"); return; }
//         char buf[128];
//         size_t copy = arg_len < sizeof(buf)-1 ? arg_len : sizeof(buf)-1;
//         memcpy(buf, arg_ptr, copy);
//         buf[copy] = '\0';

//         uint16_t tmp[PATTERN_LEN]; int found = 0;
//         for (char *p = strtok(buf, " \t"); p && found < PATTERN_LEN; p = strtok(NULL, " \t")) {
//             uint16_t v;
//             if (!parse_hex16(p, &v)) { printk("[CMD] SA: bad hex '%s'\n", p); send_response("ERR: bad hex in SP\n"); return; }
//             tmp[found++] = v;
//         }

//         if (found != PATTERN_LEN)            { printk("[CMD] SA: need 8 values, got %d\n", found); send_response("ERR: need exactly 8 values\n"); return; }
//         if (patterns_count >= MAX_PATTERNS)  { printk("[CMD] SA: patterns full\n"); send_response("ERR: patterns full\n"); return; }

//         for (int i = 0; i < PATTERN_LEN; ++i) pulse_patterns[patterns_count][i] = tmp[i];
//         size_t idx_added = patterns_count++;
//         printk("[CMD] SA stored as #%u\n", (unsigned)idx_added);
//         char ok[48]; snprintf(ok, sizeof(ok), "OK: SP stored as #%u\n", (unsigned)idx_added);
//         send_response(ok);
//         return;
//     }

//     if (strcmp(cmd, "SP") == 0) {
//         if (arg_len == 0) { printk("[CMD] SP: no arg\n"); send_response("ERR: invalid pattern index\n"); return; }
//         char buf[32];
//         size_t copy = arg_len < sizeof(buf)-1 ? arg_len : sizeof(buf)-1;
//         memcpy(buf, arg_ptr, copy);
//         buf[copy] = '\0';

//         long v = strtol(buf, NULL, 0);
//         if (v < 0 || (size_t)v >= patterns_count) {
//             printk("[CMD] SP rejected: idx=%ld >= count=%u\n", v, (unsigned)patterns_count);
//             send_response("ERR: invalid pattern index\n");
//             return;
//         }
//         number_patter = (size_t)v;
//         printk("[CMD] SP applied: active pattern=%ld\n", v);
//         char ok[40]; snprintf(ok, sizeof(ok), "OK: active pattern=%ld\n", v);
//         send_response(ok);
//         return;
//     }

//     if (strcmp(cmd, "XC") == 0) {
//         if (arg_len == 0) { printk("[CMD] XC: no args\n"); send_response(">SC;ERR<\r\n"); return; }
//         char buf[128];
//         size_t copy = arg_len < sizeof(buf)-1 ? arg_len : sizeof(buf)-1;
//         memcpy(buf, arg_ptr, copy);
//         buf[copy] = '\0';
//         for (char *s = buf; *s; ++s) if (*s == ',') *s = ' ';

//         uint16_t tmp_vals[8]; int count = 0; char *saveptr = NULL;
//         for (char *tok = strtok_r(buf, " \t", &saveptr); tok && count < 8; tok = strtok_r(NULL, " \t", &saveptr)) {
//             char *endp = NULL; long v = strtol(tok, &endp, 10);
//             if (endp == tok || *endp != '\0' || v < 5 || v > 200) {
//                 printk("[CMD] XC rejected token='%s'\n", tok);
//                 send_response(">SC;ERR<\r\n");
//                 return;
//             }
//             tmp_vals[count++] = (uint16_t)v;
//         }
//         if (count != 8) { printk("[CMD] XC: need 8 values, got %d\n", count); send_response(">SC;ERR<\r\n"); return; }
//         for (int i = 0; i < 8; ++i) pair_amplitude_uA[i] = tmp_vals[i];
//         printk("[CMD] XC applied\n");
//         send_response(">SC;OK<\r\n");
//         return;
//     }

//     printk("[CMD] Unknown command with args '%s'\n", cmd);
//     send_response(">ERR;UNKNOWN<\r\n");
// }
