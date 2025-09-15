#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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
// static void nus_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
// {
//     printk("Received BLE message:\n");

//     printk("  HEX: ");
//     for (uint16_t i = 0; i < len; i++) {
//         printk("%02X ", data[i]);
//     }

//     printk("\n  ASCII: ");
//     for (uint16_t i = 0; i < len; i++) {
//         if (data[i] >= 32 && data[i] <= 126) {
//             printk("%c", data[i]);
//         } else {
//             printk(".");
//         }
//     }
//     printk("\n");

//     process_command(data, len);
// }


// Callback za aplikaciju

// --- Callback when receive data from NUS ---
static void nus_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    printk("Received BLE message:\n");

    /* HEX prikaz */
    printk("  HEX: ");
    for (uint16_t i = 0; i < len; i++) {
        printk("%02X ", data[i]);
    }
    printk("\n");

    /* ASCII prikaz sa specijalnim karakterima */
    printk("  ASCII: ");
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        
        // Prikaz specijalnih karaktera koji se koriste u komandama
        if (c == '>') {
            printk(">");
        } else if (c == '<') {
            printk("<");
        } else if (c == ';') {
            printk(";");
        } else if (c == '\r') {
            printk("\\r");
        } else if (c == '\n') {
            printk("\\n");
        } else if (c == '\t') {
            printk("\\t");
        } else if (c >= 32 && c <= 126) {
            printk("%c", (char)c);
        } else {
            printk("."); // Neštampajući karakteri
        }
    }
    printk("\n");

    // Proveri da li je komanda u formatu >XX;...<
    if (len >= 5 && data[0] == '>' && data[len-1] == '<') {
        printk("  Format: Valid command structure\n");
        
        // Pronađi ; u komandi
        const uint8_t *semicolon = memchr(data, ';', len);
        if (semicolon && semicolon > data && semicolon < data + len - 1) {
            // Izdvoji komandu (između > i ;)
            uint8_t cmd_len = semicolon - data - 1;
            if (cmd_len > 0 && cmd_len <= 3) {
                char cmd[4] = {0};
                memcpy(cmd, data + 1, cmd_len);
                printk("  Command: %s\n", cmd);
            }
            
            // Izdvoji argumente (između ; i <) - mogu biti binarni
            uint8_t arg_len = (data + len - 1) - semicolon - 1;
            if (arg_len > 0) {
                printk("  Arguments (hex): ");
                for (uint8_t i = 0; i < arg_len; i++) {
                    printk("%02X ", semicolon[1 + i]);
                }
                printk("\n");
                
                // Konvertuj binarnu vrednost u decimalnu (BIG-ENDIAN)
                uint32_t decimal_value = 0;
                
                if (arg_len == 2) {
                    // 16-bitna vrednost (BIG-ENDIAN: prvi bajt je MSB, drugi je LSB)
                    decimal_value = (semicolon[1] << 8) | semicolon[2];
                    printk("  Decimal value: %lu\n", decimal_value);
                }
                else if (arg_len == 1) {
                    // 8-bitna vrednost
                    decimal_value = semicolon[1];
                    printk("  Decimal value: %lu\n", decimal_value);
                }
                else if (arg_len == 4) {
                    // 32-bitna vrednost (BIG-ENDIAN)
                    decimal_value = (semicolon[1] << 24) | (semicolon[2] << 16) | 
                                   (semicolon[3] << 8) | semicolon[4];
                    printk("  Decimal value: %lu\n", decimal_value);
                }
                else {
                    // Tekstualni argumenti
                    char args[128] = {0};
                    if (arg_len < sizeof(args) - 1) {
                        memcpy(args, semicolon + 1, arg_len);
                        args[arg_len] = '\0';
                        printk("  Text arguments: %s\n", args);
                        
                        // Pokušaj da konvertuješ tekstualne brojeve
                        char *endptr;
                        long text_value = strtol(args, &endptr, 0);
                        if (endptr != args) {
                            printk("  Text numeric value: %ld\n", text_value);
                        }
                    }
                }
            }
        }
    } else {
        printk("  Format: Invalid command structure\n");
    }

    process_command(data, len);
}

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
uint8_t freq_start = 0;
uint8_t freq_end = 0;
uint8_t freq_dur =0;
bool freq_sweep;

/**
 * @brief Pošalji BLE poruku u formatu >XX;...< sa binarnim payload-om
 *
 * Primer:
 *   send_kv_raw2("PW", (uint8_t[]){0x01, 0xF4}, 2);
 *   -> šalje: 3E 50 57 3B 01 F4 3C   (ASCII ">PW;" + payload + "<")
 *
 * @param hdr2    Dva karaktera komande (npr. "PW", "SF", "XC")
 * @param payload Pokazivač na bajtove payload-a (može NULL ako plen=0)
 * @param plen    Broj bajtova payload-a
 */
static void send_kv_raw2(const char hdr2[2], const uint8_t *payload, size_t plen)
{
    uint8_t buf[64];
    size_t i = 0;

    buf[i++] = '>';
    buf[i++] = (uint8_t)hdr2[0];
    buf[i++] = (uint8_t)hdr2[1];
    buf[i++] = ';';

    if (payload && plen) {
        memcpy(&buf[i], payload, plen);
        i += plen;
    }

    buf[i++] = '<';

    /* slanje preko BLE NUS-a */
    int ret = bt_nus_send(NULL, buf, i);
    if (ret) {
        printk("[TX][ERR] bt_nus_send failed (%d)\n", ret);
    } else {
        printk("[TX] Sent %zu bytes: ", i);
        for (size_t k = 0; k < i; k++) {
            printk("%02X ", buf[k]);
        }
        printk("\n");
    }
}


#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static uint8_t max_freq_for_pw(uint16_t pw_us, uint8_t frequency_default)
{
    uint8_t candidate = frequency_default;

    if (pw_us >= 1000) {
        candidate = 64;
        printk("[max_freq_for_pw] pw=%u us >=1000 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 950) {
        candidate = 67;
        printk("[max_freq_for_pw] pw=%u us >=950 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 900) {
        candidate = 68;
        printk("[max_freq_for_pw] pw=%u us >=900 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 850) {
        candidate = 71;
        printk("[max_freq_for_pw] pw=%u us >=850 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 800) {
        candidate = 72;
        printk("[max_freq_for_pw] pw=%u us >=800 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 750) {
        candidate = 75;
        printk("[max_freq_for_pw] pw=%u us >=750 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 700) {
        candidate = 77;
        printk("[max_freq_for_pw] pw=%u us >=700 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 650) {
        candidate = 81;
        printk("[max_freq_for_pw] pw=%u us >=650 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 600) {
        candidate = 82;
        printk("[max_freq_for_pw] pw=%u us >=600 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 550) {
        candidate = 85;
        printk("[max_freq_for_pw] pw=%u us >=550 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 500) {
        candidate = 89;
        printk("[max_freq_for_pw] pw=%u us >=500 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 450) {
        candidate = 93;
        printk("[max_freq_for_pw] pw=%u us >=450 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if (pw_us >= 400) {
        candidate = 97;
        printk("[max_freq_for_pw] pw=%u us >=400 -> cand=%u Hz\n", pw_us, candidate);
    }
    else if(pw_us >= 350) {
        candidate = 99;
        printk("[max_freq_for_pw] pw=%u us >=399 -> cand=100 Hz\n", pw_us);
    }
    else {
        candidate = frequency_default; /* <400 µs → 1–100 Hz dozvoljeno */
        printk("[max_freq_for_pw] pw=%u us <400 -> cand=default=%u Hz\n", pw_us, candidate);
    }

    /* Vraćamo samo ako je kandidat manji od default-a */
    if (candidate < frequency_default) {
        printk("[max_freq_for_pw] USING candidate=%u Hz (default=%u)\n", candidate, frequency_default);
        return candidate;
    } else {
        printk("[max_freq_for_pw] KEEPING default=%u Hz (candidate=%u)\n", frequency_default, candidate);
        return frequency_default;
    }
}



static uint16_t min_pw_for_freq(uint8_t freq_hz, uint16_t default_pw_us)
{
    uint16_t candidate = default_pw_us;

    if (freq_hz <= 64) {
        candidate = 1000;
        printk("[min_pw_for_freq] freq=%u Hz <=66 -> cand=1000 us\n", freq_hz);
    }
    else if (freq_hz <= 67) {
        candidate = 950;
        printk("[min_pw_for_freq] freq=%u Hz <=69 -> cand=950 us\n", freq_hz);
    }
    else if (freq_hz <= 68) {
        candidate = 900;
        printk("[min_pw_for_freq] freq=%u Hz <=70 -> cand=900 us\n", freq_hz);
    }
    else if (freq_hz <= 71) {
        candidate = 850;
        printk("[min_pw_for_freq] freq=%u Hz <=73 -> cand=850 us\n", freq_hz);
    }
    else if (freq_hz <= 72) {
        candidate = 800;
        printk("[min_pw_for_freq] freq=%u Hz <=74 -> cand=800 us\n", freq_hz);
    }
    else if (freq_hz <= 75) {
        candidate = 750;
        printk("[min_pw_for_freq] freq=%u Hz <=77 -> cand=750 us\n", freq_hz);
    }
    else if (freq_hz <= 77) {
        candidate = 700;
        printk("[min_pw_for_freq] freq=%u Hz <=79 -> cand=700 us\n", freq_hz);
    }
    else if (freq_hz <= 81) {
        candidate = 650;
        printk("[min_pw_for_freq] freq=%u Hz <=83 -> cand=650 us\n", freq_hz);
    }
    else if (freq_hz <= 82) {
        candidate = 600;
        printk("[min_pw_for_freq] freq=%u Hz <=84 -> cand=600 us\n", freq_hz);
    }
    else if (freq_hz <= 85) {
        candidate = 550;
        printk("[min_pw_for_freq] freq=%u Hz <=88 -> cand=550 us\n", freq_hz);
    }
    else if (freq_hz <= 89) {
        candidate = 500;
        printk("[min_pw_for_freq] freq=%u Hz <=91 -> cand=500 us\n", freq_hz);
    }
    else if (freq_hz <= 93) {
        candidate = 450;
        printk("[min_pw_for_freq] freq=%u Hz <=95 -> cand=450 us\n", freq_hz);
    }
    else if (freq_hz <= 97) {
        candidate = 400;
        printk("[min_pw_for_freq] freq=%u Hz <=98 -> cand=400 us\n", freq_hz);
    }
    else if(freq_hz <= 99) {
        candidate = 350;
        printk("[min_pw_for_freq] freq=%u Hz <=100 -> cand=350 us (<400 zone)\n", freq_hz);
    }
    else if (freq_hz <= 100) {
        candidate = 399;
        printk("[min_pw_for_freq] freq=%u Hz <=100 -> cand=399 us (<400 zone)\n", freq_hz);
    }
    else {
        candidate = default_pw_us;
        printk("[min_pw_for_freq] freq=%u Hz >100 -> cand=default %u us\n", freq_hz, default_pw_us);
    }

    /* Vraćamo samo ako je kandidat manji od default-a */
    if (candidate < default_pw_us) {
        printk("[min_pw_for_freq] USING candidate=%u us (default=%u)\n", candidate, default_pw_us);
        return candidate;
    } else {
        printk("[min_pw_for_freq] KEEPING default=%u us (candidate=%u)\n", default_pw_us, candidate);
        return default_pw_us;
    }
}





// process command za aplikaciju
void freq_control_start(void);
void freq_control_stop(void);

static void process_command(const uint8_t *data, uint16_t len)
{
    /* 0) Validacija okvira i pronalazak granica */
    if (!data || len < 3 || data[0] != '>' || data[len-1] != '<') {
        printk("[CMD] Bad frame: missing '>'/'<' (len=%u)\n", len);
        send_response(">ERR;BAD_FRAME<\r\n");
        return;
    }

    /* 1) Nađi ';' (ako ga ima) i izvuci CMD (2–3 slova) */
    const uint8_t *semicolon = memchr(data, ';', len);
    const uint8_t *cmd_start = data + 1;
    const uint8_t *cmd_end   = (semicolon ? semicolon : (data + len - 1));
    size_t cmd_len = (cmd_end > cmd_start) ? (size_t)(cmd_end - cmd_start) : 0;
    if (cmd_len == 0 || cmd_len > 3) {
        printk("[CMD] BAD_CMD: cmd_len=%u\n", (unsigned)cmd_len);
        send_response(">ERR;BAD_CMD<\r\n");
        return;
    }

    char cmd[4] = {0};
    memcpy(cmd, cmd_start, cmd_len);
    for (size_t i = 0; i < cmd_len; i++) cmd[i] = (char)toupper((unsigned char)cmd[i]);
    printk("[CMD] Command='%s'\n", cmd);

    /* 2) Bez-arg komande (čisto ASCII) */
    if (!semicolon) {
        if (strcmp(cmd, "SON") == 0) {
            if (!stimulation_running) {
                stimulation_running = true;
                freq_control_start();
                start_pulse_sequence();
                printk("[CMD] SON -> start OK\n");
                send_response("SONOK\r\n");
            } else {
                printk("[CMD] SON -> already running (ERR)\n");
                send_response("SONERR\r\n");
            }
            return;
        }
        if (strcmp(cmd, "OFF") == 0) {
            if (stimulation_running) {
                stimulation_running = false;
                freq_control_stop();
                stop_pulse_sequence();
                printk("[CMD] OFF -> stop OK\n");
                send_response("OFFOK\r\n");
            } else {
                printk("[CMD] OFF -> already stopped (ERR)\n");
                send_response("OFFERR\r\n");
            }
            return;
        }
        if (strcmp(cmd, "RCE") == 0) {
            if (RCE == 0) {
                RCE = 1;
                printk("[CMD] RCE -> enabled for next 8 pulses\n");
                send_response(">RCE;OK<\r\n");
            } else {
                printk("[CMD] RCE -> BUSY already enabled\n");
                send_response(">RCE;BUSY<\r\n");
            }
            return;
        }
        if (strcmp(cmd, "RSC") == 0) {
            uint16_t soc = 0;
            int ret = fuel_gauge_get_soc(&soc);
            if (ret == 0) {
                char resp[16];
                snprintk(resp, sizeof(resp), ">RSC;%u<", soc);
                printk(">RSC;%u<", soc);
                send_response(resp);
            } else {
                printk("[CMD] RSC -> fuel_gauge_get_soc ERR=%d\n", ret);
                send_response(">RSCERR<");
            }
            return;
        }
        if (strcmp(cmd, "RSS") == 0) {
                /* 1) SON/OFF – bez payload-a */
                if (stimulation_running) {
                    printk("[CMD] RSS -> SON\n");
                    send_response(">SON<\r\n");   /* ostaje čisti ASCII bez payload-a */
                } else {
                    printk("[CMD] RSS -> OFF\n");
                    send_response(">OFF<\r\n");
                }

                /* 2) SA; 8 bajtova (aktivni pattern kao 8 vrednosti/bajtova) */
                if (number_patter < patterns_count) {
                    uint8_t sa_payload[8];
                    for (int i = 0; i < 8; ++i) {
                        /* ako su ti patterni uint16_t, ovde uzimaš npr. low byte */
                        uint16_t v = pulse_patterns[number_patter][i];
                        sa_payload[i] = (uint8_t)(v & 0xFF);
                    }
                    printk("[CMD] RSS -> SA for pattern #%u\n", (unsigned)number_patter);
                    send_kv_raw2("SA", sa_payload, sizeof(sa_payload));
                } else {
                    printk("[CMD] RSS -> SA ERR (pattern idx=%u >= count=%u)\n",
                        (unsigned)number_patter, (unsigned)patterns_count);
                    send_response(">SA;ERR<\r\n");
                }

                /* 3) SF; 3 bajta: start, end, dur (po novom protokolu) */
                {
                    uint8_t sf_payload[3];
                    /* constant: dur=0 i start==end; range: dur>0 */
                    sf_payload[0] = freq_start;
                    sf_payload[1] = freq_end;
                    sf_payload[2] = freq_dur;  /* sekunde (ili kako si definisao) */
                    printk("[CMD] RSS -> SF start=%u end=%u dur=%u\n",
                        sf_payload[0], sf_payload[1], sf_payload[2]);
                    send_kv_raw2("SF", sf_payload, sizeof(sf_payload));
                }

                /* 4) PW; 2 bajta (big-endian, 50..1000) */
                {
                    uint16_t pw = pulse_width; /* napomena: pulse_width mora biti uint16_t */
                    uint8_t pw_payload[2] = { (uint8_t)(pw >> 8), (uint8_t)(pw & 0xFF) };
                    printk("[CMD] RSS -> PW=%u\n", pw);
                    send_kv_raw2("PW", pw_payload, sizeof(pw_payload));
                }

                /* 5) ST; 1 bajt (trajanje u sekundama po novom pravilu) */
                {
                    uint8_t st_payload = (stim_duration_s > 255) ? 255 : (uint8_t)stim_duration_s;
                    printk("[CMD] RSS -> ST=%u s\n", st_payload);
                    send_kv_raw2("ST", &st_payload, 1);
                }

                /* 6) RSC; 2 bajta (raw hex iz fuel_gauge_get_soc) */
                {
                    uint16_t soc_raw = 0;
                    int ret = fuel_gauge_get_soc(&soc_raw);
                    if (ret == 0) {
                        uint8_t rsc_payload[2] = { (uint8_t)(soc_raw >> 8), (uint8_t)(soc_raw & 0xFF) };
                        printk("[CMD] RSS -> RSC raw=0x%04X\n", soc_raw);
                        send_kv_raw2("RSC", rsc_payload, sizeof(rsc_payload));
                    } else {
                        printk("[CMD] RSS -> RSC ERR=%d\n", ret);
                        send_response(">RSCERR<\r\n");
                    }
                }

                /* 7) RCE; 1 bajt (poslednja maska kontakata) */
                {
                    uint8_t mask = 0;
                    extern bool saadc_get_last_burst(uint8_t *out_mask);
                    if (saadc_get_last_burst(&mask)) {
                        printk("[CMD] RSS -> RCE mask=0x%02X\n", mask);
                        send_kv_raw2("RCE", &mask, 1);
                    } else {
                        send_response(">RCE;NA<\r\n");
                    }
                }

                return;
            }


        printk("[CMD] Unknown no-arg command '%s'\n", cmd);
        send_response(">ERR;UNKNOWN<\r\n");
        return;
    }

        /* 3) Argumenti: strogo između ';' i '<' (HEX bajtovi) */
    const uint8_t *lt_ptr = memchr(data, '<', len);
    if (!semicolon || !lt_ptr || semicolon >= lt_ptr) {
        printk("[CMD] BAD_FRAME: semicolon/lt mismatch\n");
        send_response(">ERR;BAD_FRAME<\r\n");
        return;
    }

    const uint8_t *arg_ptr = semicolon + 1;
    uint16_t arg_len = (uint16_t)(lt_ptr - arg_ptr); /* '<' NE ulazi */
    printk("[CMD] Arg len=%u\n", (unsigned)arg_len);

    /* === SF: uvek 3 bajta HEX: start(Hz), end(Hz), dur(s) === */
    if (strcmp(cmd, "SF") == 0) {
        if (arg_len != 3) {
            printk("[CMD] SF rejected: arg_len=%u (expected 3)\n", (unsigned)arg_len);
            send_response(">ERR<");
            return;
        }
        uint8_t start_v = max_freq_for_pw(pulse_width*20,arg_ptr[0]); //*20 zbog deljenja u pw komandi
        uint8_t end_v   = max_freq_for_pw(pulse_width*20,arg_ptr[1]);
        uint8_t dur_v   = arg_ptr[2];
        frequency = start_v;  // default
        printk("[CMD] SF RAW(3): start=%u, end=%u, dur=%u\n", start_v, end_v, dur_v);

        /* Validacija opsega 1..100 Hz */
        if (start_v < 1 || start_v > 100 || end_v < 1 || end_v > 100) {
            printk("[CMD] SF rejected: out of range (start=%u, end=%u)\n", start_v, end_v);
            send_response(">ERR<");
            return;
        }

        if (dur_v == 0) {
            if (start_v != end_v) {
                printk("[CMD] SF rejected: dur=0 but start!=end\n");
                send_response(">ERR<");
                return;
            }
            /* CONSTANT MODE */
            frequency      = start_v;
            new_frequency  = 1;
            freq_start     = start_v;
            freq_end       = end_v;
            freq_dur       = 0;
            freq_sweep     = false;
            freq_control_stop();
            printk("[CMD] SF applied: CONSTANT %u Hz\n", start_v);
            send_response(">OK<");
            return;
        } 
        else {
            /* RANGE MODE, zahtevamo start<=end (po potrebi dozvoli i obrnuto) */
            if (start_v > end_v) {
                printk("[CMD] SF rejected: start(%u) > end(%u)\n", start_v, end_v);
                send_response(">ERR<");
                return;
            }
            freq_start = start_v;
            freq_end   = end_v;
            freq_dur   = dur_v;
            freq_sweep = true;
                        freq_control_start();

            printk("[CMD] SF applied: RANGE %u..%u Hz, dur=%u s\n", start_v, end_v, dur_v);
            send_response(">OK<");
            return;
        }
    }

    /* === PW: 2 bajta HEX (16-bit big-endian), opseg 50..1000 === */
    if (strcmp(cmd, "PW") == 0) {
        if (arg_len != 2) {
            printk("[CMD] PW rejected: arg_len=%u (expected 2)\n", (unsigned)arg_len);
            send_response(">ERR<");
            return;
        }

        uint16_t v = ((uint16_t)arg_ptr[0] << 8) | (uint16_t)arg_ptr[1];
        printk("[CMD] PW RAW(2): %u\n", v);

        if (v >= 50 && v <= 1000) {
            /* Validacija u odnosu na frekvenciju */
            uint16_t min_pw = min_pw_for_freq(frequency, v);
            pulse_width = min_pw/20;
            printk("[CMD] PW applied: %u\n", min_pw);
            send_response(">OK<");
        } else {
            printk("[CMD] PW rejected: out-of-range %u\n", v);
            send_response(">ERR<");
        }
        return;
    }


    /* === SP: 1 bajt HEX – set active pattern index === */
    if (strcmp(cmd, "SP") == 0) {
        if (arg_len != 1) {
            printk("[CMD] SP rejected: arg_len=%u (expected 1)\n", (unsigned)arg_len);
            send_response("ERR: invalid pattern index\n");
            return;
        }
        uint8_t idx = arg_ptr[0];
        if ((size_t)idx < patterns_count) {
            number_patter = (size_t)idx;
            printk("[CMD] SP applied: active pattern=%u\n", idx);
            char ok[40]; snprintf(ok, sizeof(ok), "OK: active pattern=%u\n", idx);
            send_response(ok);
        } else {
            printk("[CMD] SP rejected: idx=%u >= count=%u\n", idx, (unsigned)patterns_count);
            send_response("ERR: invalid pattern index\n");
        }
        return;
    }

    /* === ST: 1 bajt HEX – trajanje u sekundama (1..255) === */
    if (strcmp(cmd, "ST") == 0) {
        if (arg_len != 1) {
            printk("[CMD] ST rejected: arg_len=%u (expected 1)\n", (unsigned)arg_len);
            send_response(">ST;ERR<\r\n");
            return;
        }
        uint8_t sec = arg_ptr[0];
        if (sec >= 1 /*&& sec <= 255*/) {
            stim_duration_s = (uint32_t)sec;
            printk("[CMD] ST applied: %u s\n", sec);
            send_response(">ST;OK<\r\n");
        } else {
            printk("[CMD] ST rejected: out-of-range %u\n", sec);
            send_response(">ST;ERR<\r\n");
        }
        return;
    }

    /* === SA: 16 bajtova HEX – upiši u aktivni pattern kao 8 vrednosti === */
    else if (strcmp(cmd, "SA") == 0) {
        if (arg_len != 16) {
            printk("[CMD] SA rejected: arg_len=%u (expected 16)\n", (unsigned)arg_len);
            send_response("ERR: need exactly 16 bytes\n");
            return;
        }
        if (number_patter >= patterns_count) {
            printk("[CMD] SA rejected: active pattern idx=%u out of range (count=%u)\n",
                (unsigned)number_patter, (unsigned)patterns_count);
            send_response("ERR: invalid active pattern\n");
            return;
        }

        for (int i = 0; i < 8; ++i) {
            /* Objedini 2 bajta u jedan uint16_t */
            uint16_t raw = ((uint16_t)arg_ptr[2*i] << 8) | arg_ptr[2*i + 1];

            pulse_patterns[number_patter][i] = raw;

            printk("[CMD] SA ch%d=0x%04X\n", i, raw);
        }

        printk("[CMD] SA applied to pattern #%u\n", (unsigned)number_patter);

        /* Odgovor preko BLE sa HEX vrednostima */
        char out[128];
        size_t pos = 0;
        pos += snprintk(out + pos, sizeof(out) - pos, ">SA;");
        for (int i = 0; i < 8; ++i) {
            pos += snprintk(out + pos, sizeof(out) - pos, "0x%04X ", pulse_patterns[number_patter][i]);
        }
        pos += snprintk(out + pos, sizeof(out) - pos, "<\r\n");

        send_response(out);
        return;
    }


    else if (strcmp(cmd, "XC") == 0) {
        if (arg_len != 16) {
            printk("[CMD] XC rejected: arg_len=%u (expected 16)\n", (unsigned)arg_len);
            send_response(">XC;ERR<\r\n");
            return;
        }

        for (int i = 0; i < 8; ++i) {
            /* Objedini po 2 bajta u jedan uint16_t */
            uint16_t raw = ((uint16_t)arg_ptr[2*i] << 8) | arg_ptr[2*i + 1];

            /* Skaliraj vrednost: deljenje sa 10 (npr. 3100 -> 310) */
            uint16_t val = raw / 10;

            /* Granice u µA – prilagodi po potrebi */
            if (val < 5 || val > 2000) {
                printk("[CMD] XC rejected: ch%d=%u out-of-range\n", i, val);
                send_response(">XC;ERR<\r\n");
                return;
            }

            pair_amplitude_uA[i] = val;
        }

        printk("[CMD] XC applied\n");

        /* Ispis u decimalnom obliku */
        char out[128];
        size_t pos = 0;
        pos += snprintk(out + pos, sizeof(out) - pos, ">XC;");
        for (int i = 0; i < 8; ++i) {
            pos += snprintk(out + pos, sizeof(out) - pos, "%u ", pair_amplitude_uA[i]);
        }
        pos += snprintk(out + pos, sizeof(out) - pos, "<\r\n");

        send_response(out);
        return;
    }




    /* Ako je stigla neka druga komanda sa argumentima koju nismo definisali po novom protokolu */
    printk("[CMD] Unknown command with fixed-HEX args '%s' (len=%u)\n", cmd, (unsigned)arg_len);
    send_response(">ERR;UNKNOWN<\r\n");

   
}










#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(freq_sweep, LOG_LEVEL_INF);



/* Trenutno stanje */
static uint8_t cur_freq = 0;
static int dir = +1;  /* smer promena: +1 ili -1 */

/* --- Helper: postavi frekvenciju --- */
static void set_frequency_safe(uint8_t f_hz)
{
    frequency     = f_hz;
    new_frequency = 1;
    LOG_INF("[FREQ] set %u Hz", f_hz);
}

/* --- Work koji menja frekvenciju --- */
static void freq_work_handler(struct k_work *work);
K_WORK_DEFINE(freq_work, freq_work_handler);

/* --- Tajmer koji trigeruje work --- */
static void freq_timer_handler(struct k_timer *timer)
{
    k_work_submit(&freq_work);
}
K_TIMER_DEFINE(freq_timer, freq_timer_handler, NULL);

/* --- Work logika (menjanje frekvencije) --- */
static void freq_work_handler(struct k_work *work)
{
    if (!stimulation_running) {
        /* Ako stimulacija nije ON, ne radi ništa */
        return;
    }

    uint8_t start = freq_start;
    uint8_t end   = freq_end;
    uint8_t dur_s = freq_dur;

    /* Normalizacija */
    if (start < 1) start = 1;
    if (start > 100) start = 100;
    if (end   < 1) end   = 1;
    if (end   > 100) end = 100;

    if (!freq_sweep || dur_s == 0) {
        /* CONSTANT MODE */
        set_frequency_safe(start);
        /* Ne menjamo ništa dalje */
        return;
    }

    /* Ako prvi put ulazimo, inicijalizuj */
    if (cur_freq == 0) {
        cur_freq = start;
        dir = (start <= end) ? +1 : -1;
    }

    /* Postavi trenutnu vrednost */
    set_frequency_safe(cur_freq);

    /* Izračunaj sledeću */
    if (dir > 0) {
        if (cur_freq < end) cur_freq++;
        else cur_freq = start;  /* wrap */
    } else {
        if (cur_freq > end) cur_freq--;
        else cur_freq = start;  /* wrap */
    }

    /* Restart tajmera za sledeći step */
    k_timer_start(&freq_timer, K_SECONDS(dur_s), K_NO_WAIT);
}

/* --- API koji pokreće sweep ili constant --- */
void freq_control_start(void)
{
    cur_freq = 0;  /* reset */
    if (!freq_sweep || freq_dur == 0) {
        /* Constant: odmah setuj frekvenciju */
        set_frequency_safe(freq_start);
    } else {
        /* Sweep: pokreni tajmer */
        k_timer_start(&freq_timer, K_NO_WAIT, K_NO_WAIT);
    }
}

void freq_control_stop(void)
{
    k_timer_stop(&freq_timer);
    cur_freq = 0;
}




/**
 * @brief Validira i postavlja novu frekvenciju i širinu impulsa.
 *
 * @param freq_hz       Tražena frekvencija [Hz]
 * @param pulse_width_us Širina impulsa [µs]
 * @return int  0 = OK, -EINVAL = van granica
 */
int set_freq_and_pw(uint8_t freq_hz, uint16_t pulse_width_us)
{
    /* Tabela maksimalnih frekvencija po širini impulsa */
    struct {
        uint16_t pw_us;
        uint8_t  max_hz;
    } limits[] = {
        {1000, 66},
        { 950, 69},
        { 900, 70},
        { 850, 73},
        { 800, 74},
        { 750, 77},
        { 700, 79},
        { 650, 83},
        { 600, 84},
        { 550, 88},
        { 500, 91},
        { 450, 95},
        { 400, 98},
    };

    uint8_t allowed_max = 100;

    if (pulse_width_us >= 1000) {
        allowed_max = 66;
    } else if (pulse_width_us < 400) {
        allowed_max = 100; /* 1–100 dozvoljeno */
    } else {
        for (int i = 0; i < ARRAY_SIZE(limits); i++) {
            if (pulse_width_us >= limits[i].pw_us) {
                allowed_max = limits[i].max_hz;
                break;
            }
        }
    }

    if (freq_hz > allowed_max) {
        LOG_ERR("[FREQ] REJECT: freq=%u Hz, pw=%u us (max=%u Hz)",
                freq_hz, pulse_width_us, allowed_max);
        return -EINVAL;
    }

    /* Ako je sve u redu -> upiši globalne promenljive */
    frequency     = freq_hz;
    pulse_width   = pulse_width_us;
    new_frequency = 1;

    LOG_INF("[FREQ] ACCEPT: freq=%u Hz, pw=%u us", freq_hz, pulse_width_us);
    return 0;
}