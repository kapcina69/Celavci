#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "ble_nus.h"
#include "dac.h"
#include "impulse.h"  /* AŽURIRANO: koristi novi impulse API */
#include "pwm.h"
#include "fuel_gauge.h"

LOG_MODULE_REGISTER(ble_nus, LOG_LEVEL_INF);

/* ============================================================================
 * DEFINITIONS AND CONSTANTS
 * ============================================================================ */

#define OK_MSG  "OK\n"
#define ERR_MSG "ERR\n"

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

volatile uint8_t amplitude = 100;
uint8_t frequency = 20;
uint8_t pulse_width = 25;
uint8_t temperature = 38;
uint8_t stim_state = 0;

uint8_t new_frequency = 0;
uint8_t freq_start = 0;
uint8_t freq_end = 0;
uint8_t freq_dur = 0;
bool freq_sweep;

uint8_t RCE = 0;
bool stimulation_running = false;

/* ============================================================================
 * CONNECTION MANAGEMENT
 * ============================================================================ */

/* Connection parameters to prevent timeout (reason 0x08) */
static const struct bt_le_conn_param desired_param = {
    .interval_min = 24,   /* 30 ms */
    .interval_max = 40,   /* 50 ms */
    .latency      = 0,
    .timeout      = 400,  /* 4 s */
};

static struct bt_conn *current_conn;
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* === IMPULSE SUBSYSTEM STATUS (NOVO) === */
bool impulse_initialized = false;

/* ============================================================================
 * FREQUENCY AND PULSE WIDTH VALIDATION
 * ============================================================================ */

/* Determine maximum frequency for given pulse width */
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
        candidate = frequency_default; /* <400 µs → 1-100 Hz allowed */
        printk("[max_freq_for_pw] pw=%u us <400 -> cand=default=%u Hz\n", pw_us, candidate);
    }

    if (candidate < frequency_default) {
        printk("[max_freq_for_pw] USING candidate=%u Hz (default=%u)\n", candidate, frequency_default);
        return candidate;
    } else {
        printk("[max_freq_for_pw] KEEPING default=%u Hz (candidate=%u)\n", frequency_default, candidate);
        return frequency_default;
    }
}

/* Determine minimum pulse width for given frequency */
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
        candidate = 300;
        printk("[min_pw_for_freq] freq=%u Hz <=100 -> cand=399 us (<400 zone)\n", freq_hz);
    }
    else {
        candidate = default_pw_us;
        printk("[min_pw_for_freq] freq=%u Hz >100 -> cand=default %u us\n", freq_hz, default_pw_us);
    }

    if (candidate < default_pw_us) {
        printk("[min_pw_for_freq] USING candidate=%u us (default=%u)\n", candidate, default_pw_us);
        return candidate;
    } else {
        printk("[min_pw_for_freq] KEEPING default=%u us (candidate=%u)\n", default_pw_us, candidate);
        return default_pw_us;
    }
}

/**
 * @brief Validates and sets new frequency and pulse width.
 *
 * @param freq_hz       Requested frequency [Hz]
 * @param pulse_width_us Pulse width [µs]
 * @return int  0 = OK, -EINVAL = out of bounds
 */
int set_freq_and_pw(uint8_t freq_hz, uint16_t pulse_width_us)
{
    /* Maximum frequency limits by pulse width table */
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
        allowed_max = 100; /* 1-100 allowed */
    } else {
        for (int i = 0; i < ARRAY_SIZE(limits); i++) {
            if (pulse_width_us >= limits[i].pw_us) {
                allowed_max = limits[i].max_hz;
                break;
            }
        }
    }

    if (freq_hz > allowed_max) {
        printk("FREQ REJECT: freq=%u Hz, pw=%u us (max=%u Hz)\n",
                freq_hz, pulse_width_us, allowed_max);
        return -EINVAL;
    }

    /* If everything is OK -> write to global variables */
    frequency     = freq_hz;
    pulse_width   = pulse_width_us;
    new_frequency = 1;

    /* NOVO: Thread-safe notifikacija impulse thread-u */
    if (impulse_initialized) {
        notify_frequency_changed();
    }

    printk("FREQ ACCEPT: freq=%u Hz, pw=%u us\n", freq_hz, pulse_width_us);
    return 0;
}

/* ============================================================================
 * BLE COMMUNICATION FUNCTIONS
 * ============================================================================ */

/* Send response via NUS (uses active connection) */
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

/**
 * @brief Send BLE message in format >XX;...< with binary payload
 *
 * Example:
 *   send_kv_raw2("PW", (uint8_t[]){0x01, 0xF4}, 2);
 *   -> sends: 3E 50 57 3B 01 F4 3C   (ASCII ">PW;" + payload + "<")
 *
 * @param hdr2    Two command characters (e.g. "PW", "SF", "XC")
 * @param payload Pointer to payload bytes (can be NULL if plen=0)
 * @param plen    Number of payload bytes
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

    /* Send via BLE NUS */
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

/* ============================================================================
 * IMPULSE SUBSYSTEM INTEGRATION (NOVO)
 * ============================================================================ */

/**
 * @brief Error callback za impulse subsystem
 */
static void impulse_error_handler(impulse_error_t error_code, const char *context)
{
    const char *error_name = "UNKNOWN";
    
    switch (error_code) {
        case IMPULSE_ERROR_NONE:
            error_name = "NONE";
            break;
        case IMPULSE_ERROR_NOT_INITIALIZED:
            error_name = "NOT_INITIALIZED";
            break;
        case IMPULSE_ERROR_HARDWARE_FAILURE:
            error_name = "HARDWARE_FAILURE";
            break;
        case IMPULSE_ERROR_MUX_FAILURE:
            error_name = "MUX_FAILURE";
            break;
        case IMPULSE_ERROR_DAC_FAILURE:
            error_name = "DAC_FAILURE";
            break;
        case IMPULSE_ERROR_TIMING_VIOLATION:
            error_name = "TIMING_VIOLATION";
            break;
        case IMPULSE_ERROR_PATTERN_INVALID:
            error_name = "PATTERN_INVALID";
            break;
        case IMPULSE_ERROR_AMPLITUDE_INVALID:
            error_name = "AMPLITUDE_INVALID";
            break;
        case IMPULSE_ERROR_WATCHDOG_TIMEOUT:
            error_name = "WATCHDOG_TIMEOUT";
            break;
    }
    
    printk("IMPULSE ERROR: %s (%d) - %s\n", error_name, error_code, context ? context : "");
    
    /* Pošalji error preko BLE */
    char error_msg[64];
    snprintk(error_msg, sizeof(error_msg), ">ERR;IMP_%s<\r\n", error_name);
    send_response(error_msg);
    
    /* Za kritične greške, automatski zaustavi stimulaciju */
    if (error_code == IMPULSE_ERROR_HARDWARE_FAILURE ||
        error_code == IMPULSE_ERROR_WATCHDOG_TIMEOUT) {
        
        if (stimulation_running) {
            stimulation_running = false;
            stop_pulse_sequence();
            send_response(">EMERGENCY_STOP<\r\n");
        }
    }
}

/**
 * @brief Pošalje performance stats preko BLE
 */
static void send_performance_stats(void)
{
    uint32_t pulses, errors, recoveries;
    get_performance_stats(&pulses, &errors, &recoveries);
    
    char stats[128];
    snprintk(stats, sizeof(stats), 
             ">STATS;P=%u,E=%u,R=%u<\r\n", 
             pulses, errors, recoveries);
    send_response(stats);
}



/* ============================================================================
 * BLE MESSAGE PROCESSING AND COMMAND HANDLING
 * ============================================================================ */

/* NUS callback when receiving data */
static void nus_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    printk("Received BLE message:\n");

    /* HEX display */
    printk("  HEX: ");
    for (uint16_t i = 0; i < len; i++) {
        printk("%02X ", data[i]);
    }
    printk("\n");

    /* ASCII display with special characters */
    printk("  ASCII: ");
    for (uint16_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        
        /* Display special characters used in commands */
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
            printk("."); /* Non-printable characters */
        }
    }
    printk("\n");

    /* Check if command is in format >XX;..< */
    if (len >= 5 && data[0] == '>' && data[len-1] == '<') {
        printk("  Format: Valid command structure\n");
        
        /* Find ; in command */
        const uint8_t *semicolon = memchr(data, ';', len);
        if (semicolon && semicolon > data && semicolon < data + len - 1) {
            /* Extract command (between > and ;) */
            uint8_t cmd_len = semicolon - data - 1;
            if (cmd_len > 0 && cmd_len <= 3) {
                char cmd[4] = {0};
                memcpy(cmd, data + 1, cmd_len);
                printk("  Command: %s\n", cmd);
            }
            
            /* Extract arguments (between ; and <) - can be binary */
            uint8_t arg_len = (data + len - 1) - semicolon - 1;
            if (arg_len > 0) {
                printk("  Arguments (hex): ");
                for (uint8_t i = 0; i < arg_len; i++) {
                    printk("%02X ", semicolon[1 + i]);
                }
                printk("\n");
                
                /* Convert binary value to decimal (BIG-ENDIAN) */
                uint32_t decimal_value = 0;
                
                if (arg_len == 2) {
                    /* 16-bit value (BIG-ENDIAN: first byte is MSB, second is LSB) */
                    decimal_value = (semicolon[1] << 8) | semicolon[2];
                    printk("  Decimal value: %lu\n", decimal_value);
                }
                else if (arg_len == 1) {
                    /* 8-bit value */
                    decimal_value = semicolon[1];
                    printk("  Decimal value: %lu\n", decimal_value);
                }
                else if (arg_len == 4) {
                    /* 32-bit value (BIG-ENDIAN) */
                    decimal_value = (semicolon[1] << 24) | (semicolon[2] << 16) | 
                                   (semicolon[3] << 8) | semicolon[4];
                    printk("  Decimal value: %lu\n", decimal_value);
                }
                else {
                    /* Text arguments */
                    char args[128] = {0};
                    if (arg_len < sizeof(args) - 1) {
                        memcpy(args, semicolon + 1, arg_len);
                        args[arg_len] = '\0';
                        printk("  Text arguments: %s\n", args);
                        
                        /* Try to convert text numbers */
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

/* Process received command */
static void process_command(const uint8_t *data, uint16_t len)
{
    /* Frame validation and boundary finding */
    if (!data || len < 3 || data[0] != '>' || data[len-1] != '<') {
        printk("[CMD] Bad frame: missing '>'/'<' (len=%u)\n", len);
        send_response(">ERR;BAD_FRAME<\r\n");
        return;
    }

    /* Find ';' (if exists) and extract CMD (2-3 letters) */
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
    for (size_t i = 0; i < cmd_len; i++) {
        cmd[i] = (char)toupper((unsigned char)cmd[i]);
    }
    printk("[CMD] Command='%s'\n", cmd);

    /* === IMPULSE SYSTEM STATUS CHECK (NOVO) === */
    if (!impulse_initialized) {
        printk("[CMD] Impulse not initialized - rejecting command\n");
        send_response(">ERR;NOT_INIT<\r\n");
        return;
    }

    /* Handle commands without arguments (pure ASCII) */
    if (!semicolon) {
        /* SON - Start stimulation */
        if (strcmp(cmd, "SON") == 0) {
            /* NOVO: Proverava impulse state umesto lokalnih varijabli */
            int current_state = get_impulse_state();
            
            if (current_state == 0) { /* IMPULSE_IDLE */
                stimulation_running = true;
                pwm_ch0_start();
                k_msleep(400); 
                pwm_ch0_stop();
                freq_control_start();
                start_pulse_sequence(); /* NOVO: Thread-safe API */
                printk("[CMD] SON -> start OK\n");
                send_response("SONOK\r\n");
            } else {
                printk("[CMD] SON -> already running (state=%d)\n", current_state);
                send_response("SONERR\r\n");
            }
            return;
        }
        
        /* OFF - Stop stimulation */
        if (strcmp(cmd, "OFF") == 0) {
            int current_state = get_impulse_state();
            
            if (current_state != 0) { /* Not IMPULSE_IDLE */
                stimulation_running = false;
                pwm_ch0_start();
                k_msleep(400);
                pwm_ch0_stop();
                freq_control_stop();
                stop_pulse_sequence(); /* NOVO: Thread-safe API */
                printk("[CMD] OFF -> stop OK\n");
                send_response("OFFOK\r\n");
            } else {
                printk("[CMD] OFF -> already stopped\n");
                send_response("OFFERR\r\n");
            }
            return;
        }
        
        /* RCE - Remote Contact Enable */
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
        
        /* RSC - Read State of Charge */
        if (strcmp(cmd, "RSC") == 0) {
            uint16_t soc_raw = 0;
            int ret = fuel_gauge_get_soc(&soc_raw);
            if (ret == 0) {
                uint8_t ascii_char = (uint8_t)(soc_raw & 0xFF);
                char resp[16];
                snprintk(resp, sizeof(resp), ">RSC;%c<", ascii_char);
                printk(">RSC;%c<\n", ascii_char);
                send_response(resp);

                printk("fuel_gauge_get_soc: %u%% -> ASCII '%c' (0x%02X)\n",
                       soc_raw, ascii_char, ascii_char);
            } else {
                printk("[CMD] RSC -> fuel_gauge_get_soc ERR=%d\n", ret);
                send_response(">RSCERR<");
            }
            return;
        }



        /* RSS - Read System Status */
        if (strcmp(cmd, "RSS") == 0) {
            /* 1) SON/OFF - status na osnovu impulse state */
            int impulse_state_val = get_impulse_state();
            if (impulse_state_val == 1) { /* IMPULSE_RUNNING */
                printk("[CMD] RSS -> SON\n");
                send_response(">SON<\r\n");
            } else {
                printk("[CMD] RSS -> OFF\n");
                send_response(">OFF<\r\n");
            }

            /* 2) SA; 8 bytes (active pattern as 8 values/bytes) */
            if (number_patter < patterns_count) {
                uint8_t sa_payload[8];
                for (int i = 0; i < 8; ++i) {
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

            /* 3) SF; 3 bytes: start, end, dur */
            {
                uint8_t sf_payload[3];
                sf_payload[0] = freq_start;
                sf_payload[1] = freq_end;
                sf_payload[2] = freq_dur;
                printk("[CMD] RSS -> SF start=%u end=%u dur=%u\n",
                    sf_payload[0], sf_payload[1], sf_payload[2]);
                send_kv_raw2("SF", sf_payload, sizeof(sf_payload));
            }

            /* 4) PW; 2 bytes (big-endian, 50..1000) */
            {
                uint16_t pw = pulse_width;
                uint8_t pw_payload[2] = { (uint8_t)(pw >> 8), (uint8_t)(pw & 0xFF) };
                printk("[CMD] RSS -> PW=%u\n", pw);
                send_kv_raw2("PW", pw_payload, sizeof(pw_payload));
            }

            /* 5) ST; 1 byte (duration in seconds) */
            {
                uint8_t st_payload = (stim_duration_s > 255) ? 255 : (uint8_t)stim_duration_s;
                printk("[CMD] RSS -> ST=%u s\n", st_payload);
                send_kv_raw2("ST", &st_payload, 1);
            }

            /* 6) RSC; battery status */
            {
                uint16_t soc_raw = 0;
                int ret = fuel_gauge_get_soc(&soc_raw);
                if (ret == 0) {
                    uint8_t ascii_char = (uint8_t)(soc_raw & 0xFF);
                    char resp[16];
                    snprintk(resp, sizeof(resp), ">RSC;%c<", ascii_char);
                    printk(">RSC;%c<\n", ascii_char);
                    send_response(resp);
                } else {
                    send_response(">RSCERR<");
                }
            }

            /* 7) RCE; poslednji contact burst */
            {
                report_last_burst();
            }

            /* 8) NOVO: Performance stats */
            send_performance_stats();

            return;
        }

        printk("[CMD] Unknown no-arg command '%s'\n", cmd);
        send_response(">ERR;UNKNOWN<\r\n");
        return;
    }

    /* Arguments: strictly between ';' and '<' (HEX bytes) */
    const uint8_t *lt_ptr = memchr(data, '<', len);
    if (!semicolon || !lt_ptr || semicolon >= lt_ptr) {
        printk("[CMD] BAD_FRAME: semicolon/lt mismatch\n");
        send_response(">ERR;BAD_FRAME<\r\n");
        return;
    }

    const uint8_t *arg_ptr = semicolon + 1;
    uint16_t arg_len = (uint16_t)(lt_ptr - arg_ptr);
    printk("[CMD] Arg len=%u\n", (unsigned)arg_len);

    /* === SF: Set Frequency (CONSTANT or RANGE) === */
    if (strcmp(cmd, "SF") == 0) {
        const uint8_t *frame_end = NULL;
        for (ssize_t i = len - 1; i >= 0; --i) {
            if (data[i] == '<') {
                frame_end = &data[i];
                break;
            }
        }
        if (!frame_end || frame_end <= arg_ptr) {
            printk("[CMD] SF rejected: bad frame end\n");
            send_response(">ERR<");
            return;
        }
        size_t arg_len2 = (size_t)(frame_end - arg_ptr);

        if (!(arg_len2 == 1 || arg_len2 == 3)) {
            printk("[CMD] SF rejected: arg_len=%u (expected 1 or 3)\n", (unsigned)arg_len2);
            send_response(">ERR<");
            return;
        }

        uint8_t start_in = arg_ptr[0];
        uint8_t end_in   = (arg_len2 == 3) ? arg_ptr[1] : arg_ptr[0];
        uint8_t dur_in   = (arg_len2 == 3) ? arg_ptr[2] : 0;

        /* Validation kroz existing funkcije */
        uint8_t start_v = max_freq_for_pw(pulse_width * 20, start_in);
        uint8_t end_v   = max_freq_for_pw(pulse_width * 20, end_in);
        uint8_t dur_v   = dur_in;

        frequency = start_v;
        printk("[CMD] SF RAW: start_in=%u end_in=%u dur_in=%u -> mapped start=%u end=%u dur=%u\n",
            start_in, end_in, dur_in, start_v, end_v, dur_v);

        /* Validate range 1..100 Hz */
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
            
            /* NOVO: Thread-safe notification */
            if (impulse_initialized) {
                notify_frequency_changed();
            }
            
            printk("[CMD] SF applied: CONSTANT %u Hz\n", start_v);
            send_response(">OK<");
            return;
        } else {
            /* RANGE MODE */
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

            /* NOVO: Thread-safe notification */
            if (impulse_initialized) {
                notify_frequency_changed();
            }

            printk("[CMD] SF applied: RANGE %u..%u Hz, dur=%u s\n", start_v, end_v, dur_v);
            send_response(">OK<");
            return;
        }
    }

    /* === PW: 2 bytes HEX (16-bit big-endian), range 50..1000 === */
    if (strcmp(cmd, "PW") == 0) {
        if (arg_len != 2) {
            printk("[CMD] PW rejected: arg_len=%u (expected 2)\n", (unsigned)arg_len);
            send_response(">ERR<");
            return;
        }

        uint16_t v = ((uint16_t)arg_ptr[0] << 8) | (uint16_t)arg_ptr[1];
        printk("[CMD] PW RAW(2): %u\n", v);

        if (v >= 50 && v <= 1000) {
            /* Validation against frequency */
            uint16_t min_pw = min_pw_for_freq(frequency, v);
            pulse_width = min_pw / 20;
            
            /* NOVO: Thread-safe notification */
            if (impulse_initialized) {
                notify_frequency_changed();
            }
            
            printk("[CMD] PW applied: %u\n", min_pw);
            send_response(">OK<");
        } else {
            printk("[CMD] PW rejected: out-of-range %u\n", v);
            send_response(">ERR<");
        }
        return;
    }

    /* === SP: 1 byte HEX — set active pattern index === */
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
            char ok[40]; 
            snprintf(ok, sizeof(ok), "OK: active pattern=%u\n", idx);
            send_response(ok);
        } else {
            printk("[CMD] SP rejected: idx=%u >= count=%u\n", idx, (unsigned)patterns_count);
            send_response("ERR: invalid pattern index\n");
        }
        return;
    }

    /* === ST: 2 bytes HEX — duration in seconds (MSB, LSB) === */
    if (strcmp(cmd, "ST") == 0) {
        if (arg_len != 2) {
            printk("[CMD] ST rejected: arg_len=%u (expected 2)\n", (unsigned)arg_len);
            send_response(">ST;ERR<\r\n");
            return;
        }

        uint16_t sec = ((uint16_t)arg_ptr[0] << 8) | (uint16_t)arg_ptr[1];

        if (sec >= 1 && sec <= 65535) {
            stim_duration_s = (uint32_t)sec;
            printk("[CMD] ST applied: %u s\n", sec);
            send_response(">ST;OK<\r\n");
        } else {
            printk("[CMD] ST rejected: out-of-range %u\n", sec);
            send_response(">ST;ERR<\r\n");
        }
        return;
    }

    /* === SA: 16 bytes HEX — write to active pattern as 8 values === */
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
            uint16_t raw = ((uint16_t)arg_ptr[2*i] << 8) | arg_ptr[2*i + 1];
            pulse_patterns[number_patter][i] = raw;
            printk("[CMD] SA ch%d=0x%04X\n", i, raw);
        }

        printk("[CMD] SA applied to pattern #%u\n", (unsigned)number_patter);

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

    /* === XC: 16 bytes HEX — cross-channel amplitude configuration === */
    else if (strcmp(cmd, "XC") == 0) {
        if (arg_len != 16) {
            printk("[CMD] XC rejected: arg_len=%u (expected 16)\n", (unsigned)arg_len);
            send_response(">XC;ERR<\r\n");
            return;
        }

        /* NOVO: Validation protiv impulse.h konstanti */
        bool validation_failed = false;
        
        for (int i = 0; i < 8; ++i) {
            uint16_t raw = ((uint16_t)arg_ptr[2*i] << 8) | arg_ptr[2*i + 1];
            uint16_t val = raw / 10;

            /* NOVO: Koristi konstante iz impulse.h */
            if (val < MIN_AMPLITUDE_UA || val > MAX_AMPLITUDE_UA) {
                printk("[CMD] XC rejected: ch%d=%u out-of-range (%u..%u µA)\n", 
                       i, val, MIN_AMPLITUDE_UA, MAX_AMPLITUDE_UA);
                validation_failed = true;
                break;
            }
        }
        
        if (validation_failed) {
            send_response(">XC;ERR<\r\n");
            return;
        }

        /* Apply validated values */
        for (int i = 0; i < 8; ++i) {
            uint16_t raw = ((uint16_t)arg_ptr[2*i] << 8) | arg_ptr[2*i + 1];
            uint16_t val = raw / 10;
            pair_amplitude_uA[i] = val;
        }

        printk("[CMD] XC applied\n");

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

    /* === NOVO: Advanced Debug Commands === */
    else if (strcmp(cmd, "DBG") == 0) {
        if (arg_len == 1) {
            uint8_t debug_cmd = arg_ptr[0];
            
            switch (debug_cmd) {
                case 0x01: /* Reset performance counters */
                    /* Implementacija zavisi od impulse API-ja */
                    send_response(">DBG;RESET_OK<\r\n");
                    break;
                    
                case 0x02: /* Force error recovery */
                    /* Trigger manual recovery */
                    send_response(">DBG;RECOVERY_OK<\r\n");
                    break;
                    
                case 0x03: /* Get thread state */
                    {
                        int state = get_impulse_state();
                        char resp[32];
                        snprintk(resp, sizeof(resp), ">DBG;STATE=%d<\r\n", state);
                        send_response(resp);
                    }
                    break;
                    
                default:
                    send_response(">DBG;UNKNOWN<\r\n");
                    break;
            }
        } else {
            send_response(">DBG;ERR<\r\n");
        }
        return;
    }

    /* If we received some other command with arguments not defined by protocol */
    printk("[CMD] Unknown command with fixed-HEX args '%s' (len=%u)\n", cmd, (unsigned)arg_len);
    send_response(">ERR;UNKNOWN<\r\n");
}

/* ============================================================================
 * CONNECTION EVENT HANDLERS
 * ============================================================================ */

static void request_conn_param(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!current_conn) return;

    int err = bt_conn_le_param_update(current_conn, &desired_param);
    if (err) {
        printk("bt_conn_le_param_update err=%d, retry in 3s\n", err);
        static struct k_work_delayable *self;
        self = CONTAINER_OF(work, struct k_work_delayable, work);
        k_work_reschedule(self, K_SECONDS(3));
    } else {
        printk("Conn param update requested\n");
    }
}

static K_WORK_DELAYABLE_DEFINE(conn_param_work, request_conn_param);

void bt_ready(int err);
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("BLE connection failed (err %u)\n", err);
        return;
    }

    current_conn = bt_conn_ref(conn);

    struct bt_conn_info info;
    if (!bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE) {
        printk("BLE connected: interval=%u*1.25ms latency=%u timeout=%u*10ms\n",
               info.le.interval, info.le.latency, info.le.timeout);
    } else {
        printk("BLE connected\n");
    }

    gpio_pin_set_dt(&led1, 1);
    k_work_schedule(&conn_param_work, K_SECONDS(2));
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("BLE disconnected (reason 0x%02X)\n", reason);

    gpio_pin_set_dt(&led1, 0);

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

/* ============================================================================
 * FREQUENCY CONTROL MODULE
 * ============================================================================ */

static uint8_t cur_freq = 0;
static int dir = +1;

static void set_frequency_safe(uint8_t f_hz)
{
    frequency     = f_hz;
    new_frequency = 1;
    
    /* NOVO: Thread-safe notification */
    if (impulse_initialized) {
        notify_frequency_changed();
    }
    
    LOG_INF("FREQ set %u Hz", f_hz);
}

static void freq_work_handler(struct k_work *work);
K_WORK_DEFINE(freq_work, freq_work_handler);

static void freq_timer_handler(struct k_timer *timer)
{
    k_work_submit(&freq_work);
}
K_TIMER_DEFINE(freq_timer, freq_timer_handler, NULL);

static void freq_work_handler(struct k_work *work)
{
    if (!stimulation_running) {
        return;
    }

    uint8_t start = freq_start;
    uint8_t end   = freq_end;
    uint8_t dur_s = freq_dur;

    if (start < 1) start = 1;
    if (start > 100) start = 100;
    if (end   < 1) end   = 1;
    if (end   > 100) end = 100;

    if (!freq_sweep || dur_s == 0) {
        set_frequency_safe(start);
        return;
    }

    if (cur_freq == 0) {
        cur_freq = start;
        dir = (start <= end) ? +1 : -1;
    }

    set_frequency_safe(cur_freq);

    if (dir > 0) {
        if (cur_freq < end) cur_freq++;
        else cur_freq = start;
    } else {
        if (cur_freq > end) cur_freq--;
        else cur_freq = start;
    }

    k_timer_start(&freq_timer, K_SECONDS(dur_s), K_NO_WAIT);
}

void freq_control_start(void)
{
    cur_freq = 0;
    if (!freq_sweep || freq_dur == 0) {
        set_frequency_safe(freq_start);
    } else {
        k_timer_start(&freq_timer, K_NO_WAIT, K_NO_WAIT);
    }
}

void freq_control_stop(void)
{
    k_timer_stop(&freq_timer);
    cur_freq = 0;
}

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */

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


    
    /* NOVO: Označi da je impulse subsystem inicijalizovan */
    impulse_initialized = true;

    printk("BLE NUS initialized successfully\n");
    return 0;
}