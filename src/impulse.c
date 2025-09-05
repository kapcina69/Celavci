// impulse.c — stabilni pulsevi u posebnoj niti (irq_lock + k_busy_wait), bez k_work

#include "impulse.h"
#include "ble_nus.h"
#include "mux.h"
#include "dac.h"
#include "rsens.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <string.h>

/* === Devicetree aliasi === */
#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

#if !DT_NODE_HAS_PROP(PULSE_CATHODE_NODE, gpios) || !DT_NODE_HAS_PROP(PULSE_ANODE_NODE, gpios)
#error "Define pulse_cathode and pulse_anode aliases with a 'gpios' property in DTS"
#endif

/* === GPIO iz DT === */
const struct gpio_dt_spec pulse_cathode = GPIO_DT_SPEC_GET(PULSE_CATHODE_NODE, gpios);
const struct gpio_dt_spec pulse_anode   = GPIO_DT_SPEC_GET(PULSE_ANODE_NODE, gpios);
const struct gpio_dt_spec dc_dc_en      = GPIO_DT_SPEC_GET(DC_DC_EN_NODE, gpios);

/* === Globalno stanje (preuzeto iz tvog koda) === */
uint8_t  number_of_pulses      = 0;
uint32_t frequency_of_impulses = 4000;   // Hz
volatile uint32_t stim_duration_s = 1800;  // 30 min
static uint8_t tx_buffer_1[2];

/* Default 100 µA za svih 8 parova */
volatile uint16_t pair_amplitude_uA[8] = { 100,100,100,100,100,100,100,100 };

/* Pattern-i */
#define PATTERN_LEN   8
#define MAX_PATTERNS  16
uint16_t pulse_patterns[MAX_PATTERNS][PATTERN_LEN];
volatile size_t patterns_count = 0;
volatile size_t number_patter  = 0;

/* === Periodična TIHA RCE sonda (svakih 5 s) === */
static uint32_t next_silent_probe_ms = 0;
static bool     silent_probe_active  = false;
static uint8_t  silent_count         = 0;
static uint8_t  silent_mask          = 0;


/* Širina jedinice u tvom izrazu: STIMULATION_PULSE_WIDTH_US * pulse_width */
#ifndef STIMULATION_PULSE_WIDTH_US
#define STIMULATION_PULSE_WIDTH_US 1
#endif

static uint8_t rce_count = 0;       // koliko impulsa je očitano (0..8)
static uint8_t rce_mask  = 0;       // 8-bit maska rezultata
#define RCE_THRESHOLD_MV 100        // prag 100 mV

/* Kontrola rada */
static volatile bool s_impulse_inited  = false;
static volatile bool s_impulse_running = false;

/* === Timer za auto-OFF === */
static void stim_timer_handler(struct k_timer *t)
{
    s_impulse_running = false;
    stop_pulse_sequence();
    stimulation_running = false;
    send_response("OFFOK\r\n");
}
K_TIMER_DEFINE(stim_timer, stim_timer_handler, NULL);

/* === Helperi === */
static inline void set_anode(int level)   { gpio_pin_set_dt(&pulse_anode,   level); }
static inline void set_cathode(int level) { gpio_pin_set_dt(&pulse_cathode, level); }

uint32_t hz_to_us(uint32_t hz) { return hz ? (1000000UL / hz) : 0; }

static void ensure_default_pattern(void)
{
    if (patterns_count == 0) {
        static const uint16_t def[PATTERN_LEN] = {
            0x0101, 0x0202, 0x0404, 0x0808,
            0x1010, 0x2020, 0x4040, 0x8080
        };
        memcpy(pulse_patterns[0], def, sizeof(def));
        patterns_count = 1;
        number_patter  = 0;
    }
}

static int pulse_pins_init(void)
{
    if (!device_is_ready(pulse_anode.port) || !device_is_ready(pulse_cathode.port)) {
        send_response("IMP: GPIO device not ready\r\n");
        return -ENODEV;
    }
    int err = 0;
    err |= gpio_pin_configure_dt(&pulse_anode,   GPIO_OUTPUT_INACTIVE);
    err |= gpio_pin_configure_dt(&pulse_cathode, GPIO_OUTPUT_INACTIVE);
    if (err) {
        send_response("IMP: GPIO cfg error\r\n");
        return err;
    }
    set_anode(0);
    set_cathode(0);
    return 0;
}

/* === Namera: jedan "trokorak" pulsa: ANODE_ON -> CATHODE_ON -> PAUSE ===
 * Svaka faza traje width_us; precizno u µs, bez jitter-a (irq_lock + k_busy_wait).
 */

static inline void do_one_pulse_us(uint32_t width_us, uint8_t pair_idx)
{
    /* (A) Zakazivanje 5 s "tihe" sonde */
    uint32_t now_ms = k_uptime_get_32();
    if (next_silent_probe_ms == 0) {
        next_silent_probe_ms = now_ms + 5000;           // prvi put nakon 5 s
    } else if (!silent_probe_active && (int32_t)(now_ms - next_silent_probe_ms) >= 0) {
        silent_probe_active = true;                      // start nove tihe sonde (8 impulsa)
        silent_count        = 0;
        silent_mask         = 0;
        next_silent_probe_ms = now_ms + 5000;           // sledeća sonda za 5 s
    }

    /* DAC po paru (tvoj postojeći kod) */
    if (pair_idx < PATTERN_LEN) {
        uint16_t uA = pair_amplitude_uA[pair_idx];
        dac_set_value((int)(uA * 0.85));
    }

    /* 1) ANODE_ON */
    unsigned int key = irq_lock();
    set_cathode(0);
    set_anode(1);
    irq_unlock(key);
    k_busy_wait(width_us);

    /* 2) CATHODE_ON (merenje se radi JEDINO dok je katoda=1) */
    key = irq_lock();
    set_anode(0);
    set_cathode(1);
    irq_unlock(key);

    /* Da li treba da merimo u ovom impulsu? */
    bool do_meas_rce    = (RCE == 1) && (rce_count   < 8);
    bool do_meas_silent =  silent_probe_active && (silent_count < 8);

    if (do_meas_rce || do_meas_silent) {
        read_rsens();                                        // blokirajuće, ali kratko
        int32_t mv = rsens_get_last_measurement();

        uint8_t bit = (1u << (pair_idx & 7));
        if (mv > RCE_THRESHOLD_MV) {
            if (do_meas_rce)    { rce_mask    |= bit; }
            if (do_meas_silent) { silent_mask |= bit; }
        }

        if (do_meas_rce) {
            rce_count++;
            if (rce_count == 8) {
                char msg[24];
                snprintf(msg, sizeof(msg), "RCE;0x%02X\r\n", rce_mask);
                send_response(msg);

                /* Ako želiš da RCE ostane isključivo pod BLE kontrolom,
                   OBRIŠI narednu liniju; u tom slučaju dodaj BLE komandu za reset. */
                RCE = 0;  /* <- ukloni ako ne želiš auto-clear */

                rce_count = 0;
                rce_mask  = 0;
            }
        }

        if (do_meas_silent) {
            silent_count++;
            if (silent_count == 8) {
                /* TIHA sonda završena: ne šaljemo ništa i ne diramo RCE */
                silent_probe_active = false;
                silent_count = 0;
                silent_mask  = 0;
            }
        }
        if(width_us < 500){
            k_busy_wait(1);
        }else{
            k_busy_wait(width_us-400); // minimum 500us čitanje
        }
    }else{
        k_busy_wait(width_us);   /* trajanje katodne faze */
    }
    /* 3) PAUSE */
    key = irq_lock();
    set_cathode(0);
    set_anode(0);
    irq_unlock(key);
    k_busy_wait(width_us);
}


/* === Thread koji pravi pulseve stabilno === */
#define IMP_STACK  2048
#define IMP_PRIO   6          /* veći prioritet (manji broj) od RSENS niti */
K_THREAD_STACK_DEFINE(imp_stack, IMP_STACK);
static struct k_thread imp_thread_data;

static void impulse_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    if (pulse_pins_init() != 0) {
        return;
    }

    ensure_default_pattern();

    /* Pattern selekcija */
    size_t sel = number_patter >= patterns_count ? 0 : number_patter;
    const uint16_t *cur = pulse_patterns[sel];

    /* Početni MUX obrazac (prvi par) */
    uint16_t first = cur[0];
    tx_buffer_1[0] = (uint8_t)(first >> 8);
    tx_buffer_1[1] = (uint8_t)(first & 0xFF);
    (void)mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));

    number_of_pulses = 0;

    /* Rasporedi auto-OFF */
    k_timer_start(&stim_timer, K_SECONDS(stim_duration_s), K_NO_WAIT);

    send_response("IMP: ready\r\n");

             uint32_t T_intra_us          = hz_to_us(frequency_of_impulses);                              // period između pulseva
                               // period između grupa (8)


   while (s_impulse_running) {
    const uint32_t phase_us      = (uint32_t)STIMULATION_PULSE_WIDTH_US * (uint32_t)pulse_width; // 1 faza
    const uint32_t one_pulse_us  = 3U * phase_us;                                                // ANODE + CATHODE + PAUSE
    uint32_t T_block_us          = hz_to_us(frequency);  
    /* pattern & prvi MUX za grupu */
    size_t sel = number_patter >= patterns_count ? 0 : number_patter;
    const uint16_t *cur = pulse_patterns[sel];

    /* sabira koliko je trajalo u okviru ove grupe */
    uint64_t group_elapsed_us = 0;

    for (uint8_t i = 0; i < PATTERN_LEN && s_impulse_running; i++) {
        /* MUX za tekući par */
        uint16_t pat = cur[i];
        tx_buffer_1[0] = (uint8_t)(pat >> 8);
        tx_buffer_1[1] = (uint8_t)(pat & 0xFF);
        (void)mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));

        /* Jedan puls (3 faze, precizno u µs, bez IRQ-a) */
        do_one_pulse_us(phase_us, i);
        number_of_pulses++;

        group_elapsed_us += one_pulse_us;

        /* Odmor između pojedinačnih pulseva (osim posle 8-og) */
        if (i < (PATTERN_LEN - 1)) {
            uint32_t rest_us = (T_intra_us > one_pulse_us) ? (T_intra_us - one_pulse_us) : 0;
            group_elapsed_us += rest_us;

            if (rest_us >= 2000) {
                k_msleep(rest_us / 1000);
                uint32_t rem = rest_us % 1000;
                if (rem) k_busy_wait(rem);
            } else {
                k_busy_wait(rest_us);
            }
        }
    }

    /* Odmor između grupa (do perioda 'frequency') */
    uint32_t inter_rest_us = (T_block_us > group_elapsed_us) ? (uint32_t)(T_block_us - group_elapsed_us) : 0;
    if (inter_rest_us >= 2000) {
        k_msleep(inter_rest_us / 1000);
        uint32_t rem = inter_rest_us % 1000;
        if (rem) k_busy_wait(rem);
    } else {
        k_busy_wait(inter_rest_us);
    }

    /* spremi se za sledeću grupu; brojanje unutar grupe resetuješ po želji */
    number_of_pulses = 0;
}


    /* Fail-safe: pinovi u 0 */
    set_anode(0);
    set_cathode(0);
}

/* === START/STOP API (idempotentni) === */
void start_pulse_sequence(void)
{
    if (s_impulse_running) return;

    s_impulse_inited  = true;
    s_impulse_running = true;

    /* Ako imaš DC-DC enable pin, uključi ga pre starta (po potrebi) */
    if (device_is_ready(dc_dc_en.port)) {
        gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&dc_dc_en, 1);
    }

    /* Pokreni nit */
    k_thread_create(&imp_thread_data, imp_stack, IMP_STACK,
                    impulse_thread, NULL, NULL, NULL,
                    IMP_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&imp_thread_data, "impulse");
}

void stop_pulse_sequence(void)
{
    if (!s_impulse_inited) {
        send_response("IMPULSE NOT INITED\r\n");
        return;
    }
    s_impulse_running = false;
    k_timer_stop(&stim_timer);


}