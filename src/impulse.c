#include "impulse.h"
#include "ble_nus.h"
#include "mux.h"
#include "dac.h"
#include "nrfx_adc.h"
#include "pwm.h"

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

uint8_t  number_of_pulses      = 0;
uint32_t frequency_of_impulses = 5000;   // Hz
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

#ifndef STIMULATION_PULSE_WIDTH_US
#define STIMULATION_PULSE_WIDTH_US 1
#endif

static uint8_t rce_count = 0;       // koliko impulsa je očitano (0..8)
static uint8_t rce_mask  = 0;       // 8-bit maska rezultata
#define RCE_THRESHOLD_MV 100        // prag 100 mV

/* Korektivna vrednost*/
#define FREQ_CORRECT 7000

static uint8_t RCE_FIRST = 0;  

/* Kontrola rada */
static volatile bool s_impulse_inited  = false;
static volatile bool s_impulse_running = false;

/* === Forward deklaracije === */
static void stop_pulse_sequence_internal(bool from_timer);

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

static void my_adc_cb(int16_t s)
{
    // LOG_INF("One-shot SAADC sample = %d", s);
}

static int pulse_pins_init(void)
{
    saadc_ppi_oneshot_init();
    // saadc_set_callback(my_adc_cb);

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

#include <stdint.h>

#define BURST_LEN         8
uint8_t MEASURE_EVERY_N=3;  // meri svaku treću povorku (1,4,7,...)

// 1..8: tekući impuls u povorci
static uint8_t  pulse_in_burst = 1;
// 1..8: koji impuls merimo u "mernoj" povorci
static uint8_t  measure_idx    = 1;
// 1,2,3,... redni broj povorke od starta
static uint32_t burst_seq      = 1;

static inline bool is_measure_burst(void)
{
    // merni su 1., 4., 7., ... tj. (burst_seq-1) % 3 == 0
    return ((burst_seq - 1u) % MEASURE_EVERY_N) == 0u;
}

static inline void burst_advance(void)
{
    if (++pulse_in_burst > BURST_LEN) {
        // završena povorka
        pulse_in_burst = 1;

        // ako je OVA povorka bila merni ciklus, onda pređi na sledeći impuls za merenje
        if (((burst_seq - 1u) % MEASURE_EVERY_N) == 0u) {
            if (++measure_idx > BURST_LEN) {
                measure_idx = 1;
            }
        }
        // pređi na sledeću povorku
        burst_seq++;
    }
}

static inline void reset_burst_schedule(void)
{
    pulse_in_burst = 1;
    measure_idx    = 1;
    burst_seq      = 1;
}

void report_last_burst(void)
{
    uint8_t mask;
    if (saadc_get_last_burst(&mask)) {
        char send_buff[8];
        send_buff[0] = '>';
        send_buff[1] = 'R';
        send_buff[2] = 'C';
        send_buff[3] = 'E';
        send_buff[4] = ';';
        send_buff[5] = mask;   // direktno ubaci masku kao 1 bajt
        send_buff[6] = '<';
        send_buff[7] = '\0';

        bt_nus_send(NULL, send_buff, 8);
    } else {
        bt_nus_send(NULL, ">RCE;NA<", sizeof(">RCE;NA<"));
    }
}


/* === Jedan puls (3 faze) === */
static inline void do_one_pulse_us(uint32_t width_us, uint8_t pair_idx)
{
    /* DAC za taj par (po tvom kodu 0.85 skalar) */
    if (pair_idx < PATTERN_LEN) {
        uint16_t uA = pair_amplitude_uA[pair_idx];
        dac_set_value((int)(uA * 0.85f));
    }

    if (RCE == 1) {
        report_last_burst();
        RCE = 0;
    }
    if (new_frequency) {
        new_frequency = 0;
        reset_burst_schedule();
    }

    /* === 1) ANODE_ON (kritična sekcija) === */
    unsigned int key = irq_lock();
    set_cathode(0);
    set_anode(1);
    irq_unlock(key);

    k_busy_wait(width_us);  /* anoda */

    /* === 2) CATHODE_ON + (uslovno) merenje tokom katode === */
    key = irq_lock();
    set_anode(0);
    set_cathode(1);
    irq_unlock(key);

    if (frequency == 1 || frequency == 2 || frequency == 3 || frequency == 4) {
        saadc_trigger_once_ppi();
    } else if (is_measure_burst() && (pulse_in_burst == measure_idx)) {
        saadc_trigger_once_ppi();  // bez kašnjenja (PPI/DPPI)
    }

    k_busy_wait(width_us);  /* katoda */

    /* === 3) PAUSE (oba 0) === */
    key = irq_lock();
    set_cathode(0);
    set_anode(0);
    irq_unlock(key);

    // k_busy_wait(width_us);  /* pauza */

    /* pomeri brojače */
    burst_advance();
}

/* === Work koji bezbedno gasi stimulaciju (poziva se iz timer callback-a) === */
static void stop_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    stop_pulse_sequence_internal(true);  // from_timer = true
}
K_WORK_DEFINE(stim_stop_work, stop_work_handler);

/* === Timer handler: samo pokreće work koji gasi stimulaciju === */
static void stim_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    /* Zastavi glavnu petlju i prepusti gašenje work-u (bezbedno okruženje) */
    s_impulse_running = false;
    k_work_submit(&stim_stop_work);
}

/* One-shot tajmer: traje tačno stim_duration_s i onda gasi stimulaciju */
K_TIMER_DEFINE(stim_timer, stim_timer_handler, NULL);

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
    uint32_t T_intra_us          = hz_to_us(frequency_of_impulses);                              // period između pulseva

    send_response("IMP: ready\r\n");

    while (s_impulse_running) {
        const uint32_t phase_us      = (uint32_t)STIMULATION_PULSE_WIDTH_US * (uint32_t)pulse_width; // 1 faza
        const uint32_t one_pulse_us  = 2U * phase_us;                                                // ANODE + CATHODE + PAUSE
        uint32_t T_block_us          = hz_to_us(frequency);                                          // period između grupa (8)

        /* pattern & prvi MUX za grupu */
        size_t sel2 = number_patter >= patterns_count ? 0 : number_patter;
        const uint16_t *cur2 = pulse_patterns[sel2];

        /* sabira koliko je trajalo u okviru ove grupe */
        uint64_t group_elapsed_us = 0;

        for (uint8_t i = 0; i < PATTERN_LEN && s_impulse_running; i++) {
            /* MUX za tekući par */
            uint16_t pat = cur2[i];
            tx_buffer_1[0] = (uint8_t)(pat >> 8);
            tx_buffer_1[1] = (uint8_t)(pat & 0xFF);
            (void)mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));

            /* Jedan puls (3 faze, precizno u µs, bez IRQ-a) */
            do_one_pulse_us(phase_us, i);
            number_of_pulses++;

            group_elapsed_us += one_pulse_us;

            // /* Odmor između pojedinačnih pulseva (osim posle 8-og) */
            // if (i < (PATTERN_LEN - 1)) {
            //     uint32_t rest_us = 2; 
            //     group_elapsed_us += rest_us;

            //     // k_usleep(5);
            // }
        }

        
        k_usleep(T_block_us-group_elapsed_us-FREQ_CORRECT);

        /* spremi se za sledeću grupu; brojanje unutar grupe resetuješ po želji */
        number_of_pulses = 0;
    }

    /* Fail-safe: pinovi u 0 */
    set_anode(0);
    set_cathode(0);
}

/* === START/STOP === */
static void stop_pulse_sequence_internal(bool from_timer)
{
    /* Zaustavi glavnu petlju */
    s_impulse_running = false;

    /* Ugasi tajmer (ako nije već istekao) */
    k_timer_stop(&stim_timer);

    /* Spusti pinove i isključi DC/DC */
    set_anode(0);
    set_cathode(0);

    if (device_is_ready(dc_dc_en.port)) {
        gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&dc_dc_en, 0);
    }
    stimulation_running = false;

    if (from_timer) {
        send_response("OFFOK\r\n");
    } else {
        send_response("STOPPED\r\n");
    }
}

void start_pulse_sequence(void)
{
    if (s_impulse_running) {
        return;
    }

    s_impulse_inited  = true;
    s_impulse_running = true;

    /* Uključi DC-DC pre starta (po potrebi) */
    if (device_is_ready(dc_dc_en.port)) {
        gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&dc_dc_en, 1);
    }

    /* Start one-shot timera koji trajanje sesije vezuje za stim_duration_s */
    k_timer_start(&stim_timer, K_SECONDS(stim_duration_s), K_NO_WAIT);

    /* Pokreni nit sa impulsima */
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
    stop_pulse_sequence_internal(false);
    pwm_ch1_stop();

}

