// impulse.c
#include "impulse.h"
#include "ble_nus.h"
#include "mux.h"
#include "dac.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>   // ARRAY_SIZE
#include <string.h>

/* === Devicetree aliasi === */
#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

// === Globalne promenljive === */
uint8_t  number_of_pulses         = 0;
uint32_t frequency_of_impulses    = 4000;   // Hz
int32_t voltage;

static uint8_t tx_buffer_1[2];

volatile uint32_t stim_duration_s = 1800;  // default 30 min


// Default 100 µA za svih 8 parova
volatile uint16_t pair_amplitude_uA[8] = {
    10, 10, 10, 10, 10, 10, 10, 10
};


/* === Globalni storage za pattern-e === */
#define PATTERN_LEN   8
#define MAX_PATTERNS  16

/* Kolekcija pattern-a: pulse_patterns[i] je niz od 8 parova [anoda(hi) | katoda(lo)] */
uint16_t pulse_patterns[MAX_PATTERNS][PATTERN_LEN];
volatile size_t patterns_count = 0;

/* Bira koji pattern iz kolekcije se koristi pri generisanju */
volatile size_t number_patter = 0;

/* === GPIO definicije iz DT === */
const struct gpio_dt_spec pulse_cathode = GPIO_DT_SPEC_GET(PULSE_CATHODE_NODE, gpios);
const struct gpio_dt_spec pulse_anode   = GPIO_DT_SPEC_GET(PULSE_ANODE_NODE, gpios);
const struct gpio_dt_spec dc_dc_en      = GPIO_DT_SPEC_GET(DC_DC_EN_NODE, gpios);

/* === Pomoćne funkcije === */
uint32_t hz_to_us(uint32_t frequency_hz)
{
    return (frequency_hz == 0) ? 0 : (1000000UL / frequency_hz);
}

/* === Stanje mašine === */
enum pulse_state_e {
    PULSE_IDLE = 0,
    PULSE_ANODE_ON,
    PULSE_CATHODE_ON,
    PULSE_PAUSE
};

static volatile enum pulse_state_e pulse_state = PULSE_IDLE;

static bool s_impulse_inited = false;
static bool s_impulse_running = false;

/* === Work koji izvršava state machine === */
static void pulse_work_fn(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(pulse_work, pulse_work_fn);



static void stim_timer_handler(struct k_timer *t)
{
    s_impulse_running = false;
    stop_pulse_sequence();
    stimulation_running = false;
    send_response("OFFOK\r\n");  // automatski OFF kada istekne
}
K_TIMER_DEFINE(stim_timer, stim_timer_handler, NULL);


/* === Interno: obezbedi bar jedan default pattern ako kolekcija još nema ni jedan === */
static void ensure_default_pattern(void)
{
    if (patterns_count == 0) {
        static const uint16_t def[PATTERN_LEN] = {
            0x0101, 0x0202, 0x0404, 0x0808,
            0x1010, 0x2020, 0x4040, 0x8080
        };
        memcpy(pulse_patterns[0], def, sizeof(def));
        patterns_count = 1;
        number_patter = 0;
    }
}

/* === State machine u work kontekstu  === */
static void pulse_work_fn(struct k_work *w)
{
    ARG_UNUSED(w);

    /* Uveri se da imamo bar jedan pattern */
    ensure_default_pattern();

    /* Zaštita indeksa */
    size_t sel = number_patter;
    if (sel >= patterns_count) {
        sel = 0;
    }
    const uint16_t *cur = pulse_patterns[sel];

    switch (pulse_state) {
    case PULSE_ANODE_ON:
        dac_set_value(pair_amplitude_uA[number_of_pulses] * 0.85); 
        gpio_pin_set_dt(&pulse_cathode, 0);
        gpio_pin_set_dt(&pulse_anode,   1);
        pulse_state = PULSE_CATHODE_ON;
        k_work_reschedule(&pulse_work, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width));
        break;

    case PULSE_CATHODE_ON:
        gpio_pin_set_dt(&pulse_anode,   0);
        gpio_pin_set_dt(&pulse_cathode, 1);
        pulse_state = PULSE_PAUSE;
        k_work_reschedule(&pulse_work, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width));
        break;

    case PULSE_PAUSE:
        // int err=read_ntcs_voltage_and_report();
        gpio_pin_set_dt(&pulse_anode,   0);
        gpio_pin_set_dt(&pulse_cathode, 0);

        if (number_of_pulses < PATTERN_LEN) {
            /* Uzmi 16-bitni par: [anoda (MSB)] [katoda (LSB)] */
            uint16_t pattern = cur[number_of_pulses];
            tx_buffer_1[0] = (uint8_t)(pattern >> 8);     // anoda
            tx_buffer_1[1] = (uint8_t)(pattern & 0xFF);   // katoda
            (void)mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));
        }

        number_of_pulses++;

        if (!s_impulse_running) {
            /* Ako je stop zatražen dok je work u toku – idi u idle i ne re-schedule-uj */
            pulse_state = PULSE_IDLE;
            return;
            
        }

        if (number_of_pulses >= PATTERN_LEN) {
            number_of_pulses = 0;
            pulse_state = PULSE_IDLE;
            /* Ostavio sam tvoju formulu za raspored posle grupe */
            k_work_reschedule(&pulse_work, K_USEC(hz_to_us(frequency)-16*STIMULATION_PULSE_WIDTH_US*pulse_width-7*hz_to_us(frequency_of_impulses)-4000));
        } else {
            pulse_state = PULSE_ANODE_ON;
            k_work_reschedule(&pulse_work, K_USEC(hz_to_us(frequency_of_impulses)));
        }
        break;

    case PULSE_IDLE:
    default:
        gpio_pin_set_dt(&pulse_anode,   0);
        gpio_pin_set_dt(&pulse_cathode, 0);
        if (!s_impulse_running) {
            /* stopirano – ne nastavljamo ciklus */
            return;
        }
        pulse_state = PULSE_ANODE_ON;
        k_work_reschedule(&pulse_work, K_USEC(hz_to_us(frequency_of_impulses)));
        break;
    }
}

/* === START: idempotentno; ne zakazuje dupli work ciklus === */
void start_pulse_sequence(void)
{
    

    if (s_impulse_running) {
        /* već radi */
        return;
    }

    ensure_default_pattern();

    /* početni MUX obrazac — prvi element aktuelnog pattern-a */
    size_t sel = number_patter >= patterns_count ? 0 : number_patter;
    uint16_t first = pulse_patterns[sel][0];
    tx_buffer_1[0] = (uint8_t)(first >> 8);
    tx_buffer_1[1] = (uint8_t)(first & 0xFF);
    (void)mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));

    number_of_pulses = 0;
    s_impulse_running = true;
    s_impulse_inited = true;
    pulse_state = PULSE_ANODE_ON;
    k_timer_start(&stim_timer, K_SECONDS(stim_duration_s), K_NO_WAIT);


    k_work_reschedule(&pulse_work, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width));
}

/* === STOP: idempotentno; gasi pinove i ne ostavlja re-schedule u letu === */
void stop_pulse_sequence(void)
{
    if (!s_impulse_inited) {
        send_response("IMPULSE NOT INITED\r\n");
        return; /* ništa da se gasi */
    }

    s_impulse_running = false;

    /* Otkaži zakazani rad; OK je pozvati i ako nije zakazan */
    (void)k_work_cancel_delayable(&pulse_work);

    /* Obezbedi “fail-safe” stanje pinova */
    gpio_pin_set_dt(&pulse_anode,   0);
    gpio_pin_set_dt(&pulse_cathode, 0);

    pulse_state = PULSE_IDLE;
}
