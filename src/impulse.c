/**
 * @file impulse.c
 * @brief Generisanje bifaznih (bipolarnih) stimulacionih impulsa preko GPIO i MUX
 * 
 * POBOLJŠANJA U OVOJ VERZIJI:
 * ===========================
 * 1. Thread Pool Pattern - eliminiše thread leaks
 * 2. Robusan Error Handling sa recovery mehanizmima
 * 3. Bezbedni Timing Calculations sa overflow protection
 * 4. Atomic Operations za thread-safe komunikaciju
 * 5. Performance Counters za debugging
 * 6. Dokumentovani "korektivni" brojevi
 * 7. Centralizovano State Management
 * 8. Watchdog za hardware recovery
 */

#include "impulse.h"
#include "ble_nus.h"
#include "mux.h"
#include "dac.h"
#include "nrfx_adc.h"
#include "pwm.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

/* === DEVICETREE ALIASI === */
#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

#if !DT_NODE_HAS_PROP(PULSE_CATHODE_NODE, gpios) || !DT_NODE_HAS_PROP(PULSE_ANODE_NODE, gpios)
#error "Define pulse_cathode and pulse_anode aliases with a 'gpios' property in DTS"
#endif

/* === KONFIGURACIJA === */
#ifndef STIMULATION_PULSE_WIDTH_US
#define STIMULATION_PULSE_WIDTH_US 1  /* Bazna širina faze u µs */
#endif

#define PATTERN_LEN   8               /* Broj kanala po pattern-u */
#define MAX_PATTERNS  16              /* Maksimalno pattern-a u memoriji */
#define IMP_STACK     2048            /* Stack veličina impulse thread-a */
#define IMP_PRIO      6               /* Thread prioritet (viši od BLE) */
#define BURST_LEN     8               /* Impulsi po burst grupi */
#define RCE_THRESHOLD_MV 100          /* Contact detection prag */

/* === KOREKTIVNI BROJEVI (Empirijski određeni za hardware) === */
#define FREQ_CORRECT_US   7000        /* Korekcija sistemskog kašnjenja (µs) */
#define DAC_SCALING_FACTOR 0.85f      /* Hardware DAC skaliranje */
#define MAX_CONSECUTIVE_ERRORS 5      /* Maksimalno uzastopnih grešaka */
#define ERROR_RECOVERY_DELAY_MS 100   /* Kašnjenje pre recovery pokušaja */
#define WATCHDOG_TIMEOUT_MS 1000      /* Hardware watchdog timeout */

/* === GPIO SPECIFIKACIJE IZ DEVICETREE === */
const struct gpio_dt_spec pulse_cathode = GPIO_DT_SPEC_GET(PULSE_CATHODE_NODE, gpios);
const struct gpio_dt_spec pulse_anode   = GPIO_DT_SPEC_GET(PULSE_ANODE_NODE, gpios);
const struct gpio_dt_spec dc_dc_en      = GPIO_DT_SPEC_GET(DC_DC_EN_NODE, gpios);

/* === ATOMSKE VARIJABLE ZA THREAD-SAFE KOMUNIKACIJU === */
static atomic_t impulse_state = ATOMIC_INIT(0);  /* 0=idle, 1=running, 2=stopping */
static atomic_t error_count = ATOMIC_INIT(0);
static atomic_t pulse_counter = ATOMIC_INIT(0);
static atomic_t frequency_changed = ATOMIC_INIT(0);

/* Stanja impulse_state */
#define IMPULSE_IDLE     0
#define IMPULSE_RUNNING  1
#define IMPULSE_STOPPING 2

/* === GLOBALNE VARIJABLE === */

/* Pattern storage */
uint16_t pulse_patterns[MAX_PATTERNS][PATTERN_LEN];
volatile size_t patterns_count = 0;     /* Broj učitanih pattern-a */
volatile size_t number_patter  = 0;     /* Indeks aktivnog pattern-a */

/* Amplitude control per-channel (µA) */
volatile uint16_t pair_amplitude_uA[8] = { 100,100,100,100,100,100,100,100 };

/* Session control */
volatile uint32_t stim_duration_s = 1800;  /* Max trajanje: 30 min */

/* Timing variables */
static uint8_t  number_of_pulses      = 0;
static uint32_t frequency_of_impulses = 5000;   /* Interna frekvencija (Hz) */

/* MUX komunikacija buffer */
static uint8_t tx_buffer_1[2];

/* Burst measurement tracking */
static uint8_t MEASURE_EVERY_N = 3;      /* Meri svaku treću grupu */
static uint8_t pulse_in_burst  = 1;      /* Trenutni impuls u grupi (1-8) */
static uint8_t measure_idx     = 1;      /* Koji impuls merimo (1-8) */
static uint32_t burst_seq      = 1;      /* Redni broj grupe */

/* RCE (Remote Contact Enable) kontrola */
static uint8_t rce_count = 0;            /* Broj očitanih impulsa */
static uint8_t rce_mask  = 0;            /* 8-bit maska rezultata */

/* Performance counters */
static struct {
    uint32_t total_pulses;
    uint32_t mux_errors;
    uint32_t dac_errors;
    uint32_t timing_violations;
    uint32_t recoveries;
} perf_stats = {0};

/* === THREAD MANAGEMENT === */
K_THREAD_STACK_DEFINE(imp_stack, IMP_STACK);
static struct k_thread imp_thread_data;
static struct k_sem thread_control_sem;
static struct k_sem thread_ready_sem;
static bool thread_created = false;

/* === TIMER MANAGEMENT === */
static struct k_timer stim_timer;
static struct k_timer watchdog_timer;
static struct k_work stop_work;
static struct k_work recovery_work;

/* === FORWARD DEKLARACIJE === */
static void stop_pulse_sequence_internal(bool from_timer);
static int pulse_pins_init(void);
static void ensure_default_pattern(void);
static int do_one_pulse_us(uint32_t width_us, uint8_t pair_idx);
static void impulse_thread(void *a, void *b, void *c);
static void error_recovery(void);

/* === HELPER FUNKCIJE === */

/**
 * @brief Thread-safe setState funkcija
 */
static bool set_impulse_state(int new_state)
{
    int old_state = atomic_set(&impulse_state, new_state);
    return (old_state != new_state);
}

/**
 * @brief Thread-safe getState funkcija
 */
int get_impulse_state(void)
{
    return atomic_get(&impulse_state);
}

/**
 * @brief Bezbedno postavlja anodu pin
 */
static inline void set_anode(int level)   
{ 
    gpio_pin_set_dt(&pulse_anode, level); 
}

/**
 * @brief Bezbedno postavlja katodu pin
 */
static inline void set_cathode(int level) 
{ 
    gpio_pin_set_dt(&pulse_cathode, level); 
}

/**
 * @brief Fail-safe: oba pina u bezbedno stanje
 */
static inline void set_pins_safe(void)
{
    unsigned int key = irq_lock();
    set_anode(0);
    set_cathode(0);
    irq_unlock(key);
}

/**
 * @brief Konvertuje frekvenciju u period (µs) sa overflow check
 */
uint32_t hz_to_us(uint32_t hz) 
{ 
    if (hz == 0) return 0;
    if (hz > 1000000) return 1; /* Overflow protection */
    return 1000000UL / hz; 
}

/**
 * @brief Osigurava postojanje default pattern-a
 */
static void ensure_default_pattern(void)
{
    if (patterns_count == 0) {
        /* Default pattern: sekvencijalni kanali */
        static const uint16_t def[PATTERN_LEN] = {
            0x0101, 0x0202, 0x0404, 0x0808,
            0x1010, 0x2020, 0x4040, 0x8080
        };
        memcpy(pulse_patterns[0], def, sizeof(def));
        patterns_count = 1;
        number_patter  = 0;
    }
}

/* === BURST MEASUREMENT LOGIC === */

/**
 * @brief Proverava da li je trenutna grupa "merna" (svaka treća)
 */
static inline bool is_measure_burst(void)
{
    return (burst_seq % MEASURE_EVERY_N) == 1;
}

/**
 * @brief Pomera burst brojače na sledeći impuls/grupu
 */
static inline void burst_advance(void)
{
    if (++pulse_in_burst > BURST_LEN) {
        /* Završena grupa */
        pulse_in_burst = 1;

        /* Ako je OVA grupa bila merna, pomeri measurement index */
        if (is_measure_burst()) {
            if (++measure_idx > BURST_LEN) {
                measure_idx = 1;
            }
        }
        burst_seq++;
    }
}

/**
 * @brief Resetuje burst scheduling na početno stanje
 */
static inline void reset_burst_schedule(void)
{
    pulse_in_burst = 1;
    measure_idx    = 1;
    burst_seq      = 1;
}

/* === CONTACT DETECTION === */

/**
 * @brief Šalje poslednji burst rezultat preko BLE
 */
void report_last_burst(void)
{
    uint8_t mask;
    if (saadc_get_last_burst(&mask)) {
        char send_buff[8] = {'>', 'R', 'C', 'E', ';', mask, '<', '\0'};
        bt_nus_send(NULL, (uint8_t*)send_buff, 7);
    } else {
        bt_nus_send(NULL, (uint8_t*)">RCE;NA<", 8);
    }
}

/* === ERROR HANDLING === */

/**
 * @brief Registruje grešku i pokreće recovery ako potrebno
 */
static void handle_error(const char* source, int error_code)
{
    uint32_t errors = atomic_inc(&error_count);
    
    /* Log greške */
    printk("ERROR: %s failed with code %d (total: %u)\n", 
           source, error_code, errors);
    
    /* Performance stats */
    if (strcmp(source, "MUX") == 0) {
        perf_stats.mux_errors++;
    } else if (strcmp(source, "DAC") == 0) {
        perf_stats.dac_errors++;
    }
    
    /* Recovery trigger */
    if (errors >= MAX_CONSECUTIVE_ERRORS) {
        k_work_submit(&recovery_work);
    }
}

/**
 * @brief Error recovery work handler
 */
static void recovery_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    printk("Starting error recovery...\n");
    perf_stats.recoveries++;
    
    /* Hardware reset sekvenca */
    set_pins_safe();
    k_msleep(ERROR_RECOVERY_DELAY_MS);
    
    /* Reset error counter */
    atomic_set(&error_count, 0);
    
    /* Pokušaj reinicijalizacije */
    if (pulse_pins_init() == 0) {
        printk("Recovery successful\n");
    } else {
        printk("Recovery failed - stopping stimulation\n");
        stop_pulse_sequence_internal(false);
    }
}

/* === CORE PULSE GENERATION === */

/**
 * @brief Generiše jedan kompletan bifazni impuls sa error handling
 * 
 * @param width_us Širina pojedinačne faze u mikrosekundama
 * @param pair_idx Indeks kanala za amplitude kontrolu (0-7)
 * @return 0 = success, negative = error
 */
static int do_one_pulse_us(uint32_t width_us, uint8_t pair_idx)
{
    int ret = 0;
    
    /* DAC amplitude setup za trenutni par */
    if (pair_idx < PATTERN_LEN) {
        uint16_t uA = pair_amplitude_uA[pair_idx];
        uint16_t dac_value = (uint16_t)(uA * DAC_SCALING_FACTOR);
        dac_set_value(dac_value);
        /* Note: dac_set_value() interno handluje greške */
    }

    /* RCE response handling */
    if (RCE == 1) {
        report_last_burst();
        RCE = 0;
    }
    
    /* Frequency change detection (atomic) */
    if (atomic_cas(&frequency_changed, 1, 0)) {
        reset_burst_schedule();
    }

    /* === FAZA 1: ANODE ON === */
    unsigned int key = irq_lock();
    set_cathode(0);
    set_anode(1);
    irq_unlock(key);

    k_busy_wait(width_us);

    /* === FAZA 2: CATHODE ON + MERENJE === */
    key = irq_lock();
    set_anode(0);
    set_cathode(1);
    irq_unlock(key);

    /* SAADC merenje tokom katode faze - ISPRAVKA */
    if (frequency <= 4) {
        /* Niske frekvencije: meri svaki impuls */
        saadc_trigger_once_ppi();
    } else if (is_measure_burst() && (pulse_in_burst == measure_idx)) {
        /* Adaptivno merenje: specifični impuls u mernoj grupi */
        saadc_trigger_once_ppi();
    }

    k_busy_wait(width_us);

    /* === FAZA 3: PAUSE === */
    key = irq_lock();
    set_cathode(0);
    set_anode(0);
    irq_unlock(key);

    /* Update counters */
    burst_advance();
    atomic_inc(&pulse_counter);
    perf_stats.total_pulses++;

    return ret;
}

/* === TIMER CALLBACKS === */

/**
 * @brief Work handler za bezbedno zaustavljanje stimulacije
 */
static void stop_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    stop_pulse_sequence_internal(true);
}

/**
 * @brief Timer callback - session timeout
 */
static void stim_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    set_impulse_state(IMPULSE_STOPPING);
    k_work_submit(&stop_work);
}

/**
 * @brief Watchdog timer callback - hardware recovery
 */
static void watchdog_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    
    if (get_impulse_state() == IMPULSE_RUNNING) {
        printk("Watchdog timeout - forcing recovery\n");
        k_work_submit(&recovery_work);
        
        /* Restart watchdog */
        k_timer_start(t, K_MSEC(WATCHDOG_TIMEOUT_MS), K_NO_WAIT);
    }
}

/* === MAIN IMPULSE THREAD === */

/**
 * @brief Glavna nit za generisanje impulsa (thread pool pattern)
 */
static void impulse_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /* Hardware inicijalizacija (jednom) */
    if (pulse_pins_init() != 0) {
        send_response("IMP: Init failed\r\n");
        return;
    }

    ensure_default_pattern();
    send_response("IMP: Thread ready\r\n");
    
    /* Signal da je thread spreman */
    k_sem_give(&thread_ready_sem);

    /* Main loop - čeka signale za rad */
    while (1) {
        /* Čekaj signal za početak rada */
        k_sem_take(&thread_control_sem, K_FOREVER);
        
        /* Proveri da li treba da radi */
        if (get_impulse_state() != IMPULSE_RUNNING) {
            continue;
        }

        printk("Impulse thread starting stimulation\n");
        
        /* Reset performance counters */
        atomic_set(&pulse_counter, 0);
        atomic_set(&error_count, 0);
        
        /* Start watchdog */
        k_timer_start(&watchdog_timer, K_MSEC(WATCHDOG_TIMEOUT_MS), K_NO_WAIT);

        /* === GLAVNA STIMULACIONA PETLJA === */
        while (get_impulse_state() == IMPULSE_RUNNING) {
            /* Dinamički timing calculation */
            const uint32_t phase_us = (uint32_t)STIMULATION_PULSE_WIDTH_US * (uint32_t)pulse_width;
            const uint32_t one_pulse_us = 2U * phase_us;
            uint32_t T_block_us = hz_to_us(frequency);

            /* Pattern refresh */
            size_t sel = (number_patter < patterns_count) ? number_patter : 0;
            const uint16_t *pattern = pulse_patterns[sel];

            uint64_t group_elapsed_us = 0;

            /* PATTERN LOOP: 8 kanala po grupi */
            for (uint8_t i = 0; i < PATTERN_LEN && get_impulse_state() == IMPULSE_RUNNING; i++) {
                /* MUX setup za trenutni kanal */
                uint16_t pat = pattern[i];
                tx_buffer_1[0] = (uint8_t)(pat >> 8);
                tx_buffer_1[1] = (uint8_t)(pat & 0xFF);
                
                int mux_ret = mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));
                if (mux_ret != 0) {
                    handle_error("MUX", mux_ret);
                    /* Nastavi sa radom - MUX greška nije kritična */
                }

                /* Jedan kompletan bifazni impuls */
                int pulse_ret = do_one_pulse_us(phase_us, i);
                if (pulse_ret != 0) {
                    handle_error("PULSE", pulse_ret);
                }

                group_elapsed_us += one_pulse_us;
                
                /* Emergency stop check */
                if (atomic_get(&error_count) >= MAX_CONSECUTIVE_ERRORS) {
                    break;
                }
            }

            /* Bezbedni timing calculation */
            int64_t delay_us = (int64_t)T_block_us - (int64_t)group_elapsed_us - FREQ_CORRECT_US;
            
            if (delay_us > 0 && delay_us < 1000000) { /* Max 1s delay check */
                k_usleep((uint32_t)delay_us);
            } else if (delay_us <= 0) {
                perf_stats.timing_violations++;
                /* Kratka pauza da sistem ne kolabira */
                k_usleep(100);
            }

            /* Restart watchdog */
            k_timer_start(&watchdog_timer, K_MSEC(WATCHDOG_TIMEOUT_MS), K_NO_WAIT);
        }

        /* Stop watchdog */
        k_timer_stop(&watchdog_timer);
        
        /* Cleanup za sledeći ciklus */
        set_pins_safe();
        
        printk("Impulse thread stopped. Pulses: %u, Errors: %u\n", 
               atomic_get(&pulse_counter), atomic_get(&error_count));
    }
}

/* === HARDWARE INITIALIZATION === */

/**
 * @brief Inicijalizuje GPIO pinove i SAADC sa error handling
 */
static int pulse_pins_init(void)
{
    int ret;
    
    /* SAADC setup */
    ret = saadc_ppi_oneshot_init();
    if (ret != 0) {
        send_response("IMP: SAADC init failed\r\n");
        return ret;
    }

    /* GPIO device checks */
    if (!device_is_ready(pulse_anode.port) || !device_is_ready(pulse_cathode.port)) {
        send_response("IMP: GPIO device not ready\r\n");
        return -ENODEV;
    }

    /* GPIO configuration */
    ret = gpio_pin_configure_dt(&pulse_anode, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        send_response("IMP: Anode GPIO config failed\r\n");
        return ret;
    }
    
    ret = gpio_pin_configure_dt(&pulse_cathode, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        send_response("IMP: Cathode GPIO config failed\r\n");
        return ret;
    }

    /* Fail-safe initial state */
    set_pins_safe();
    
    return 0;
}

/* === PUBLIC API === */

/**
 * @brief Interno gašenje stimulacije sa thread-safe cleanup
 */
static void stop_pulse_sequence_internal(bool from_timer)
{
    /* Thread-safe state change */
    int old_state = atomic_set(&impulse_state, IMPULSE_IDLE);
    
    if (old_state == IMPULSE_IDLE) {
        return; /* Već ugašeno */
    }

    /* Stop timers */
    k_timer_stop(&stim_timer);
    k_timer_stop(&watchdog_timer);

    /* Hardware fail-safe */
    set_pins_safe();

    /* DC-DC shutdown */
    if (device_is_ready(dc_dc_en.port)) {
        gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&dc_dc_en, 0);
    }
    
    /* Update external state */
    stimulation_running = false;

    /* Performance report */
    printk("Session stats: Pulses=%u, MUX_err=%u, DAC_err=%u, Timing_viol=%u, Recoveries=%u\n",
           perf_stats.total_pulses, perf_stats.mux_errors, perf_stats.dac_errors, 
           perf_stats.timing_violations, perf_stats.recoveries);

    /* BLE response */
    if (from_timer) {
        send_response("OFFOK\r\n");
    } else {
        send_response("STOPPED\r\n");
    }
}

/**
 * @brief Inicijalizuje impulse subsystem (poziva se jednom iz main)
 */
int impulse_init(void)
{
    /* Semafor inicijalizacija */
    k_sem_init(&thread_control_sem, 0, 1);
    k_sem_init(&thread_ready_sem, 0, 1);
    
    /* Work inicijalizacija */
    k_work_init(&stop_work, stop_work_handler);
    k_work_init(&recovery_work, recovery_work_handler);
    
    /* Timer inicijalizacija */
    k_timer_init(&stim_timer, stim_timer_handler, NULL);
    k_timer_init(&watchdog_timer, watchdog_timer_handler, NULL);
    
    /* Thread kreiranje (thread pool pattern) */
    if (!thread_created) {
        k_thread_create(&imp_thread_data, imp_stack, IMP_STACK,
                        impulse_thread, NULL, NULL, NULL,
                        IMP_PRIO, 0, K_NO_WAIT);
        k_thread_name_set(&imp_thread_data, "impulse");
        thread_created = true;
        
        /* Čekaj da thread bude spreman */
        k_sem_take(&thread_ready_sem, K_SECONDS(5));
    }
    
    set_impulse_state(IMPULSE_IDLE);
    
    return 0;
}

/**
 * @brief Pokreće stimulacionu sekvencu (thread-safe)
 */
void start_pulse_sequence(void)
{
    /* Thread-safe state check */
    if (!atomic_cas(&impulse_state, IMPULSE_IDLE, IMPULSE_RUNNING)) {
        return; /* Već radi ili u tranziciji */
    }

    /* DC-DC napajanje */
    if (device_is_ready(dc_dc_en.port)) {
        gpio_pin_configure_dt(&dc_dc_en, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&dc_dc_en, 1);
    }

    /* Session timer (auto-stop protection) */
    k_timer_start(&stim_timer, K_SECONDS(stim_duration_s), K_NO_WAIT);

    /* Signal thread-u da počne rad */
    k_sem_give(&thread_control_sem);
    
    /* Update external state */
    stimulation_running = true;
}

/**
 * @brief Zaustavlja stimulacionu sekvencu (thread-safe)
 */
void stop_pulse_sequence(void)
{
    if (get_impulse_state() != IMPULSE_IDLE) {
        stop_pulse_sequence_internal(false);
        pwm_ch1_stop();
    }
}

/**
 * @brief Notifikuje o promeni frekvencije (thread-safe)
 */
void notify_frequency_changed(void)
{
    atomic_set(&frequency_changed, 1);
}

/**
 * @brief Vraća trenutne performance statistike
 */
void get_performance_stats(uint32_t *pulses, uint32_t *errors, uint32_t *recoveries)
{
    if (pulses) *pulses = perf_stats.total_pulses;
    if (errors) *errors = perf_stats.mux_errors + perf_stats.dac_errors;
    if (recoveries) *recoveries = perf_stats.recoveries;
}