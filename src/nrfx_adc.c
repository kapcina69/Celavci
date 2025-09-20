/**
 * @file nrfx_adc.c
 * @brief Ultra-fast SAADC module - FIXED COMPILE ERRORS
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SAADC_FAST, LOG_LEVEL_INF);

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_gpio.h>
#include "nrfx_adc.h"
#include "pwm.h"
#include "ble_nus.h"
#include "impulse.h"

#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

/* ====== FIXED CONSTANTS ====== */
#define THRESHOLD_MV              100u
#define THRESHOLD_HYST_MV          10u
#define SAADC_CYCLE_TIMEOUT_MS      2u
#define SAADC_MAINT_PERIOD_MS    5000u
#define SAADC_ABORT_STREAK_REINIT   3u

#define SAADC_FAST_MODE           true
#define SAADC_OVERSAMPLE_DISABLE  true
#define SAADC_ACQTIME_MINIMAL     true

#define OUT_PIN        28u
#define OUT_PIN_MASK   (1UL << OUT_PIN)
BUILD_ASSERT(OUT_PIN < 32, "OUT_PIN must be on P0");

/* ====== INPUT PIN SELECTION ====== */
#if NRF_SAADC_HAS_AIN_AS_PIN
  #if defined(CONFIG_SOC_NRF54L15)
    #define NRF_SAADC_INPUT_AIN4 NRF_PIN_PORT_TO_PIN_NUMBER(11U, 1)
    #define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN4
  #else
    #error "Postavi odgovarajući AIN pin za tvoj SoC."
  #endif
#else
  #define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN3  /* nRF52840: AIN3 = P0.02 */
#endif

/* ====== OPTIMIZED CHANNEL CONFIG ====== */
#if SAADC_FAST_MODE
static nrfx_saadc_channel_t g_chan = {
    .channel_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_3US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED,
    },
    .pin_p = SAADC_INPUT_PIN,
    .pin_n = NRF_SAADC_INPUT_DISABLED,
    .channel_index = 0
};
#else
static nrfx_saadc_channel_t g_chan = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);
#endif

/* ====== ATOMIC STATE VARIABLES ====== */
static atomic_t g_measurement_active = ATOMIC_INIT(0);
static atomic_t g_sample_ready = ATOMIC_INIT(0);
static atomic_t g_error_count = ATOMIC_INIT(0);
static atomic_t g_total_measurements = ATOMIC_INIT(0);

static volatile int16_t g_last_sample;
static volatile bool   g_initialized = false;
static saadc_cb_t      g_user_cb;

static struct k_sem    g_saadc_idle_sem;

/* ====== (D)PPI VARIABLES ====== */
static uint8_t g_ch_started_to_sample;
static bool    g_gppi_allocd;
static bool    g_irq_connected;

/* ====== BURST TRACKING ====== */
#define BURST_LEN       8u
#define BURST_MSB_FIRST 0

#if BURST_MSB_FIRST
  #define BURST_BIT(i) (1u << (BURST_LEN - 1u - (i)))
#else
  #define BURST_BIT(i) (1u << (i))
#endif

static atomic_t g_burst_mask_accum = ATOMIC_INIT(0);
static atomic_t g_burst_idx = ATOMIC_INIT(0);
static atomic_t g_burst_last = ATOMIC_INIT(0);
static atomic_t g_burst_last_valid = ATOMIC_INIT(0);

/* ====== THRESHOLD & GAIN ====== */
static uint32_t g_gain_num = 1, g_gain_den = 1;
static uint16_t g_thresh_code_hi = 0;
static uint16_t g_thresh_code_lo = 0;
static atomic_t g_over_state = ATOMIC_INIT(0);

/* ====== PERFORMANCE COUNTERS ====== */
static struct {
    atomic_t successful_measurements;
    atomic_t failed_measurements;
    atomic_t timeouts;
    atomic_t recoveries;
    atomic_t contact_detections;
    uint32_t max_conversion_time_us;
} perf_stats = {
    .successful_measurements = ATOMIC_INIT(0),
    .failed_measurements = ATOMIC_INIT(0),
    .timeouts = ATOMIC_INIT(0),
    .recoveries = ATOMIC_INIT(0),
    .contact_detections = ATOMIC_INIT(0),
    .max_conversion_time_us = 0
};

/* ====== WORK ITEMS ====== */
static struct k_work_delayable g_saadc_maint;
static struct k_work g_recovery_work;

/* ====== FORWARD DECLARATIONS ====== */
static int  saadc_hw_setup(void);
static int  saadc_reinit_safely(void);
static void saadc_maint_work(struct k_work *work);
static void saadc_recovery_work(struct k_work *work);

/* ====== UTILITY FUNCTIONS ====== */

static inline uint16_t mv_to_code(uint32_t mv)
{
    uint32_t num = mv * g_gain_den * 2048u;
    uint32_t den = 600u * g_gain_num;
    return (uint16_t)(num / den);
}

static void gain_to_mul(nrf_saadc_gain_t g, uint32_t *num, uint32_t *den)
{
    switch (g) {
    case NRF_SAADC_GAIN1_6: *num = 6; *den = 1; break;
    case NRF_SAADC_GAIN1_5: *num = 5; *den = 1; break;
    case NRF_SAADC_GAIN1_4: *num = 4; *den = 1; break;
    case NRF_SAADC_GAIN1_3: *num = 3; *den = 1; break;
    case NRF_SAADC_GAIN1_2: *num = 2; *den = 1; break;
    case NRF_SAADC_GAIN1:   *num = 1; *den = 1; break;
    case NRF_SAADC_GAIN2:   *num = 1; *den = 2; break;
    case NRF_SAADC_GAIN4:   *num = 1; *den = 4; break;
    default:                *num = 1; *den = 1; break;
    }
}

static const char *evt_name(nrfx_saadc_evt_type_t t)
{
    switch (t) {
    case NRFX_SAADC_EVT_DONE:           return "DONE";
    case NRFX_SAADC_EVT_LIMIT:          return "LIMIT";
    case NRFX_SAADC_EVT_CALIBRATEDONE:  return "CALIBRATEDONE";
    case NRFX_SAADC_EVT_BUF_REQ:        return "BUF_REQ";
    case NRFX_SAADC_EVT_READY:          return "READY";
    default:                            return "UNKNOWN";
    }
}

/* ====== SAADC EVENT HANDLER ====== */
static void saadc_evt(nrfx_saadc_evt_t const *e)
{
    switch (e->type) {

    case NRFX_SAADC_EVT_DONE: {
        int16_t s = *((int16_t *)e->data.done.p_buffer);
        if (s < 0) s = 0;
        
        g_last_sample = s;
        atomic_set(&g_sample_ready, 1);
        atomic_set(&g_measurement_active, 0);
        atomic_set(&g_error_count, 0);
        atomic_inc(&perf_stats.successful_measurements);
        atomic_inc(&g_total_measurements);

        /* Contact detection with hysteresis */
        bool current_over = atomic_get(&g_over_state);
        bool over_new = current_over
                        ? ((uint16_t)s >  g_thresh_code_lo)
                        : ((uint16_t)s >= g_thresh_code_hi);

        if (over_new != current_over) {
            atomic_set(&g_over_state, over_new ? 1 : 0);
            
            if (over_new) {
                pwm_ch1_stop();
            } else {
                pwm_ch1_start();
                atomic_inc(&perf_stats.contact_detections);
            }
        }

        /* Burst tracking */
        uint32_t idx = atomic_get(&g_burst_idx);
        uint32_t mask = atomic_get(&g_burst_mask_accum);
        
        if (over_new) {
            mask |= (uint32_t)BURST_BIT(idx);
            atomic_set(&g_burst_mask_accum, mask);
        }
        
        idx++;
        if (idx >= BURST_LEN) {
            atomic_set(&g_burst_last, mask);
            atomic_set(&g_burst_last_valid, 1);
            atomic_set(&g_burst_mask_accum, 0);
            atomic_set(&g_burst_idx, 0);
        } else {
            atomic_set(&g_burst_idx, idx);
        }

        if (g_user_cb) { 
            g_user_cb(s); 
        }

        k_sem_give(&g_saadc_idle_sem);
        break;
    }

    case NRFX_SAADC_EVT_BUF_REQ:
        LOG_DBG("saadc_evt: %s (ignore for 1-shot)", evt_name(e->type));
        break;

    case NRFX_SAADC_EVT_READY:
        LOG_DBG("saadc_evt: %s", evt_name(e->type));
        break;

    case NRFX_SAADC_EVT_CALIBRATEDONE:
        LOG_DBG("saadc_evt: %s", evt_name(e->type));
        break;

    case NRFX_SAADC_EVT_LIMIT:
        LOG_DBG("saadc_evt: %s", evt_name(e->type));
        break;

    default:
        LOG_DBG("saadc_evt: unexpected evt type=%u", (unsigned)e->type);
        atomic_inc(&perf_stats.failed_measurements);
        break;
    }
}

/* ====== HARDWARE SETUP ====== */
static int saadc_hw_setup(void)
{
    nrfx_err_t err;

#if defined(CONFIG_SOC_NRF54L15)
    g_chan.channel_config.gain = NRF_SAADC_GAIN1_4;
#else
    g_chan.channel_config.gain = NRF_SAADC_GAIN1_6;
#endif

    LOG_INF("SAADC FAST setup: gain=%d, input_pin=%u, acq_time=3us", 
            (int)g_chan.channel_config.gain, (unsigned)SAADC_INPUT_PIN);

    err = nrfx_saadc_channels_config(&g_chan, 1);
    if (err != NRFX_SUCCESS) { 
        LOG_ERR("channels_config: 0x%08x", err); 
        return -EIO; 
    }

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    
#if SAADC_FAST_MODE
    adv.oversampling = NRF_SAADC_OVERSAMPLE_DISABLED;
    adv.internal_timer_cc = 0;
    adv.start_on_end = false;
#endif

    err = nrfx_saadc_advanced_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, &adv, saadc_evt);
    if (err != NRFX_SUCCESS) { 
        LOG_ERR("advanced_mode_set: 0x%08x", err); 
        return -EIO; 
    }

    gain_to_mul(g_chan.channel_config.gain, &g_gain_num, &g_gain_den);
    uint32_t hi_mv = THRESHOLD_MV + THRESHOLD_HYST_MV;
    uint32_t lo_mv = (THRESHOLD_MV > THRESHOLD_HYST_MV) ? (THRESHOLD_MV - THRESHOLD_HYST_MV) : 0u;

    g_thresh_code_hi = mv_to_code(hi_mv);
    g_thresh_code_lo = mv_to_code(lo_mv);
    atomic_set(&g_over_state, 0);

    LOG_INF("Fast thresholds: T=%umV, Hyst=%umV -> HI_code=%u, LO_code=%u",
            THRESHOLD_MV, THRESHOLD_HYST_MV, g_thresh_code_hi, g_thresh_code_lo);

    err = nrfx_gppi_channel_alloc(&g_ch_started_to_sample);
    if (err != NRFX_SUCCESS) { 
        LOG_ERR("gppi alloc: 0x%08x", err); 
        return -EIO; 
    }
    g_gppi_allocd = true;

    uint32_t ev_started = nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
    uint32_t t_sample   = nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
    nrfx_gppi_channel_endpoints_setup(g_ch_started_to_sample, ev_started, t_sample);
    nrfx_gppi_channels_enable(BIT(g_ch_started_to_sample));
    
    LOG_INF("GPPI FAST ch=%u: STARTED(0x%08x) -> SAMPLE(0x%08x) enabled",
            g_ch_started_to_sample, ev_started, t_sample);

    atomic_set(&g_measurement_active, 0);
    atomic_set(&g_sample_ready, 0);
    g_user_cb = NULL;

    return 0;
}

/* ====== RECOVERY FUNCTIONS ====== */
static void saadc_recovery_work(struct k_work *work)
{
    ARG_UNUSED(work);
    
    LOG_WRN("SAADC Fast recovery initiated");
    atomic_inc(&perf_stats.recoveries);
    
    nrfx_saadc_abort();
    atomic_set(&g_measurement_active, 0);
    
    k_busy_wait(1000); /* 1ms */
    
    atomic_set(&g_error_count, 0);
    
    if (k_sem_count_get(&g_saadc_idle_sem) == 0) {
        k_sem_give(&g_saadc_idle_sem);
    }
    
    LOG_INF("SAADC Fast recovery completed");
}

static int saadc_reinit_safely(void)
{
    LOG_WRN("SAADC Fast reinit request");

    nrfx_saadc_abort();
    atomic_set(&g_measurement_active, 0);

    if (k_sem_take(&g_saadc_idle_sem, K_MSEC(1)) != 0) {
        LOG_WRN("reinit: sem busy");
        return -EBUSY;
    }

    if (g_gppi_allocd) {
        nrfx_gppi_channels_disable(BIT(g_ch_started_to_sample));
        nrfx_gppi_channel_free(g_ch_started_to_sample);
        LOG_INF("GPPI ch freed: %u", g_ch_started_to_sample);
        g_gppi_allocd = false;
    }

    nrfx_saadc_uninit();
    LOG_INF("SAADC uninit");

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        k_sem_give(&g_saadc_idle_sem);
        LOG_ERR("saadc init (reinit) failed: 0x%08x", err);
        return -EIO;
    }

    int r = saadc_hw_setup();
    k_sem_give(&g_saadc_idle_sem);

    if (r == 0) {
        LOG_INF("SAADC Fast reinit OK");
    } else {
        LOG_ERR("SAADC Fast reinit error: %d", r);
    }
    return r;
}

/* ====== MAINTENANCE ====== */
static void saadc_maint_work(struct k_work *work)
{
    ARG_UNUSED(work);
    
    uint32_t errors = atomic_get(&g_error_count);
    
    if (errors >= SAADC_ABORT_STREAK_REINIT) {
        LOG_WRN("maint: error streak >= %d -> forced reinit", SAADC_ABORT_STREAK_REINIT);
        if (saadc_reinit_safely() == 0) {
            atomic_set(&g_error_count, 0);
            LOG_INF("maint: error streak reset");
        }
    }

    static uint32_t report_counter = 0;
    if (++report_counter >= 10) {
        report_counter = 0;
        /* FIXED: Cast atomic_val_t to unsigned long for %lu format */
        LOG_INF("SAADC Stats: OK=%lu, Fail=%lu, Timeout=%lu, Recovery=%lu, Contact=%lu",
                (unsigned long)atomic_get(&perf_stats.successful_measurements),
                (unsigned long)atomic_get(&perf_stats.failed_measurements),
                (unsigned long)atomic_get(&perf_stats.timeouts),
                (unsigned long)atomic_get(&perf_stats.recoveries),
                (unsigned long)atomic_get(&perf_stats.contact_detections));
    }

    k_work_reschedule(&g_saadc_maint, K_MSEC(SAADC_MAINT_PERIOD_MS));
}

/* ====== PUBLIC API ====== */

int saadc_ppi_oneshot_init(void)
{
    if (g_initialized) {
        LOG_WRN("SAADC already initialized");
        return 0;
    }

    if (!g_irq_connected) {
        IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                    DT_IRQ(DT_NODELABEL(adc), priority),
                    nrfx_isr, nrfx_saadc_irq_handler, 0);
        g_irq_connected = true;
        LOG_INF("SAADC IRQ connected (prio=%d)", DT_IRQ(DT_NODELABEL(adc), priority));
    }

    k_sem_init(&g_saadc_idle_sem, 1, 1);
    LOG_DBG("sem init: count=%d", k_sem_count_get(&g_saadc_idle_sem));

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) { 
        LOG_ERR("saadc_init: 0x%08x", err); 
        return -EIO; 
    }

    int r = saadc_hw_setup();
    if (r) return r;

    k_work_init(&g_recovery_work, saadc_recovery_work);
    k_work_init_delayable(&g_saadc_maint, saadc_maint_work);
    k_work_schedule(&g_saadc_maint, K_MSEC(SAADC_MAINT_PERIOD_MS));
    LOG_INF("maint scheduled in %u ms", (unsigned)SAADC_MAINT_PERIOD_MS);

    g_initialized = true;

    LOG_INF("SAADC FAST 1-shot OK: T=%umV (+/-%umV), codes HI=%u LO=%u, PWM feedback",
            THRESHOLD_MV, THRESHOLD_HYST_MV, g_thresh_code_hi, g_thresh_code_lo);
    return 0;
}

int saadc_trigger_once_ppi(void)
{
    if (!g_initialized) {
        return -ENODEV;
    }

    if (!atomic_cas(&g_measurement_active, 0, 1)) {
        LOG_DBG("trigger: BUSY (atomic)");
        return -EBUSY;
    }

    if (k_sem_take(&g_saadc_idle_sem, K_NO_WAIT) != 0) {
        atomic_set(&g_measurement_active, 0);
        LOG_DBG("trigger: BUSY (sem)");
        return -EBUSY;
    }

    atomic_set(&g_sample_ready, 0);

    nrfx_err_t err = nrfx_saadc_buffer_set((int16_t *)&g_last_sample, 1);
    if (err != NRFX_SUCCESS) {
        atomic_set(&g_measurement_active, 0);
        k_sem_give(&g_saadc_idle_sem);
        
        if (atomic_inc(&g_error_count) >= SAADC_ABORT_STREAK_REINIT) {
            k_work_submit(&g_recovery_work);
        }
        
        LOG_ERR("buffer_set: 0x%08x", err);
        atomic_inc(&perf_stats.failed_measurements);
        return (err == NRFX_ERROR_BUSY) ? -EBUSY : -EIO;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        atomic_set(&g_measurement_active, 0);
        k_sem_give(&g_saadc_idle_sem);
        
        if (atomic_inc(&g_error_count) >= SAADC_ABORT_STREAK_REINIT) {
            k_work_submit(&g_recovery_work);
        }
        
        LOG_ERR("mode_trigger: 0x%08x", err);
        atomic_inc(&perf_stats.failed_measurements);
        return (err == NRFX_ERROR_BUSY) ? -EBUSY : -EIO;
    }

    LOG_DBG("trigger: armed");
    return 0;
}

/* ====== DATA RETRIEVAL ====== */

bool saadc_get_last_burst(uint8_t *mask_out)
{
    if (!atomic_get(&g_burst_last_valid)) {
        LOG_DBG("get_last_burst: no data");
        return false;
    }

    uint8_t v = (uint8_t)atomic_get(&g_burst_last);
    if (mask_out) *mask_out = v;
    
    LOG_INF("get_last_burst: 0x%02X", v);
    return true;
}

uint8_t saadc_get_burst_progress(void)
{
    uint8_t progress = (uint8_t)atomic_get(&g_burst_idx);
    LOG_DBG("burst_progress: %u", progress);
    return progress;
}

bool saadc_get_last(int16_t *out)
{
    if (!atomic_get(&g_sample_ready)) {
        return false;
    }
    
    if (out) {
        *out = g_last_sample;
    }
    return true;
}

void saadc_set_callback(saadc_cb_t cb)
{
    g_user_cb = cb;
    LOG_INF("callback set: %p", (void*)cb);
}

/* ====== PERFORMANCE MONITORING ====== */

void saadc_get_performance_stats(uint32_t *successful, uint32_t *failed, 
                                uint32_t *timeouts, uint32_t *recoveries,
                                uint32_t *contacts, uint32_t *max_time_us)
{
    if (successful) *successful = (uint32_t)atomic_get(&perf_stats.successful_measurements);
    if (failed) *failed = (uint32_t)atomic_get(&perf_stats.failed_measurements);
    if (timeouts) *timeouts = (uint32_t)atomic_get(&perf_stats.timeouts);
    if (recoveries) *recoveries = (uint32_t)atomic_get(&perf_stats.recoveries);
    if (contacts) *contacts = (uint32_t)atomic_get(&perf_stats.contact_detections);
    if (max_time_us) *max_time_us = perf_stats.max_conversion_time_us;
}

void saadc_reset_performance_stats(void)
{
    atomic_set(&perf_stats.successful_measurements, 0);
    atomic_set(&perf_stats.failed_measurements, 0);
    atomic_set(&perf_stats.timeouts, 0);
    atomic_set(&perf_stats.recoveries, 0);
    atomic_set(&perf_stats.contact_detections, 0);
    perf_stats.max_conversion_time_us = 0;
    
    LOG_INF("SAADC performance counters reset");
}

uint32_t saadc_get_total_measurements(void)
{
    return (uint32_t)atomic_get(&g_total_measurements);
}

bool saadc_get_contact_state(void)
{
    return atomic_get(&g_over_state) != 0;
}

/* ====== ENHANCED API ====== */

int saadc_fast_init_for_impulse(void)
{
    int ret = saadc_ppi_oneshot_init();
    if (ret != 0) {
        LOG_ERR("SAADC fast init failed: %d", ret);
        return ret;
    }
    
    ret = saadc_self_test();
    if (ret != 0) {
        LOG_ERR("SAADC self test failed: %d", ret);
        return ret;
    }
    
    LOG_INF("SAADC fast init for impulse: SUCCESS");
    return 0;
}

int saadc_self_test(void)
{
    if (!g_initialized) {
        LOG_ERR("SAADC not initialized");
        return -ENODEV;
    }
    
    LOG_INF("SAADC self-test starting...");
    
    int ret = saadc_trigger_once_ppi();
    if (ret != 0) {
        LOG_ERR("Self-test failed: trigger error %d", ret);
        return ret;
    }
    
    int attempts = 100;
    while (attempts-- > 0 && !atomic_get(&g_sample_ready)) {
        k_usleep(100);
    }
    
    if (!atomic_get(&g_sample_ready)) {
        LOG_ERR("Self-test failed: timeout waiting for sample");
        return -ETIME;
    }
    
    int16_t sample;
    if (!saadc_get_last(&sample)) {
        LOG_ERR("Self-test failed: no sample available");
        return -ENODATA;
    }
    
    LOG_INF("SAADC self-test passed: sample=%d (0x%04X)", sample, (uint16_t)sample);
    return 0;
}

int saadc_set_threshold(uint16_t threshold_mv, uint16_t hysteresis_mv)
{
    if (!g_initialized) {
        return -ENODEV;
    }
    
    if (threshold_mv > 600 || hysteresis_mv > threshold_mv) {
        return -EINVAL;
    }
    
    uint32_t hi_mv = threshold_mv + hysteresis_mv;
    uint32_t lo_mv = (threshold_mv > hysteresis_mv) ? (threshold_mv - hysteresis_mv) : 0u;

    g_thresh_code_hi = mv_to_code(hi_mv);
    g_thresh_code_lo = mv_to_code(lo_mv);
    
    LOG_INF("Threshold updated: %umV ± %umV -> codes %u/%u", 
            threshold_mv, hysteresis_mv, g_thresh_code_hi, g_thresh_code_lo);
    
    return 0;
}

int saadc_get_threshold(uint16_t *threshold_mv, uint16_t *hysteresis_mv)
{
    if (!threshold_mv || !hysteresis_mv) {
        return -EINVAL;
    }
    
    uint32_t hi_mv = (g_thresh_code_hi * 600u * g_gain_num) / (g_gain_den * 2048u);
    uint32_t lo_mv = (g_thresh_code_lo * 600u * g_gain_num) / (g_gain_den * 2048u);
    
    *threshold_mv = (uint16_t)((hi_mv + lo_mv) / 2);
    *hysteresis_mv = (uint16_t)((hi_mv - lo_mv) / 2);
    
    return 0;
}

int saadc_force_recovery(void)
{
    if (!g_initialized) {
        return -ENODEV;
    }
    
    LOG_WRN("SAADC manual recovery requested");
    k_work_submit(&g_recovery_work);
    return 0;
}

uint32_t saadc_get_error_count(void)
{
    return (uint32_t)atomic_get(&g_error_count);
}

bool saadc_is_ready(void)
{
    return g_initialized && (atomic_get(&g_error_count) < SAADC_ABORT_STREAK_REINIT);
}

void saadc_shutdown(void)
{
    if (!g_initialized) {
        return;
    }
    
    LOG_INF("SAADC shutdown initiated");
    
    k_work_cancel_delayable(&g_saadc_maint);
    k_work_cancel(&g_recovery_work);
    
    nrfx_saadc_abort();
    atomic_set(&g_measurement_active, 0);
    
    if (g_gppi_allocd) {
        nrfx_gppi_channels_disable(BIT(g_ch_started_to_sample));
        nrfx_gppi_channel_free(g_ch_started_to_sample);
        g_gppi_allocd = false;
    }
    
    nrfx_saadc_uninit();
    
    g_initialized = false;
    g_user_cb = NULL;
    
    /* FIXED: Cast to unsigned long for proper format */
    LOG_INF("SAADC shutdown complete. Final stats: OK=%lu, Fail=%lu, Recovery=%lu",
            (unsigned long)atomic_get(&perf_stats.successful_measurements),
            (unsigned long)atomic_get(&perf_stats.failed_measurements),
            (unsigned long)atomic_get(&perf_stats.recoveries));
}