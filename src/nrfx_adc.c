#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SAADC_1SHOT, LOG_LEVEL_INF);

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_gpio.h>
#include "nrfx_adc.h"
#include "ble_nus.h"
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

/* ====== Pragovi ====== */
#define THRESHOLD_MV          100u
#define THRESHOLD_HYST_MV     10u   /* histereza oko praga */

/* ====== Watchdog za 1-shot ciklus ====== */
#define SAADC_CYCLE_TIMEOUT_MS  5u  /* ako DONE ne stigne za ovoliko ms -> abort */

/* ====== Periodično održavanje ====== */
#define SAADC_MAINT_PERIOD_MS   (30u * 1000u) /* ~0.5 min */
#define SAADC_ABORT_STREAK_REINIT 3            /* posle ovoliko aborta zaredom -> reinit */

/* ====== GPIO izlaz: P0.28 (PO POTREBI PROMENI BROJ) ====== */
#define OUT_PIN        28u          /* 0.28 za buzz */
#define OUT_PIN_MASK   (1UL << OUT_PIN)
BUILD_ASSERT(OUT_PIN < 32, "OUT_PIN must be on P0");

/* ====== SAADC ulaz ====== */
#if NRF_SAADC_HAS_AIN_AS_PIN
  #if defined(CONFIG_SOC_NRF54L15)
    #define NRF_SAADC_INPUT_AIN4 NRF_PIN_PORT_TO_PIN_NUMBER(11U, 1)
    #define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN4
  #else
    #error "Postavi odgovarajući AIN pin za tvoj SoC."
  #endif
#else
  #define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN3
#endif

static nrfx_saadc_channel_t g_chan = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);

/* ====== Stanje ====== */
static volatile int16_t g_sample;
static volatile bool   g_in_progress;
static volatile bool   g_has_last;
static saadc_cb_t      g_user_cb;

/* Binarni semafor: tačno jedan aktivan ciklus */
static struct k_sem    g_saadc_idle_sem;
/* Watchdog timestamper */
static uint32_t        g_cycle_start_ms = 0;
/* Poslednji uspešan DONE (za diagnostiku/health) */
static uint32_t        g_last_done_ms = 0;

/* (D)PPI: SAADC STARTED -> SAMPLE */
static uint8_t g_ch_started_to_sample;
static bool    g_gppi_allocd;
static bool    g_irq_connected;

/* ===== Rezultat po povorci (8 impulsa) ===== */
#define BURST_LEN       8u
#define BURST_MSB_FIRST 0
#if BURST_MSB_FIRST
  #define BURST_BIT(i) (1u << (BURST_LEN - 1u - (i)))
#else
  #define BURST_BIT(i) (1u << (i))
#endif

static volatile uint8_t g_burst_mask_accum = 0; /* bit i: 1 ako je impuls i > prag */
static volatile uint8_t g_burst_idx        = 0; /* 0..7 */
static volatile uint8_t g_burst_last       = 0; /* poslednja kompletna povorka */
static volatile bool    g_burst_last_valid = false;

/* Prag u ADC kodovima (FS = 600mV * (num/den), 12b => 2048) */
static uint32_t g_gain_num = 1, g_gain_den = 1;  /* puni se u setupu */
static uint16_t g_thresh_code_hi = 0;            /* T + hyst */
static uint16_t g_thresh_code_lo = 0;            /* T - hyst */
static bool     g_over_state     = false;        /* latched stanje sa histerezom */

/* Periodično održavanje / reinit heuristika */
static struct k_work_delayable g_saadc_maint;
static atomic_t g_abort_streak = ATOMIC_INIT(0);

/* ===== Deklaracije ===== */
static int  saadc_hw_setup(void);
static int  saadc_reinit_safely(void);
static void saadc_maint_work(struct k_work *work);

/* Pretvori mV u ADC kod: code = mv * den * 2048 / (600 * num) */
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

static inline void drive_out_on_change(bool high)
{
    /* Pročitaj trenutni HW status pina i upiši samo ako se razlikuje */
    const uint32_t is_high = (NRF_P0->OUT & OUT_PIN_MASK) ? 1u : 0u;
    if (is_high == (uint32_t)high) {
        return; /* već je u željenom stanju – bez upisa */
    }
    if (high) { NRF_P0->OUTSET = OUT_PIN_MASK; }
    else       { NRF_P0->OUTCLR = OUT_PIN_MASK; }
}

/* ====== ISR: kratak, determinističan ====== */
static void saadc_evt(nrfx_saadc_evt_t const *e)
{
    if (e->type != NRFX_SAADC_EVT_DONE) {
        return;
    }

    /* 1) Uzorak + status */
    int16_t s = *((int16_t *)e->data.done.p_buffer);
    if (s < 0) s = 0;  /* single-ended: saturiši negativno */
    g_has_last     = true;
    g_in_progress  = false;
    g_last_done_ms = k_uptime_get_32();
    atomic_set(&g_abort_streak, 0);

    /* 2) Prag sa histerezom */
    bool over_new = g_over_state
                    ? ((uint16_t)s >  g_thresh_code_lo)   /* ostani gore dok ne padne ispod LO */
                    : ((uint16_t)s >= g_thresh_code_hi);  /* pređi gore tek preko HI */

    /* Promeni pin samo ako se stanje promenilo (manje thrasha) */
    if (over_new != g_over_state) {
        g_over_state = over_new;
        if (g_over_state) { NRF_P0->OUTSET = OUT_PIN_MASK; }
        else              { NRF_P0->OUTCLR = OUT_PIN_MASK; }
    }

    /* 3) Akumulacija maske povorke */
    uint8_t idx = g_burst_idx;
    if (over_new) {
        g_burst_mask_accum |= (uint8_t)BURST_BIT(idx);
    }

    idx++;
    if (idx >= BURST_LEN) {
        g_burst_last       = g_burst_mask_accum;
        g_burst_last_valid = true;
        g_burst_mask_accum = 0;
        g_burst_idx        = 0;
    } else {
        g_burst_idx = idx;
    }

    /* 4) Lagan callback (bez BLE/log u ISR-u) */
    if (g_user_cb) { g_user_cb(s); }

    /* 5) Oslobodi watchdog/semafor */
    g_cycle_start_ms = 0;
    k_sem_give(&g_saadc_idle_sem);
}

/* ====== HW setup: kanal, adv mode, GPPI, pragovi ====== */
static int saadc_hw_setup(void)
{
    nrfx_err_t err;

#if defined(CONFIG_SOC_NRF54L15)
    g_chan.channel_config.gain = NRF_SAADC_GAIN1_4;
#else
    g_chan.channel_config.gain = NRF_SAADC_GAIN1_6;
#endif
    err = nrfx_saadc_channels_config(&g_chan, 1);
    if (err != NRFX_SUCCESS) { LOG_ERR("channels_config: 0x%08x", err); return -EIO; }

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, &adv, saadc_evt);
    if (err != NRFX_SUCCESS) { LOG_ERR("advanced_mode_set: 0x%08x", err); return -EIO; }

    /* Koeficijenti i pragovi (posle postavljanja gain-a) */
    gain_to_mul(g_chan.channel_config.gain, &g_gain_num, &g_gain_den);

    uint32_t hi_mv = THRESHOLD_MV + THRESHOLD_HYST_MV;
    uint32_t lo_mv = (THRESHOLD_MV > THRESHOLD_HYST_MV) ? (THRESHOLD_MV - THRESHOLD_HYST_MV) : 0u;

    g_thresh_code_hi = mv_to_code(hi_mv);
    g_thresh_code_lo = mv_to_code(lo_mv);
    g_over_state     = false;

    /* (D)PPI: STARTED → SAMPLE (nulta latencija) */
    err = nrfx_gppi_channel_alloc(&g_ch_started_to_sample);
    if (err != NRFX_SUCCESS) { LOG_ERR("gppi alloc: 0x%08x", err); return -EIO; }
    g_gppi_allocd = true;

    nrfx_gppi_channel_endpoints_setup(
        g_ch_started_to_sample,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_STARTED),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));
    nrfx_gppi_channels_enable(BIT(g_ch_started_to_sample));

    g_in_progress = false;
    g_has_last    = false;
    g_user_cb     = NULL;

    return 0;
}

/* ====== Siguran re-init (samo kada smo idle, van ISR-a) ====== */
static int saadc_reinit_safely(void)
{
    /* prekini eventualnu aktivnu konverziju */
    nrfx_saadc_abort();
    g_in_progress = false;
    g_cycle_start_ms = 0;

    /* pokušaj da obezbediš ekskluzivu kratko */
    if (k_sem_take(&g_saadc_idle_sem, K_MSEC(1)) != 0) {
        return -EBUSY;
    }

    if (g_gppi_allocd) {
        nrfx_gppi_channels_disable(BIT(g_ch_started_to_sample));
        nrfx_gppi_channel_free(g_ch_started_to_sample);
        g_gppi_allocd = false;
    }

    nrfx_saadc_uninit();

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        k_sem_give(&g_saadc_idle_sem);
        LOG_ERR("saadc re-init failed: 0x%08x", err);
        return -EIO;
    }
    int r = saadc_hw_setup();
    k_sem_give(&g_saadc_idle_sem);

    if (r == 0) {
        LOG_INF("SAADC reinit OK");
    }
    return r;
}

/* ====== Periodično održavanje (kalibracija + uslovni reinit) ====== */
static void saadc_maint_work(struct k_work *work)
{
    ARG_UNUSED(work);
    const uint32_t now = k_uptime_get_32();

    /* 1) Ako ciklus traje predugo -> abort + broj zaglavljenja */
    if (g_in_progress && g_cycle_start_ms &&
        (int32_t)(now - g_cycle_start_ms) > (int32_t)SAADC_CYCLE_TIMEOUT_MS) {

        nrfx_saadc_abort();
        g_in_progress = false;
        g_cycle_start_ms = 0;

        /* oslobodi semafor ako je prazan */
        if (k_sem_count_get(&g_saadc_idle_sem) == 0) {
            k_sem_give(&g_saadc_idle_sem);
        }

        atomic_inc(&g_abort_streak);
        LOG_WRN("SAADC watchdog: abortovao zaglavljenu konverziju");
    }

    /* 2) Ako smo idle, pokušaj kalibraciju s vremena na vreme (svaki ciklus) */
    if (!g_in_progress && k_sem_count_get(&g_saadc_idle_sem) > 0) {

            (void)saadc_reinit_safely();
        
    }

    /* 3) Ako pređe n aborta zaredom, uradi re-init */
    if (atomic_get(&g_abort_streak) >= SAADC_ABORT_STREAK_REINIT) {
        if (saadc_reinit_safely() == 0) {
            atomic_set(&g_abort_streak, 0);
        }
    }

    /* 4) resched */
    k_work_reschedule(&g_saadc_maint, K_MSEC(SAADC_MAINT_PERIOD_MS));
}

/* ====== API ====== */

int saadc_ppi_oneshot_init(void)
{
    /* P0.x kao izlaz (default low) */
    nrf_gpio_cfg_output(OUT_PIN);
    NRF_P0->OUTCLR = OUT_PIN_MASK;

    /* IRQ CONNECT samo jednom */
    if (!g_irq_connected) {
        IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                    DT_IRQ(DT_NODELABEL(adc), priority),
                    nrfx_isr, nrfx_saadc_irq_handler, 0);
        g_irq_connected = true;
    }

    /* Semafor: dozvoli odmah jedan trigger */
    k_sem_init(&g_saadc_idle_sem, 1, 1);

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) { LOG_ERR("saadc_init: 0x%08x", err); return -EIO; }

    int r = saadc_hw_setup();
    if (r) return r;

    /* Startuj periodično održavanje */
    k_work_init_delayable(&g_saadc_maint, saadc_maint_work);
    k_work_schedule(&g_saadc_maint, K_MSEC(SAADC_MAINT_PERIOD_MS));

    LOG_INF("SAADC 1-shot OK: T=%umV (+/-%umV), codes HI=%u LO=%u, P0.%u out",
            THRESHOLD_MV, THRESHOLD_HYST_MV, g_thresh_code_hi, g_thresh_code_lo, OUT_PIN);
    return 0;
}

/* Triggeruj jednu konverziju; sam odglavljuje ako je prethodna zapela */
int saadc_trigger_once_ppi(void)
{
    /* 1) Ako je prethodni ciklus zapinjao – abort + odglavi */
    uint32_t now = k_uptime_get_32();
    if (g_in_progress && g_cycle_start_ms != 0 &&
        (int32_t)(now - g_cycle_start_ms) > (int32_t)SAADC_CYCLE_TIMEOUT_MS) {

        nrfx_saadc_abort();
        g_in_progress = false;
        g_cycle_start_ms = 0;

        if (k_sem_count_get(&g_saadc_idle_sem) == 0) {
            k_sem_give(&g_saadc_idle_sem);
        }

        atomic_inc(&g_abort_streak);
        LOG_WRN("SAADC watchdog (trigger): abortovao zaglavljenu konverziju");
    }

    /* 2) Uzmi semafor sa kratkim tajmautom (smanjuje false-BUSY šum) */
    if (k_sem_take(&g_saadc_idle_sem, K_USEC(200)) != 0) {
        send_response("BUSY");
        saadc_reinit_safely();
        return -EBUSY;
    }

    g_in_progress = true;
    g_has_last    = false;

    nrfx_err_t err = nrfx_saadc_buffer_set((int16_t *)&g_sample, 1);
    if (err != NRFX_SUCCESS) {
        g_in_progress = false;
        if (k_sem_count_get(&g_saadc_idle_sem) == 0) {
            k_sem_give(&g_saadc_idle_sem);
        }
        LOG_ERR("buffer_set: 0x%08x", err);
        return (err == NRFX_ERROR_BUSY) ? -EBUSY : -EIO;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        g_in_progress = false;
        if (k_sem_count_get(&g_saadc_idle_sem) == 0) {
            k_sem_give(&g_saadc_idle_sem);
        }
        LOG_ERR("mode_trigger: 0x%08x", err);
        return (err == NRFX_ERROR_BUSY) ? -EBUSY : -EIO;
    }

    /* 3) Armiraj watchdog */
    g_cycle_start_ms = k_uptime_get_32();

    return 0;
}

/* Poslednji kompletan 8-bitni rezultat; true ako postoji važeći rezultat */
bool saadc_get_last_burst(uint8_t *mask_out)
{
    if (!g_burst_last_valid) return false;

    /* 8-bitni read je atomičan, ali zaključaj zbog flag-a */
    unsigned int key = irq_lock();
    uint8_t v = g_burst_last;
    irq_unlock(key);

    if (mask_out) *mask_out = v;
    return true;
}

/* Napredak tekuće povorke (0..8) */
uint8_t saadc_get_burst_progress(void)
{
    return g_burst_idx;
}

/* Postavi lagan callback (ne raditi BLE/log u ISR-u) */
void saadc_set_callback(saadc_cb_t cb)
{
    g_user_cb = cb;
}
