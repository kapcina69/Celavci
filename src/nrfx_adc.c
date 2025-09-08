#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SAADC_1SHOT, LOG_LEVEL_INF);

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>
#include "nrfx_adc.h"
#include "ble_nus.h"
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

/* ====== Prag ====== */
#define THRESHOLD_MV 100

/* ====== GPIO izlaz: P0.28 ====== */
#define GPIO0_NODE DT_NODELABEL(gpio0)
static const struct device * const port0 = DEVICE_DT_GET(GPIO0_NODE);
/* Ako zaista koristiš P0.28, postavi na 28. (trenutno 3 po tvom kodu) */
#define OUT_PIN 3  /* P0.28 */

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

/* Jedini bafer: 1 uzorak */
static volatile int16_t g_sample;
static volatile bool   g_in_progress;
static volatile bool   g_has_last;
static saadc_cb_t      g_user_cb;

/* (D)PPI: SAADC STARTED -> SAMPLE */
static uint8_t g_ch_started_to_sample;
static bool    g_gppi_allocd;

/* IRQ povezan samo jednom */
static bool    g_irq_connected;

/* Konverzija koda u mV: FS = 600mV * (num/den), 12-bit → delilac 2048 */
static uint32_t g_gain_num = 1, g_gain_den = 1;  /* popunjava se u init-u */
static inline int32_t sample_to_mV(int16_t s)
{
    if (s < 0) s = 0; /* single-ended */
    /* mV = s * (600 * num) / (den * 2048) */
    int32_t num = (int32_t)s * 600 * (int32_t)g_gain_num;
    int32_t den = (int32_t)g_gain_den * 2048;
    return num / den;
}

/* ===== Deklaracije reset/ponovni setup ===== */
static int  saadc_hw_setup(void);
static void saadc_full_reset(const char *reason);

/* ====== SAADC event handler ====== */
static void saadc_evt(nrfx_saadc_evt_t const * e)
{
    switch (e->type) {
    case NRFX_SAADC_EVT_DONE: {
        int16_t s = *((int16_t *)e->data.done.p_buffer);
        g_has_last = true;
        g_in_progress = false;

        /* Prag → postavi P0.28 (direktno iz ISR-a) */
        int32_t mv = sample_to_mV(s);
        gpio_pin_set(port0, OUT_PIN, (mv > THRESHOLD_MV) ? 1 : 0);

        if (g_user_cb) { g_user_cb(s); }
        break;
    }
    case NRFX_SAADC_EVT_CALIBRATEDONE:
        LOG_INF("SAADC calibrate done");
        break;
    default:
        /* Nema error event-a u nrfx_saadc, ali loguj sve ostalo */
        LOG_DBG("SAADC evt=%d", e->type);
        break;
    }
}

static void gain_to_mul(nrf_saadc_gain_t g, uint32_t *num, uint32_t *den)
{
    /* FS = 600mV * (num/den) */
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

/* ====== Interno: kompletan HW setup (kanal, adv mode, GPPI) ====== */
static int saadc_hw_setup(void)
{
    nrfx_err_t err;

#if defined(CONFIG_SOC_NRF54L15)
    g_chan.channel_config.gain = NRF_SAADC_GAIN1_4;
#else
    g_chan.channel_config.gain = NRF_SAADC_GAIN1_6;
#endif
    err = nrfx_saadc_channels_config(&g_chan, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("channels_config: 0x%08x", err);
        return -EIO;
    }

    nrfx_saadc_adv_config_t adv = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT, &adv, saadc_evt);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("advanced_mode_set: 0x%08x", err);
        return -EIO;
    }

    /* koeficijenti za mV */
    gain_to_mul(g_chan.channel_config.gain, &g_gain_num, &g_gain_den);

    /* (D)PPI: STARTED → SAMPLE (nulta latencija) */
    err = nrfx_gppi_channel_alloc(&g_ch_started_to_sample);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("gppi alloc: 0x%08x", err);
        return -EIO;
    }
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

/* ====== Potpuni reset SAADC + GPPI i re-init ====== */
static void saadc_full_reset(const char *reason)
{
    LOG_WRN("SAADC reset (%s)", reason ? reason : "n/a");

    if (g_gppi_allocd) {
        nrfx_gppi_channels_disable(BIT(g_ch_started_to_sample));
        nrfx_gppi_channel_free(g_ch_started_to_sample);
        g_gppi_allocd = false;
    }

    nrfx_saadc_uninit();

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc re-init failed: 0x%08x", err);
        /* Ako ni init ne prođe, prijavi preko BLE pa izađi */
        send_response(">SAADC_RESET_FAIL<");
        return;
    }

    int r = saadc_hw_setup();
    if (r) {
        LOG_ERR("saadc_hw_setup after reset failed: %d", r);
        send_response(">SAADC_RESET_FAIL<");
        return;
    }

    send_response(">SAADC_RESET_OK<");
}

/* ====== API ====== */

int saadc_ppi_oneshot_init(void)
{
    /* GPIO init: P0.28 kao izlaz, startno 0 */
    if (!device_is_ready(port0)) {
        LOG_ERR("gpio0 not ready");
        return -ENODEV;
    }
    int ret = gpio_pin_configure(port0, OUT_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret) {
        LOG_ERR("gpio_pin_configure P0.%d err=%d", OUT_PIN, ret);
        return ret;
    }

    /* IRQ CONNECT samo prvi put */
    if (!g_irq_connected) {
        IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
                    DT_IRQ(DT_NODELABEL(adc), priority),
                    nrfx_isr, nrfx_saadc_irq_handler, 0);
        g_irq_connected = true;
    }

    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("saadc_init: 0x%08x", err);
        saadc_full_reset("init_fail");
        return -EIO;
    }

    int r = saadc_hw_setup();
    if (r) {
        saadc_full_reset("hw_setup_fail");
        return r;
    }

    LOG_INF("One-shot SAADC+PPI sa pragom %dmV spreman (P0.%d kao izlaz)", THRESHOLD_MV, OUT_PIN);
    return 0;
}

int saadc_trigger_once_ppi(void)
{
    if (g_in_progress) {
        send_response("BUSY");
        saadc_full_reset("trigger_while_busy");
        return -EBUSY;
    }
    g_in_progress = true;
    g_has_last    = false;

    nrfx_err_t err = nrfx_saadc_buffer_set((int16_t *)&g_sample, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("buffer_set: 0x%08x", err);
        g_in_progress = false;

        /* Reset i jedan retry */
        saadc_full_reset("buffer_set");
        err = nrfx_saadc_buffer_set((int16_t *)&g_sample, 1);
        if (err != NRFX_SUCCESS) {
            LOG_ERR("buffer_set after reset: 0x%08x", err);
            return -EIO;
        }
    }

    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("mode_trigger: 0x%08x", err);
        g_in_progress = false;

        /* Reset i jedan retry */
        saadc_full_reset("mode_trigger");
        err = nrfx_saadc_mode_trigger();
        if (err != NRFX_SUCCESS) {
            LOG_ERR("mode_trigger after reset: 0x%08x", err);
            return -EIO;
        }
    }

    return 0;
}

void saadc_set_callback(saadc_cb_t cb) { g_user_cb = cb; }

bool saadc_get_last(int16_t *out)
{
    if (!g_has_last) return false;
    if (out) *out = (int16_t)g_sample;
    return true;
}

/* Opciona javna funkcija za ručni reset iz drugih delova koda */
void saadc_force_reset(void)
{
    saadc_full_reset("manual");
}
