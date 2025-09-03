// rsens.c — "always-on" thread + semafor + jednokratni ADC setup (NCS 2.9.1)

#include "rsens.h"
#include "ble_nus.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>

/* ================== PODEŠAVANJA ================== */

/* Uključi periodičan BLE heartbeat (poruka da je thread živ) */
#define RSENS_HEARTBEAT_MS   2000   /* 0 = bez heartbeata */

/* Stack i prioritet za thread:
 * Veći stack jer koristimo snprintk + BLE send_response */
#define RSENS_STACK_SIZE     4096
#define RSENS_PRIORITY       10     /* manji broj = veći prioritet; 10 je "mirno" */

/* ================== ADC KANAL (iz DTS) ================== */

static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

/* Globalni baferi */
static int16_t raw_val;
static int32_t mv_val;
static struct  adc_sequence seq;

/* ================== SINHRONIZACIJA ================== */

/* Semafor za okidanje merenja (možeš ga zvati iz impulse.c) */
K_SEM_DEFINE(rsens_sem, 0, 16);

/* ================== THREAD KONTEKST ================== */

K_THREAD_STACK_DEFINE(rsens_stack, RSENS_STACK_SIZE);
static struct k_thread rsens_thread_data;

/* ================== POMOĆNE FUNKCIJE ================== */

static int rsens_adc_setup_once(void)
{
    if (!device_is_ready(adc_channel.dev)) {
        send_response("RSENS: device not ready\r\n");
        return -ENODEV;
    }

    int err = adc_channel_setup_dt(&adc_channel);
    if (err) {
        char m[48];
        snprintk(m, sizeof(m), "RSENS: setup ERR %d\r\n", err);
        send_response(m);
        return err;
    }

    /* Pripremi sekvencu JEDNOM — ovo skraćuje vreme kasnijeg čitanja */
    seq.channels     = BIT(adc_channel.channel_id);
    seq.buffer       = &raw_val;
    seq.buffer_size  = sizeof(raw_val);
    seq.resolution   = adc_channel.resolution;
    seq.oversampling = 0;
    seq.calibrate    = false;

    return 0;
}

static int rsens_read_mv_once(int32_t *out_mv)
{
    int err = adc_read(adc_channel.dev, &seq);
    if (err) return err;

    int32_t mv = raw_val;
    err = adc_raw_to_millivolts_dt(&adc_channel, &mv);
    if (err) return err;

    *out_mv = mv;
    return 0;
}

/* ================== THREAD BODY (UVEK RADI) ================== */

static void rsens_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    send_response("RSENS thread started\r\n");

    /* Heartbeat tajmer (ako je omogućen) */
#if RSENS_HEARTBEAT_MS > 0
    int64_t next_hb = k_uptime_get() + RSENS_HEARTBEAT_MS;
#endif

    while (1) {
        /* Pokušaj da uzmeš semafor sa kratkim timeout-om,
         * da bi thread “disao” i slao heartbeat po potrebi */
        if (k_sem_take(&rsens_sem, K_MSEC(5)) == 0) {
            /* Okidano merenje */
            int err = rsens_read_mv_once(&mv_val);
            if (err) {
                char m[40];
                snprintk(m, sizeof(m), "RSENS read/conv ERR %d\r\n", err);
                send_response(m);
            } else {
                char buf[32];
                snprintk(buf, sizeof(buf), "RSENS: %ld mV\r\n", (long)mv_val);
                send_response(buf);
            }
        }

#if RSENS_HEARTBEAT_MS > 0
        int64_t now = k_uptime_get();
        if (now >= next_hb) {
            send_response("RSENS alive\r\n");
            next_hb = now + RSENS_HEARTBEAT_MS;
        }
#endif
        /* Kratko “odspavaj” kako ne bi spinovao CPU ako nema trigera */
        k_sleep(K_MSEC(1));
    }
}

/* ================== JAVNI API ================== */

void rsens_init(void)
{
    int err = rsens_adc_setup_once();
    if (err == 0) {
        send_response("RSENS init done\r\n");
    }
    /* Thread uvek startujemo, bez obzira na BLE stanje */
    k_thread_create(&rsens_thread_data, rsens_stack, RSENS_STACK_SIZE,
                    rsens_thread, NULL, NULL, NULL,
                    RSENS_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&rsens_thread_data, "rsens");
}

void rsens_trigger_measurement(void)
{
    /* Zovi ovo npr. odmah nakon gpio_pin_set_dt(&pulse_cathode, 1); */
    send_response("RSENS trigger give\r\n");
    k_sem_give(&rsens_sem);
}
