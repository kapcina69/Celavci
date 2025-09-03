// rsens_simple.c - Pojednostavljeni pristup bez direktnog IRQ
#include "rsens.h"
#include "ble_nus.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>

static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static int32_t last_mv;

void rsens_init(void)
{
    if (!device_is_ready(adc_channel.dev)) {
        send_response("RSENS: ADC device not ready\r\n");
        return;
    }

    int err = adc_channel_setup_dt(&adc_channel);
    if (err) {
        char error_msg[48];
        snprintk(error_msg, sizeof(error_msg), "RSENS: setup error %d\r\n", err);
        send_response(error_msg);
        return;
    }

    send_response("RSENS initialization complete\r\n");
}

void read_rsens(void)
{
    int16_t buf;
    int32_t mv_value;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    adc_sequence_init_dt(&adc_channel, &sequence);
    
    int err = adc_read(adc_channel.dev, &sequence);
    if (err) {
        send_response("RSENS: read error\r\n");
        return;
    }

    mv_value = buf;
    err = adc_raw_to_millivolts_dt(&adc_channel, &mv_value);
    
    if (err == 0) {
        last_mv = mv_value;
        char buf_msg[32];
        snprintk(buf_msg, sizeof(buf_msg), "RSENS: %ld mV\r\n", (long)mv_value);
        send_response(buf_msg);
    } else {
        send_response("RSENS: conversion error\r\n");
    }
}

void rsens_on_cathode_set(bool high)
{
    if (high) {
        read_rsens(); // Blokirajući poziv, ali jednostavnije
    }
}

int32_t rsens_get_last_measurement(void)
{
    return last_mv;
}