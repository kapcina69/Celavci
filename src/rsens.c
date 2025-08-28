#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <stdio.h>   
#include "ble_nus.h"

#define ADC_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_channel = 
    ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);


int init_ntc_adc(void)
{
    if(!device_is_ready(adc_channel.dev)) {
        send_response("ADC device not ready\r\n");
        return -ENODEV;
    }
    int err = adc_channel_setup_dt(&adc_channel);
    if (err) {
        send_response("ADC channel setup failed\r\n");
        return err;
    }
    return 0;
}

int read_ntcs_voltage_and_report(void)
{
    if (!device_is_ready(adc_channel.dev)) {
        send_response("NTC: device not ready\r\n");
        return -ENODEV;
    }

    /* (Re)setup je bezbedan i ako je već urađen */
    int err = adc_channel_setup_dt(&adc_channel);
    if (err) {
        char m[48]; snprintf(m, sizeof(m), "NTC: setup ERR %d\r\n", err);
        send_response(m);
        return err;
    }

    /* Jedna vrednost koja služi i kao RAW i kao mV (in-place konverzija) */
    int32_t val = 0;

    struct adc_sequence seq = {
        .channels     = BIT(adc_channel.channel_id),
        .buffer       = &val,
        .buffer_size  = sizeof(val),
        .resolution   = adc_channel.resolution,
        .oversampling = 0,
        .calibrate    = false,
    };

    err = adc_read(adc_channel.dev, &seq);
    if (err) {
        char m[48]; snprintf(m, sizeof(m), "NTC: read ERR %d\r\n", err);
        send_response(m);
        return err;
    }

    /* RAW -> mV (NCS 2.9.1: 2 argumenta, in-place) */
    err = adc_raw_to_millivolts_dt(&adc_channel, &val);
    if (err) {
        char m[48]; snprintf(m, sizeof(m), "NTC: conv ERR %d\r\n", err);
        send_response(m);
        return err;
    }

    char line[48];
    snprintf(line, sizeof(line), "RSENS: %ld mV\r\n", (long)val);
    send_response(line);
    return 0;
}