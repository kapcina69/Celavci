#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*saadc_cb_t)(int16_t sample);  // poziva se iz ISR-a, neka bude kratko

int  saadc_ppi_oneshot_init(void);    // inicijalizacija SAADC + (D)PPI veze
int  saadc_trigger_once_ppi(void);    // one-shot okidanje (bez čekanja), vraća 0 ili -EBUSY/-EIO
void saadc_set_callback(saadc_cb_t cb);

bool saadc_get_last(int16_t *out);    // opciono: pokupi poslednji uzorak (ako postoji)

#ifdef __cplusplus
}
#endif
