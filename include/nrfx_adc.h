#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*saadc_cb_t)(int16_t sample);  

int  saadc_ppi_oneshot_init(void);    // inicijalizacija SAADC + (D)PPI veze
int  saadc_trigger_once_ppi(void);    

bool saadc_get_last(int16_t *out);    

bool saadc_get_last_burst(uint8_t *mask_out);

uint8_t saadc_get_burst_progress(void);
#ifdef __cplusplus
}
#endif
