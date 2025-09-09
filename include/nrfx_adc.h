#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*saadc_cb_t)(int16_t sample);  // poziva se iz ISR-a, neka bude kratko

int  saadc_ppi_oneshot_init(void);    // inicijalizacija SAADC + (D)PPI veze
int  saadc_trigger_once_ppi(void);    // one-shot okidanje (bez čekanja), vraća 0 ili -EBUSY/-EIO

bool saadc_get_last(int16_t *out);    // opciono: pokupi poslednji uzorak (ako postoji)
/* Vrati poslednji kompletan 8-bitni rezultat (za prethodnu završenu povorku).
 * Ako postoji važeći rezultat vrati true i upiše masku u *mask_out.
 * Rezultat ostaje dostupan i tokom sledeće povorke. */
bool saadc_get_last_burst(uint8_t *mask_out);

/* (opciono) Vrati koliko je impulsa već obrađeno u TEKUĆOJ povorci (0..8). */
uint8_t saadc_get_burst_progress(void);
#ifdef __cplusplus
}
#endif
