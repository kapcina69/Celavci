#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

#ifdef __cplusplus
extern "C" {
#endif





/**
 * @brief Inicijalizuje ADC kanale povezane na NTC senzore.
 *
 * Podesi ADC kanale definisane u devicetree konfiguraciji.
 *
 * @return 0 u slučaju uspeha, negativan kod greške u suprotnom.
 */
int init_ntc_adc(void);

/**
 * @brief Čita napone sa svih NTC senzora.
 *
 * @param voltages Niz u koji se smeštaju očitani naponi (mV).
 * @return 0 u slučaju uspeha, negativan kod greške u suprotnom.
 */
int read_ntcs_voltages(int32_t voltages);

#ifdef __cplusplus
}
#endif
