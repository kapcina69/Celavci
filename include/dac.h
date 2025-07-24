#ifndef DAC_H
#define DAC_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "ble_nus.h"
#include <zephyr/drivers/i2c.h>

#define DAC_ADDR 0x63

/* Eksterna promenljiva za amplitudu - mora biti definisana u main fajlu */

/**
 * @brief Inicijalizacija DAC periferije
 * @return 0 na uspeh, negativan error kod u slučaju greške
 */
int dac_init(void);

/**
 * @brief Postavlja vrednost na DAC
 * @param value Vrednost za upis (0-1023)
 */
void dac_set_value(uint16_t value);



#endif /* DAC_H */