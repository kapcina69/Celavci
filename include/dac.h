/**
 * @file dac.h
 * @brief I2C upravljanje eksternim 10‑bitnim DAC‑om.
 *
 * API obezbeđuje inicijalizaciju I2C magistrale i upis 10‑bitne vrednosti
 * (0–1023) u DAC izlazni registar.
 */

#ifndef DAC_H
#define DAC_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/**
 * @def DAC_ADDR
 * @brief I2C adresa DAC uređaja.
 */
#define DAC_ADDR 0x63

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicijalizuje I2C i priprema DAC za rad.
 *
 * @retval 0  Uspeh.
 * @retval <0 Negativan errno kod (npr. uređaj nije spreman).
 */
int dac_init(void);

/**
 * @brief Postavlja 10‑bitnu vrednost na DAC izlazu.
 *
 * @param value Vrednost 0–1023 (višak se seče na opseg).
 *
 * @note Funkcija ne blokira duže od I2C transfera.
 */
void dac_set_value(uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* DAC_H */