#ifndef DAC_H
#define DAC_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include "ble_nus.h"

#define DAC_ADDR 0x63 /**< I2C address of the external DAC device. */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the DAC peripheral over I2C.
 *
 * Configures the I2C interface and prepares the DAC for operation.
 *
 * @return 0 on success, or a negative error code on failure.
 */
int dac_init(void);

/**
 * @brief Sets the output value of the DAC.
 *
 * Sends a 10-bit value (0–1023) to the DAC over I2C to control analog output.
 *
 * @param value The value to write to the DAC (0–1023).
 */
void dac_set_value(uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* DAC_H */
