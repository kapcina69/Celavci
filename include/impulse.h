#ifndef IMPULSE_H
#define IMPULSE_H

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* Define protocol parameters */
#define STIMULATION_PULSE_WIDTH_US 100

static uint8_t tx_buffer_1[2];


/* Get node identifiers from aliases */
#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE DT_ALIAS(dc_dc_en)

/* GPIO specifications */
extern const struct gpio_dt_spec pulse_cathode;
extern const struct gpio_dt_spec pulse_anode;
extern const struct gpio_dt_spec dc_dc_en;

extern struct mux_config stim_mux_config;


/**
 * @brief Convert frequency (in Hz) to period (in ms)
 *
 * @param frequency_hz Frequency in hertz
 * @return uint32_t Period in milliseconds
 */uint32_t hz_to_ms(uint32_t frequency_hz);

 /**
  * @brief Initialize the pulse timer
  */
void pulse_timer_init(void);

/**
 * @brief Start the pulse sequence
 */
void start_pulse_sequence(void);

/**
 * @brief Stop the pulse sequence
 */
void stop_pulse_sequence(void);

/**
 * @brief Generate a pulse sequence
 */
void generate_pulse_sequence(void);

#endif /* IMPULSE_H */