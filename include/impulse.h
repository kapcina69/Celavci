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
 * @brief Converts frequency in Hz to period in microseconds.
 *
 * @param frequency_hz Frequency in Hertz. If 0, the function returns 0.
 * @return Period in microseconds. Returns 0 if input frequency is 0.
 *
 * @note This function performs integer division: 1 second = 1,000,000 microseconds.
 */
uint32_t hz_to_us(uint32_t frequency_hz);


/**
 * @brief Timer callback handler that drives the stimulation pulse sequence.
 *
 * This function controls the state machine that toggles GPIO pins to generate
 * a biphasic stimulation pulse (anode and cathode phases) and applies a specific
 * pulse pattern to the multiplexer (MUX) after each pulse.
 *
 * States:
 * - PULSE_ANODE_ON: Activates the anode, starts timer for pulse width duration.
 * - PULSE_CATHODE_ON: Activates the cathode, starts timer for pulse width.
 * - PULSE_PAUSE: Turns off both anode and cathode, updates MUX pattern, and delays based on frequency.
 * - PULSE_IDLE: Waits until next stimulation trigger.
 *
 * @param timer Pointer to the Zephyr kernel timer object.
 */
void pulse_timer_handler(struct k_timer *timer);


/**
 * @brief Starts a new stimulation pulse sequence.
 *
 * Initializes the MUX with the first pattern, activates the anode, and starts the timer.
 * This function only starts the sequence if the current state is PULSE_IDLE.
 */
void start_pulse_sequence(void);


/**
 * @brief Stops the ongoing stimulation pulse sequence.
 *
 * Stops the timer, disables anode and cathode output, and resets the state to PULSE_IDLE.
 */
void stop_pulse_sequence(void);


/**
 * @brief Triggers the generation of a stimulation pulse sequence.
 *
 * This is a wrapper around start_pulse_sequence(), typically used as an external API.
 */
void generate_pulse_sequence(void);


#endif /* IMPULSE_H */