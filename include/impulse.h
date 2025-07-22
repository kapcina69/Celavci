#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>


/*Define protocol parameters*/
#define STIMULATION_PULSE_WIDTH_US 100

/* Get node identifiers from aliases */
#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE DT_ALIAS(dc_dc_en)

/* GPIO specifications */
extern const struct gpio_dt_spec pulse_cathode;
extern const struct gpio_dt_spec pulse_anode;
extern const struct gpio_dt_spec dc_dc_en;

void generate_pulse_sequence(void);
