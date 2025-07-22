#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include "impulse.h"
#include "ble_nus.h"

/*DAC parameters*/
#define DAC_ADDR 0x63
extern const struct device *i2c_dev;

void dac_set_value(uint16_t value);