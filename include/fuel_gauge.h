#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

void bq27220_init(void);
void bq27220_read_basic(void);   /* šalje preko BLE: mV, %, °C, mA */
void i2c_scan_all(void);