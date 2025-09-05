// rsens.h
#ifndef RSENS_H
#define RSENS_H

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

void rsens_init(void);
/* Javi RSENS modulu novo stanje katode (poziva impulse.c) */
void rsens_on_cathode_set(bool high);
void read_rsens(void);

/* DODATO: omogući čitanje poslednjeg merenja u mV */
int32_t rsens_get_last_measurement(void);

#ifdef __cplusplus
}
#endif
#endif