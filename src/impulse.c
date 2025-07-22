#include "impulse.h"
#include "ble_nus.h"

#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

const struct gpio_dt_spec pulse_cathode = GPIO_DT_SPEC_GET(PULSE_CATHODE_NODE, gpios);
const struct gpio_dt_spec pulse_anode   = GPIO_DT_SPEC_GET(PULSE_ANODE_NODE, gpios);
const struct gpio_dt_spec dc_dc_en      = GPIO_DT_SPEC_GET(DC_DC_EN_NODE, gpios);


/**
 * @brief Konvertuje frekvenciju (u Hz) u periodu (u ms)
 *
 * @param frequency_hz Frekvencija u hercima
 * @return uint32_t Perioda u milisekundama
 */
uint32_t hz_to_ms(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return 0; // ili greška: deljenje nulom
    }
    return 1000 / frequency_hz;
}

/* Function to generate the specific pulse sequence */
void generate_pulse_sequence(void)
{
    gpio_pin_set_dt(&pulse_anode, 1);
    printk("Anode ON\n");
    k_sleep(K_USEC(STIMULATION_PULSE_WIDTH_US*pulse_width));
    
    /* Turn off anode */
    gpio_pin_set_dt(&pulse_anode, 0);
    
    gpio_pin_set_dt(&pulse_cathode, 1);
    printk("Cathode ON\n");
    k_sleep(K_USEC(STIMULATION_PULSE_WIDTH_US*pulse_width));
    
    /* Turn off cathode */
    gpio_pin_set_dt(&pulse_cathode, 0);
    
    /* Phase 3: Pause 500ms */
    printk("Pause\n");
    k_sleep(K_MSEC(hz_to_ms(frequency)));
}