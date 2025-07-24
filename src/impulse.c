#include "impulse.h"
#include "ble_nus.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

const struct gpio_dt_spec pulse_cathode = GPIO_DT_SPEC_GET(PULSE_CATHODE_NODE, gpios);
const struct gpio_dt_spec pulse_anode   = GPIO_DT_SPEC_GET(PULSE_ANODE_NODE, gpios);
const struct gpio_dt_spec dc_dc_en      = GPIO_DT_SPEC_GET(DC_DC_EN_NODE, gpios);

static struct k_timer pulse_timer;
static enum {
    PULSE_IDLE,
    PULSE_ANODE_ON,
    PULSE_CATHODE_ON,
    PULSE_PAUSE
} pulse_state = PULSE_IDLE;

/**
 * @brief Convert frequency (in Hz) to period (in ms)
 *
 * @param frequency_hz Frequency in hertz
 * @return uint32_t Period in milliseconds
 */
uint32_t hz_to_ms(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return 0; // or error: division by zero
    }
    return 1000 / frequency_hz;
}

void pulse_timer_handler(struct k_timer *timer)
{
    switch (pulse_state) {
        case PULSE_ANODE_ON:
            /* Turn off anode and turn on cathode */
            gpio_pin_set_dt(&pulse_anode, 0);
            gpio_pin_set_dt(&pulse_cathode, 1);
            pulse_state = PULSE_CATHODE_ON;
            k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
            break;
            
        case PULSE_CATHODE_ON:
            /* Turn off cathode */
            gpio_pin_set_dt(&pulse_cathode, 0);
            pulse_state = PULSE_PAUSE;
            k_timer_start(&pulse_timer, K_MSEC(hz_to_ms(frequency)), K_NO_WAIT);
            break;
            
        case PULSE_PAUSE:
            /* Start new pulse sequence */
            gpio_pin_set_dt(&pulse_anode, 1);
            pulse_state = PULSE_ANODE_ON;
            k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
            break;
            
        case PULSE_IDLE:
        default:
            /* Do nothing */
            break;
    }
}

/* Initialize the pulse timer */
void pulse_timer_init(void)
{
    k_timer_init(&pulse_timer, pulse_timer_handler, NULL);
}

/* Function to start the pulse sequence */
void start_pulse_sequence(void)
{
    if (pulse_state == PULSE_IDLE) {
        gpio_pin_set_dt(&pulse_anode, 1);
        pulse_state = PULSE_ANODE_ON;
        k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
    }
}

/* Function to stop the pulse sequence */
void stop_pulse_sequence(void)
{
    k_timer_stop(&pulse_timer);
    gpio_pin_set_dt(&pulse_anode, 0);
    gpio_pin_set_dt(&pulse_cathode, 0);
    pulse_state = PULSE_IDLE;
}

/* Function to generate the specific pulse sequence */
void generate_pulse_sequence(void)
{
    start_pulse_sequence();
}