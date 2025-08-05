#include "impulse.h"
#include "ble_nus.h"
#include "mux.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

uint8_t number_of_pulses = 0;
uint32_t frequency_of_impulses = 1000; // Default frequency

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


uint32_t hz_to_us(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return 0;
    }
    return 1000000UL / frequency_hz;
}



void pulse_timer_handler(struct k_timer *timer)
{
    static const uint8_t pulse_pattern[] = {
        0x01, 0x02, 0x04, 0x08,
        0x10, 0x20, 0x40, 0x80
    };

    switch (pulse_state) {
        case PULSE_ANODE_ON:
            // Set anode high, cathode low
            gpio_pin_set_dt(&pulse_cathode, 0);
            gpio_pin_set_dt(&pulse_anode, 1); 
            pulse_state = PULSE_CATHODE_ON;
            k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
            break;

        case PULSE_CATHODE_ON:
            // Set cathode high, anode low
            gpio_pin_set_dt(&pulse_anode, 0); 
            gpio_pin_set_dt(&pulse_cathode, 1); 
            pulse_state = PULSE_PAUSE;
            k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
            break;

        case PULSE_PAUSE:
            // Turn off both anode and cathode
            gpio_pin_set_dt(&pulse_anode, 0);
            gpio_pin_set_dt(&pulse_cathode, 0);

            // Update MUX with the next pattern
            if (number_of_pulses < ARRAY_SIZE(pulse_pattern)) {
                uint8_t pattern = pulse_pattern[number_of_pulses];
                tx_buffer_1[0] = tx_buffer_1[1] = pattern;
                mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1));
            }

            number_of_pulses++;

            if (number_of_pulses >= ARRAY_SIZE(pulse_pattern)) {
                number_of_pulses = 0;
                pulse_state = PULSE_IDLE;
                k_timer_start(&pulse_timer, K_USEC(hz_to_us(frequency)), K_NO_WAIT);
            } else {
                pulse_state = PULSE_ANODE_ON;
                k_timer_start(&pulse_timer, K_USEC(hz_to_us(frequency_of_impulses)), K_NO_WAIT);
            }
            break;

        case PULSE_IDLE:
        default:
            gpio_pin_set_dt(&pulse_anode, 0);
            gpio_pin_set_dt(&pulse_cathode, 0);
            pulse_state = PULSE_ANODE_ON;
            k_timer_start(&pulse_timer, K_USEC(hz_to_us(frequency_of_impulses)), K_NO_WAIT);
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
        tx_buffer_1[0] = 0x01;
        tx_buffer_1[1] = 0x01;
        mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1)); // Send data to MUX
        number_of_pulses++; 
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