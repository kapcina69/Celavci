#include "impulse.h"
#include "ble_nus.h"
#include "mux.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define PULSE_CATHODE_NODE DT_ALIAS(pulse_cathode)
#define PULSE_ANODE_NODE   DT_ALIAS(pulse_anode)
#define DC_DC_EN_NODE      DT_ALIAS(dc_dc_en)

uint8_t number_of_pulses = 0;
uint8_t frequency_of_impulses = 20; // Default frequency

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


uint32_t hz_to_ms(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return 0; // or error: division by zero
    }
    return 1000 / frequency_hz;
}
#define PAUSE_BETWEEN_SEQUENCES_MS 200 // Pauza između sekvenci (možeš menjati)

void pulse_timer_handler(struct k_timer *timer)
{
    switch (pulse_state) {
        case PULSE_ANODE_ON:
            gpio_pin_set_dt(&pulse_anode, 0);
            gpio_pin_set_dt(&pulse_cathode, 1);
            pulse_state = PULSE_CATHODE_ON;
            k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
            break;
            
        case PULSE_CATHODE_ON:
            gpio_pin_set_dt(&pulse_cathode, 0);
            pulse_state = PULSE_PAUSE;
            k_timer_start(&pulse_timer, K_MSEC(hz_to_ms(frequency_of_impulses)), K_NO_WAIT);
            break;
            
        case PULSE_PAUSE:
            switch(number_of_pulses){
                case 0:
                    tx_buffer_1[0] = 0x01; tx_buffer_1[1] = 0x01; break;
                case 1:
                    tx_buffer_1[0] = 0x02; tx_buffer_1[1] = 0x02; break;
                case 2:
                    tx_buffer_1[0] = 0x04; tx_buffer_1[1] = 0x04; break;
                case 3:
                    tx_buffer_1[0] = 0x08; tx_buffer_1[1] = 0x08; break;
                case 4:
                    tx_buffer_1[0] = 0x10; tx_buffer_1[1] = 0x10; break;
                case 5:
                    tx_buffer_1[0] = 0x20; tx_buffer_1[1] = 0x20; break;
                case 6:
                    tx_buffer_1[0] = 0x40; tx_buffer_1[1] = 0x40; break;
                case 7:
                    tx_buffer_1[0] = 0x80; tx_buffer_1[1] = 0x80; break;
            }

            mux_write(&stim_mux_config, tx_buffer_1, sizeof(tx_buffer_1)); // Send to MUX
            
            if (number_of_pulses == 7) {
                number_of_pulses = 0;
                pulse_state = PULSE_IDLE;
                
                // Pauza pre sledeće serije impulsa
                k_timer_start(&pulse_timer, K_MSEC(hz_to_ms(frequency)), K_NO_WAIT);
            } else {
                number_of_pulses++;
                gpio_pin_set_dt(&pulse_anode, 1);
                pulse_state = PULSE_ANODE_ON;
                k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
            }
            break;
            
        case PULSE_IDLE:
        default:
            // Nakon pauze kreće nova sekvenca
            gpio_pin_set_dt(&pulse_anode, 1);
            pulse_state = PULSE_ANODE_ON;
            k_timer_start(&pulse_timer, K_USEC(STIMULATION_PULSE_WIDTH_US * pulse_width), K_NO_WAIT);
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