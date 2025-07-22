#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>



// SPI definicija
#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
extern struct spi_dt_spec spi_mux;

/**
 * @brief Send a 16-bit data to the MUX via SPI.
 *
 * This function sends a 16-bit data value to the MUX device over SPI.
 * The data is split into two bytes (MSB and LSB) before transmission.
 *
 * @param data The 16-bit data to send.
 */
void set_mux(uint16_t data);