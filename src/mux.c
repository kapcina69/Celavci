#include "mux.h"
#include <zephyr/logging/log.h>


LOG_MODULE_DECLARE(main);  


struct spi_dt_spec spi_mux = SPI_DT_SPEC_GET(DT_NODELABEL(mux_stim), SPIOP, 0);


// Funkcija za slanje 16-bitnog podatka preko SPI
void set_mux(uint16_t data)
{
    LOG_INF("Slanje MUX podatka: 0x%04X", data);

    uint8_t tx_data[2] = {
        (uint8_t)((data >> 8) & 0xFF),  // MSB
        (uint8_t)(data & 0xFF)         // LSB
    };

    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data)
    };

    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    int ret = spi_write_dt(&spi_mux, &tx);
    if (ret != 0) {
        LOG_ERR("SPI write failed! (code: %d)", ret);
    } else {
        LOG_INF("Podaci uspešno poslati MUX-u");
    }
}