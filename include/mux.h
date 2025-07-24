#ifndef MUX_H
#define MUX_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Default SPI and GPIO device for the multiplexer
 */
 #define MUX_SPI1_DEV   DEVICE_DT_GET(DT_NODELABEL(spi1))
 #define MUX_SPI2_DEV   DEVICE_DT_GET(DT_NODELABEL(spi2))
#define MUX_GPIO_DEV  DEVICE_DT_GET(DT_NODELABEL(gpio0))

/**
 * @brief Configuration parameters for the MUX controller
 */
struct mux_config {
    const struct device *spi_dev;   /**< SPI controller device */
    struct spi_config spi_config;
    const struct device *gpio_dev;  /**< GPIO controller device */
    uint8_t le_pin;                 /**< Latch enable pin number */
    uint8_t clr_pin;                /**< Clear pin number */
    uint8_t num_channels; // number of channels
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the multiplexer interface
 *
 * @param config Pointer to mux_config
 * @return 0 on success, negative errno on failure
 */
int mux_init(struct mux_config *config);

/**
 * @brief Write a buffer to the multiplexer over SPI
 *
 * @param config   Pointer to mux_config
 * @param buff_ptr Pointer to data buffer
 * @param buff_len Length of data buffer
 * @return 0 on success, negative errno on failure
 */
int mux_write(struct mux_config *config, uint8_t *buff_ptr, uint8_t buff_len);

/**
 * @brief Set or clear the multiplexer output
 *
 * @param config Pointer to mux_config
 * @param clear  If true, clear output; else assert output
 * @return 0 on success, negative errno on failure
 */
int mux_set_clr(struct mux_config *config, int state);

/**
 * @brief Set or clear the multiplexer output
 *
 * @param config Pointer to mux_config
 * @param clear  If true, clear output; else assert output
 * @return 0 on success, negative errno on failure
 */
 int mux_set_le(struct mux_config *config, int state);

 int mux_set_single_channel(struct mux_config *config, int channel, int state);

#ifdef __cplusplus
}
#endif

#endif /* MUX_H */