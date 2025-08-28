

#include "mux.h"

#include <math.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>


int mux_init(struct mux_config* config)
{
    int ret = 0;

    if (!config->spi_dev) {
        printf("[mux][init] ERROR: SPI device not valid!");
        return -1;
    }

    config->spi_config.frequency = 1U * 1000U * 1000U,  /* 1 MHz */
    config->spi_config.operation = SPI_OP_MODE_MASTER   /* master mode */ 
               | SPI_TRANSFER_MSB                       /* MSB first */
               | SPI_WORD_SET(8);                       /* 8-bit words */
    config->spi_config.slave     = 0;                   /* CS 0 */
    // config->spi_config.cs        = NULL,                /* manual CS, if needed */


    if(!config->gpio_dev)
    {
        printf("[mux][init] ERROR: GPIO device not valid!\n");
        return -1;
    }

    // configure nLE and CLR pins
    ret = gpio_pin_configure(config->gpio_dev, config->le_pin, GPIO_OUTPUT_ACTIVE);
    ret = gpio_pin_configure(config->gpio_dev, config->clr_pin, GPIO_OUTPUT_INACTIVE);

    if (ret < 0)
    {
        printf("[mux][init] ERROR: error configuring LE or CLR pins: %d\n", ret);
        return ret;
    }

    // // ensure nLE and CLR pins are in desired initial state (do we need this?)
    // mux_set_clr(config->gpio_dev, 0);
    // mux_set_le(config->gpio_dev, 1);

    return 0;
}

int mux_write(struct mux_config* config, uint8_t * buff_ptr, uint8_t buff_len)
{
    int ret = 0;

    if (!config->spi_dev) {
        printf("[mux][write] ERROR: SPI device not valid!\n");
        return -1;
    }

    if(!config->gpio_dev)
    {
        printf("[mux][init] ERROR: GPIO device not valid!\n");
        return -1;
    }
    
    // disable latches
    ret = mux_set_le(config, 1);

    // clear latches
    ret = mux_set_clr(config, 1);


    // setup data transfer
    struct spi_buf tx_buf = {
        .buf = buff_ptr,
        .len = buff_len,
    };

    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count   = 1,
    };

    ret = spi_write(config->spi_dev, &config->spi_config, &tx);

    if (ret) {
        printf("[mux][write] ERROR: SPI write failed: %d\n", ret);
    } else {
        printf("[mux][write] INFO: SPI write succeeded\n");
    }

    // un-clear latches
    mux_set_clr(config, 0);

    // propagate data to latches
    mux_set_le(config, 0);
    k_sleep(K_USEC(100));
    mux_set_le(config, 1);

    return 0;
}

int mux_set_clr(struct mux_config* config, int state)
{
    int ret = 0;
    ret = gpio_pin_set(config->gpio_dev, config->clr_pin, state);

    if(ret < 0)
    {
        printf("[mux][set_clr] ERROR: CLR pin set failed: %d\n", ret);
        return ret;
    }

    return 0;
}

int mux_set_le(struct mux_config* config, int state)
{
    int ret = 0;
    ret = gpio_pin_set(config->gpio_dev, config->le_pin, state);

    if(ret < 0)
    {
        printf("[mux][set_clr] ERROR: LE pin set failed: %d\n", ret);
        return ret;
    }

    return 0;
}

int mux_set_single_channel(struct mux_config *config, int channel, int state)
{
    int ret = 0;

    if(channel >= config->num_channels)
    {
        printf("[mux][set_channel] ERROR: channel number larger than max available channels\n");
        return -1;
    }

    uint8_t tx_buffer[2];
    uint64_t channel_output = BIT(channel);

    tx_buffer[0] = (uint8_t) (channel_output >> 8) & 0x00ff;
    tx_buffer[1] = (uint8_t) channel_output & 0x00ff;

    ret = mux_write(config, tx_buffer, 2);

    return ret;
}


int mux_set_channel_pair(struct mux_config *config, int channel, int state)
{
    int ret = 0;

    if(channel >= config->num_channels/2)
    {
        printf("[mux][set_channel_pair] ERROR: channel number larger than max available channels\n");
        return -1;
    }

    uint8_t tx_buffer[2];
    uint64_t channel_output = 0x03 << (channel*2);

    tx_buffer[0] = (uint8_t) (channel_output >> 8) & 0x00ff;
    tx_buffer[1] = (uint8_t) channel_output & 0x00ff;

    ret = mux_write(config, tx_buffer, 2);

    return ret;
}

int mux_test(struct mux_config *mux_test_config)
{
    int ret = 0;

    // cycle through all chanels
    for(int i=0; i < mux_test_config->num_channels; i++)
    {
        ret = mux_set_single_channel(mux_test_config, i, 1);

        if(ret<0)
        {
            printf("[mux][test] ERROR: mux set channel error occured while running test\n");
        }

        k_sleep(K_SECONDS(1));
    }

    return 0;
}
