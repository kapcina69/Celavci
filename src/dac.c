#include "dac.h"

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static volatile bool dac_thread_running = false;


int dac_init(void)
{
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return -ENODEV;
    }
    return 0;
}

void dac_set_value(uint16_t value)
{
    if (value > 0x03FF) {
        value = 0x03FF;
    }

    uint8_t buffer[3];
    buffer[0] = 0x40;
    buffer[1] = (value >> 2) & 0xFF;
    buffer[2] = (value & 0x03) << 6;

    int ret = i2c_write(i2c_dev, buffer, sizeof(buffer), DAC_ADDR);
    if (ret < 0) {
        printk("DAC slanje greska: %d\n", ret);
    } else {
        printk("DAC postavljen na: %u\n", value);
    }
}
