#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include "ble_nus.h"

static const struct i2c_dt_spec bq = I2C_DT_SPEC_GET(DT_NODELABEL(fuelgauge));

#define BQ_REG_TEMP            0x06
#define BQ_REG_VOLTAGE         0x08
#define BQ_REG_AVG_CURRENT     0x0C
#define BQ_REG_STATE_OF_CHARGE 0x2C

#define FG_REPORT_PERIOD_SEC   10

static inline void bq_tbuf_delay(void) { k_busy_wait(70); }

static int bq_read_u16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    int ret = i2c_write_read_dt(&bq, &reg, 1, buf, 2);  // <-- robustnije
    if (ret) return ret;
    *out = ((uint16_t)buf[1] << 8) | buf[0];            
    return 0;
}

static int bq_read_soc_pct(uint8_t *pct_out)
{
    uint8_t reg = BQ_REG_STATE_OF_CHARGE;
    uint8_t one = 0;
    int ret = i2c_write_read_dt(&bq, &reg, 1, &one, 1);
    if (ret) return ret;
    if (one > 100) one = 100;
    *pct_out = one;
    return 0;
}

static int bq_read_soc_raw_bytes(uint8_t *lsb, uint8_t *msb)
{
    uint8_t reg = BQ_REG_STATE_OF_CHARGE;
    uint8_t buf[2] = {0};
    int ret = i2c_write_read_dt(&bq, &reg, 1, buf, 2);
    if (ret) return ret;
    *lsb = buf[0];
    *msb = buf[1];
    return 0;
}

void bq27220_read_basic(void)
{
    if (!device_is_ready(bq.bus)) {
        send_response("FUEL: not ready (bus)\r\n");
        return;
    }

    uint16_t mv = 0, t_01K = 0;
    int16_t i_ma = 0;
    uint8_t soc_pct = 0;

    int r1 = bq_read_u16(BQ_REG_VOLTAGE, &mv);
    bq_tbuf_delay();

    /* ČITAMO 1 BAJT kao procenat */
    int r2 = bq_read_soc_pct(&soc_pct);
    bq_tbuf_delay();

    int r3 = bq_read_u16(BQ_REG_TEMP, &t_01K);
    bq_tbuf_delay();
    int r4 = bq_read_u16(BQ_REG_AVG_CURRENT, &i_ma);

    if (r1 || r2 || r3 || r4) {
        char e[64];
        snprintk(e, sizeof(e), "FUEL: err v=%d soc=%d t=%d i=%d\r\n", r1, r2, r3, r4);
        send_response(e);
        return;
    }

    /* (opciono) heks-debug da vidimo šta dolazi u oba bajta 0x1C */
    uint8_t lsb=0, msb=0;
    if (bq_read_soc_raw_bytes(&lsb, &msb) == 0) {
        char dbg[48];
        snprintk(dbg, sizeof(dbg), "FUEL: SOC bytes LSB=0x%02X MSB=0x%02X\r\n", lsb, msb);
        send_response(dbg);
    }

    int32_t t_x10c = (int32_t)t_01K - 2731; /* 0.1°C */

    char out[96];
    snprintk(out, sizeof(out),
             "FUEL: %u mV, %u %% SOC, %d.%d C, %d mA\r\n",
             mv, soc_pct, (int)(t_x10c/10), (int)abs(t_x10c%10), (int)i_ma);
    send_response(out);
}

int fuel_gauge_get_soc(uint16_t *soc)
{
    uint16_t raw;
    int ret = bq_read_u16(BQ_REG_STATE_OF_CHARGE, &raw);
    if (ret) return ret;

    *soc = raw;   // već u procentima (0–100)
    return 0;
}

/* ===== NOVO: periodicni k_work koji se startuje iz init-a ===== */
static struct k_work_delayable fg_work;
static bool fg_work_started;

static void fg_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    /* Pozovi postojeću funkciju koja šalje podatke */
    bq27220_read_basic();

    /* Samorescheduluj za sledeći ciklus */
    k_work_schedule(&fg_work, K_SECONDS(FG_REPORT_PERIOD_SEC));
}

void bq27220_init(void)
{
    if (!device_is_ready(bq.bus)) {
        send_response("FUEL: I2C bus not ready\r\n");
        return;
    }

    /* Jednokratna provera čitanja pri init-u (opciono) */
    uint16_t mv = 0;
    int ret = bq_read_u16(BQ_REG_VOLTAGE, &mv);
    if (ret == 0) {
        send_response("FUEL: raw init OK\r\n");
    } else {
        char msg[48];
        snprintk(msg, sizeof(msg), "FUEL: raw init E=%d\r\n", ret);
        send_response(msg);
    }

    /* Startuj periodično slanje iz init-a (samo jednom) */
    if (!fg_work_started) {
        k_work_init_delayable(&fg_work, fg_work_handler);
        /* Prvi put odmah, sledeći put na FG_REPORT_PERIOD_SEC */
        k_work_schedule(&fg_work, K_NO_WAIT);
        fg_work_started = true;
    }
}
