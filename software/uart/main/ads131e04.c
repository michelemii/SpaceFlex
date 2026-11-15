/*
 * ads131e04.c
 * Low level driver for the TI ADS131E04, ESP-IDF v5.x / v6.x.
 *
 * Datasheet facts used here (ADS131E0x, SBAS561C):
 *  1. SPI mode 1 (CPOL 0, CPHA 1): DOUT changes on SCLK rising edge,
 *    DIN latched on SCLK falling edge. tSCLK >= 50 ns.
 *  2. tCLK = 1 / 2.048 MHz = 488 ns (internal oscillator, 2.5 % tolerance over temperature).
 *  3. Multibyte commands: 4 tCLK (tSDECODE) between bytes, CS low for the
 *    whole command, 4 tCLK (tSCCS) after the last SCLK before CS high,
 *    CS high for at least 2 tCLK (tCSH).
 *  4. Power up: wait for supplies and VCAP1, RESET low >= 1 tCLK, then
 *    18 tCLK before talking to the device. The device wakes up in RDATAC
 *    mode, where RREG is ignored: SDATAC first.
 *  5. Internal reference: set PDB_REFBUF, wait about 150 ms.
 *  6. RDATA (SDATAC mode, device converting): no SCLK rate restriction, no
 *    wait before the data bytes, and the read may overlap the next DRDY
 *    without data corruption. That is why this driver uses RDATA and not
 *    RDATAC: in RDATAC a late read near the 4 tCLK update window returns
 *    corrupted data.
 */
#include "ads131e04.h"
#include "board.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 4 tCLK = 1.95 us nominal, 2.0 us with the oscillator 2.5 % slow. */
#define T_4CLK_US   3
/* tCSH: 2 tCLK = 1 us. */
#define T_CSH_US    2

static spi_device_handle_t s_dev;
static pcnt_unit_handle_t  s_pcnt;

/* RDATA opcode followed by zeros on DIN (in DRAM, not flash). */
static uint8_t s_rdata_tx[1 + ADS_MAX_FRAME_BYTES] = { ADS_CMD_RDATA };

static inline void cs_low(void)  { gpio_set_level(PIN_ADS_CS, 0); }
static inline void cs_high(void) { gpio_set_level(PIN_ADS_CS, 1); }

/* vTaskDelay that never waits less than ms milliseconds. */
static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

static uint8_t xfer_byte(uint8_t tx)
{
    spi_transaction_t t = { 0 };
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = tx;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t));
    return t.rx_data[0];
}

esp_err_t ads_bus_init(void)
{
    esp_err_t err;

    /* Set the output levels before enabling the drivers: no glitches on
     * CS / START / RESET. */
    gpio_set_level(PIN_ADS_CS, 1);
    gpio_set_level(PIN_ADS_START, 0);
    gpio_set_level(PIN_ADS_RESET, 1);

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_ADS_CS) | (1ULL << PIN_ADS_START) | (1ULL << PIN_ADS_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&out);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(PIN_ADS_CS, 1);
    gpio_set_level(PIN_ADS_START, 0);
    gpio_set_level(PIN_ADS_RESET, 1);

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_ADS_DIN,
        .miso_io_num = PIN_ADS_DOUT,
        .sclk_io_num = PIN_ADS_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    /* No DMA: transfers are at most 16 bytes, the CPU path has less latency. */
    err = spi_bus_initialize(ADS_SPI_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t dev = {
        .mode = 1,                       /* CPOL 0, CPHA 1 */
        .clock_speed_hz = ADS_SPI_HZ,
        .spics_io_num = -1,              /* CS is driven by hand */
        .queue_size = 1,
    };
    return spi_bus_add_device(ADS_SPI_HOST, &dev, &s_dev);
}

void ads_bus_acquire(void)
{
    ESP_ERROR_CHECK(spi_device_acquire_bus(s_dev, portMAX_DELAY));
}

void ads_bus_release(void)
{
    spi_device_release_bus(s_dev);
}

void ads_set_start(bool high)
{
    gpio_set_level(PIN_ADS_START, high ? 1 : 0);
}

void ads_cmd(uint8_t opcode)
{
    cs_low();
    xfer_byte(opcode);
    esp_rom_delay_us(T_4CLK_US);   /* tSCCS and decode time */
    cs_high();
    esp_rom_delay_us(T_CSH_US);
}

void ads_rregs(uint8_t addr, uint8_t *dst, uint8_t count)
{
    if (count == 0) {
        return;
    }
    cs_low();
    xfer_byte((uint8_t)(ADS_CMD_RREG | (addr & 0x1F)));
    esp_rom_delay_us(T_4CLK_US);
    xfer_byte((uint8_t)((count - 1) & 0x1F));
    esp_rom_delay_us(T_4CLK_US);
    for (uint8_t i = 0; i < count; i++) {
        dst[i] = xfer_byte(0x00);
        esp_rom_delay_us(T_4CLK_US);
    }
    cs_high();
    esp_rom_delay_us(T_CSH_US);
}

uint8_t ads_rreg(uint8_t addr)
{
    uint8_t v = 0;
    ads_rregs(addr, &v, 1);
    return v;
}

void ads_wreg(uint8_t addr, uint8_t value)
{
    cs_low();
    xfer_byte((uint8_t)(ADS_CMD_WREG | (addr & 0x1F)));
    esp_rom_delay_us(T_4CLK_US);
    xfer_byte(0x00);               /* one register */
    esp_rom_delay_us(T_4CLK_US);
    xfer_byte(value);
    esp_rom_delay_us(T_4CLK_US);
    cs_high();
    esp_rom_delay_us(T_CSH_US);
}

bool ads_wreg_verify(uint8_t addr, uint8_t value)
{
    ads_wreg(addr, value);
    return ads_rreg(addr) == value;
}

esp_err_t ads_powerup(uint8_t *id_out, bool first_boot)
{
    ads_set_start(false);
    cs_high();
    gpio_set_level(PIN_ADS_RESET, 1);

    if (first_boot) {
        /* Supplies, VCAP1 (22 uF) and power on reset. */
        delay_ms(ADS_POWERUP_WAIT_MS);
    }

    /* RESET pulse: low >= 1 tCLK, then >= 18 tCLK (8.8 us) of initialization. */
    gpio_set_level(PIN_ADS_RESET, 0);
    esp_rom_delay_us(10);
    gpio_set_level(PIN_ADS_RESET, 1);
    esp_rom_delay_us(100);

    /* The device wakes up in RDATAC mode: stop it, otherwise RREG is ignored.
     * The firmware then stays in SDATAC mode for good (data read by RDATA). */
    ads_cmd(ADS_CMD_SDATAC);
    esp_rom_delay_us(T_4CLK_US);

    uint8_t id = ads_rreg(ADS_REG_ID);
    if (id_out) {
        *id_out = id;
    }
    if ((id & ADS_ID_FAMILY_MASK) != ADS_ID_FAMILY) {
        return ESP_ERR_NOT_FOUND;
    }

    /* Internal reference buffer on, 2.4 V. */
    if (!ads_wreg_verify(ADS_REG_CONFIG3, ADS_CONFIG3_VALUE)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    delay_ms(ADS_REF_SETTLE_MS);

    /* Known idle state: 1 kSPS, no test signal, fault comparators default,
     * channels powered down until a START request arrives. */
    bool ok = ads_wreg_verify(ADS_REG_CONFIG1, ADS_CONFIG1_BASE | 0x06);
    ok = ok && ads_wreg_verify(ADS_REG_CONFIG2, ADS_CONFIG2_BASE);
    ok = ok && ads_wreg_verify(ADS_REG_FAULT, 0x00);
    for (int ch = 0; ch < 4 && ok; ch++) {
        ok = ads_wreg_verify((uint8_t)(ADS_REG_CH1SET + ch), ADS_CHSET_OFF);
    }
    return ok ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

void ads_read_data(uint8_t *dst, size_t len)
{
    uint8_t rx[1 + ADS_MAX_FRAME_BYTES];
    if (len > ADS_MAX_FRAME_BYTES) {
        len = ADS_MAX_FRAME_BYTES;
    }
    spi_transaction_t t = { 0 };
    t.length = (len + 1) * 8;
    t.tx_buffer = s_rdata_tx;      /* 0x12 then zeros: nothing else is decoded */
    t.rx_buffer = rx;
    cs_low();
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t));
    cs_high();
    memcpy(dst, &rx[1], len);      /* rx[0] was clocked during the opcode */
}

/* ---------------- DRDY: PCNT edge counter + wake up interrupt ---------------- */

static void IRAM_ATTR drdy_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)arg, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t drdy_counter_init(void)
{
    pcnt_unit_config_t ucfg = {
        .low_limit = -32768,
        .high_limit = 32767,
        .flags.accum_count = 1,          /* 32 bit count across 16 bit overflows */
    };
    esp_err_t err = pcnt_new_unit(&ucfg, &s_pcnt);
    if (err != ESP_OK) {
        return err;
    }
    /* DRDY is a clean push pull signal; reject anything shorter than 200 ns. */
    pcnt_glitch_filter_config_t filt = { .max_glitch_ns = 200 };
    err = pcnt_unit_set_glitch_filter(s_pcnt, &filt);
    if (err != ESP_OK) {
        return err;
    }
    pcnt_chan_config_t ccfg = {
        .edge_gpio_num = PIN_ADS_DRDY,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t chan;
    err = pcnt_new_channel(s_pcnt, &ccfg, &chan);
    if (err != ESP_OK) {
        return err;
    }
    /* count +1 on the falling edge (data ready), ignore the rising edge */
    err = pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_add_watch_point(s_pcnt, ucfg.high_limit);   /* needed by accum_count */
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_enable(s_pcnt);
    if (err != ESP_OK) {
        return err;
    }
    err = pcnt_unit_clear_count(s_pcnt);
    if (err != ESP_OK) {
        return err;
    }
    return pcnt_unit_start(s_pcnt);
}

esp_err_t ads_drdy_init(void *task_to_notify)
{
    /* PCNT first: on some IDF versions creating the channel reconfigures the
     * pad, so the GPIO interrupt is configured afterwards. */
    esp_err_t err = drdy_counter_init();
    if (err != ESP_OK) {
        return err;
    }

    /* DRDY: push pull output of the ADC. The pull up only keeps the line
     * defined while the ADC is unpowered. */
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_ADS_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    err = gpio_config(&in);
    if (err != ESP_OK) {
        return err;
    }
    gpio_intr_disable(PIN_ADS_DRDY);   /* enabled only while acquiring */

    /* Installed from the acquisition task, so the interrupt is allocated on
     * the same core the task is pinned to. */
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return gpio_isr_handler_add(PIN_ADS_DRDY, drdy_isr, task_to_notify);
}

void ads_drdy_irq_enable(bool enable)
{
    if (enable) {
        gpio_intr_enable(PIN_ADS_DRDY);
    } else {
        gpio_intr_disable(PIN_ADS_DRDY);
    }
}

void ads_drdy_count_reset(void)
{
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt));
}

uint32_t ads_drdy_count(void)
{
    int v = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_pcnt, &v));
    return (uint32_t)v;
}
