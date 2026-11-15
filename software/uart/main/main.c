/*
 * main.c
 * ESP32 DevKitC + ADS131E04: acquisition on request from the PC over UART0.
 *
 * Tasks:
 *   cmd_task (core 0)  parses PC frames, validates START, answers PING/errors
 *   acq_task (core 1)  owns the ADC: power up, configuration, OFFSETCAL,
 *                      DRDY driven reads, builds DATA frames
 *   tx_task  (core 0)  sends queued frames on the UART (CRC computed here)
 *
 * Data path: DRDY falling edge -> ISR -> task notification -> RDATA read of
 * the frame -> sample copied into a pooled DATA frame -> tx queue -> UART.
 * DRDY edges are also counted in hardware (PCNT). The count read before and
 * after each RDATA gives the exact index of the conversion that was read; if
 * a new conversion completed during the read, the sample is discarded. Late
 * reads are reported as missed samples, so a delivered sample is never
 * mislabeled, duplicated or corrupted, whatever the interrupt latency.
 *
 * Protocol: see proto.h.
 */
#include <stdatomic.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "ads131e04.h"
#include "board.h"
#include "proto.h"

/* ---------------- frame pool shared by acq_task and tx_task ---------------- */
#define POOL_N  48

typedef struct {
    uint8_t  type;
    uint16_t len;                      /* payload length */
    uint8_t  frame[DATA_FRAME_MAX];    /* header + payload + CRC */
} pkt_t;

static pkt_t s_pool[POOL_N];
static QueueHandle_t s_free_q;         /* uint8_t: free pool indices */
static QueueHandle_t s_tx_q;           /* uint8_t: indices ready to send */

/* ---------------- requests to the acquisition task ---------------- */
typedef enum { REQ_START = 0, REQ_READ_REGS } req_kind_t;

typedef struct {
    req_kind_t  kind;
    start_cfg_t cfg;
    acq_plan_t  plan;
} req_t;

static QueueHandle_t s_req_q;

/* ---------------- shared state ---------------- */
enum { STATE_IDLE = 0, STATE_BUSY = 1, STATE_BOOTING = 2 };

static atomic_int  s_state = STATE_BOOTING;
static atomic_bool s_stop_req = false;
static atomic_int  s_adc_id = 0;
static atomic_bool s_adc_ok = false;

/* Gain code each channel was offset calibrated with (0 = not calibrated). */
static uint8_t s_cal_gain[ADS_MAX_CH];

/* vTaskDelay that never waits less than ms milliseconds. */
static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

static inline uint8_t *pkt_payload(pkt_t *p)
{
    return &p->frame[PROTO_HDR_LEN];
}

/* ======================================================================== */
/*                               TX side                                    */
/* ======================================================================== */

static void tx_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint8_t idx;
        if (xQueueReceive(s_tx_q, &idx, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        pkt_t *p = &s_pool[idx];
        size_t n = proto_finalize(p->frame, p->type, p->len);
        uart_write_bytes(PC_UART_NUM, p->frame, n);
        xQueueSend(s_free_q, &idx, portMAX_DELAY);
    }
}

/* Ordered send (goes through the same queue as DATA frames). Used by the
 * acquisition task, so ACK -> DATA -> END keep their order on the wire. */
static void send_queued(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint8_t idx;
    xQueueReceive(s_free_q, &idx, portMAX_DELAY);
    pkt_t *p = &s_pool[idx];
    p->type = type;
    p->len = len;
    if (len) {
        memcpy(pkt_payload(p), payload, len);
    }
    xQueueSend(s_tx_q, &idx, portMAX_DELAY);
}

/* Immediate send from the command task. uart_write_bytes holds the driver TX
 * mutex for the whole call, so a frame is never split by another writer. */
static void send_direct(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[PROTO_OVERHEAD + 32];
    if (len > 32) {
        return;
    }
    size_t n = proto_build(buf, type, payload, len);
    uart_write_bytes(PC_UART_NUM, buf, n);
}

static void send_ack_direct(uint8_t cmd, uint8_t status)
{
    uint8_t p[ACK_LEN_SHORT] = { cmd, status };
    send_direct(RSP_ACK, p, sizeof p);
}

static void send_ack_queued(uint8_t cmd, uint8_t status)
{
    uint8_t p[ACK_LEN_SHORT] = { cmd, status };
    send_queued(RSP_ACK, p, sizeof p);
}

static void build_info(uint8_t *p)
{
    put_u16(&p[0], (uint16_t)((FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR));
    p[2] = (uint8_t)atomic_load(&s_adc_id);
    p[3] = atomic_load(&s_adc_ok) ? 1 : 0;
    put_u32(&p[4], PC_UART_BAUD);
    put_u32(&p[8], ADS_SPI_HZ);
    put_u16(&p[12], ADS_VREF_MV);
    p[14] = (uint8_t)atomic_load(&s_state);
}

/* ======================================================================== */
/*                            Command side                                  */
/* ======================================================================== */

static void handle_command(uint8_t type, const uint8_t *pl, uint16_t len)
{
    switch (type) {
    case CMD_PING: {
        uint8_t p[INFO_LEN];
        build_info(p);
        send_direct(RSP_INFO, p, INFO_LEN);
        break;
    }

    case CMD_STOP:
        if (len != 0) {
            send_ack_direct(CMD_STOP, ST_BAD_LEN);
        } else if (atomic_load(&s_state) == STATE_BUSY) {
            atomic_store(&s_stop_req, true);
            send_ack_direct(CMD_STOP, ST_OK);
        } else {
            send_ack_direct(CMD_STOP, ST_NOT_RUNNING);
        }
        break;

    case CMD_READ_REGS: {
        if (len != 0) {
            send_ack_direct(CMD_READ_REGS, ST_BAD_LEN);
            break;
        }
        int expected = STATE_IDLE;
        if (!atomic_compare_exchange_strong(&s_state, &expected, STATE_BUSY)) {
            send_ack_direct(CMD_READ_REGS, ST_BUSY);
            break;
        }
        req_t r = { .kind = REQ_READ_REGS };
        xQueueSend(s_req_q, &r, portMAX_DELAY);
        break;
    }

    case CMD_START: {
        req_t r = { .kind = REQ_START };
        int st = proto_parse_start(pl, len, &r.cfg);
        if (st == ST_OK) {
            st = proto_plan(&r.cfg, PC_UART_BAUD, PC_UART_MAX_LOAD_PCT, &r.plan);
        }
        if (st != ST_OK) {
            send_ack_direct(CMD_START, (uint8_t)st);
            break;
        }
        int expected = STATE_IDLE;
        if (!atomic_compare_exchange_strong(&s_state, &expected, STATE_BUSY)) {
            send_ack_direct(CMD_START, ST_BUSY);
            break;
        }
        atomic_store(&s_stop_req, false);
        xQueueSend(s_req_q, &r, portMAX_DELAY);
        break;
    }

    default:
        send_ack_direct(type, ST_UNKNOWN_CMD);
        break;
    }
}

static void cmd_task(void *arg)
{
    (void)arg;
    static proto_parser_t parser;
    uint8_t rx[64];
    TickType_t last_byte = xTaskGetTickCount();

    proto_parser_reset(&parser);
    for (;;) {
        int n = uart_read_bytes(PC_UART_NUM, rx, sizeof rx, pdMS_TO_TICKS(20));
        TickType_t now = xTaskGetTickCount();
        if (n <= 0) {
            /* Drop a half received frame after 100 ms of silence. */
            if (proto_parser_in_frame(&parser) && (now - last_byte) > pdMS_TO_TICKS(100)) {
                proto_parser_reset(&parser);
            }
            continue;
        }
        last_byte = now;
        for (int i = 0; i < n; i++) {
            pp_result_t r = proto_parser_feed(&parser, rx[i]);
            if (r == PP_FRAME) {
                handle_command(parser.type, parser.payload, parser.len);
            } else if (r == PP_BAD_CRC) {
                send_ack_direct(parser.type, ST_BAD_FRAME);
            } else if (r == PP_BAD_LEN) {
                send_ack_direct(parser.type, ST_BAD_LEN);
            }
        }
    }
}

/* ======================================================================== */
/*                          Acquisition side                                */
/* ======================================================================== */

static void adc_init(bool first_boot)
{
    uint8_t id = 0;
    esp_err_t err = ads_powerup(&id, first_boot);
    atomic_store(&s_adc_id, id);
    atomic_store(&s_adc_ok, err == ESP_OK);
    memset(s_cal_gain, 0, sizeof s_cal_gain);   /* RESET clears the calibration */
}

static void do_read_regs(void)
{
    uint8_t regs[REGS_LEN];
    ads_rregs(0x00, regs, REGS_LEN);
    send_queued(RSP_REGS, regs, REGS_LEN);
}

/* Builds DATA frames in pool buffers. */
typedef struct {
    pkt_t   *pkt;
    uint8_t  idx;
    uint8_t *wp;
    uint32_t first;
    uint8_t  n;
    uint16_t seq;
    uint32_t packets;
    uint8_t  ch_mask;
    uint8_t  bpc;
    uint8_t  bps;
    uint8_t  flags;
} writer_t;

static void writer_flush(writer_t *w)
{
    if (w->pkt == NULL) {
        return;
    }
    uint8_t *pl = pkt_payload(w->pkt);
    put_u16(&pl[0], w->seq);
    put_u32(&pl[2], w->first);
    pl[6] = w->n;
    pl[7] = w->ch_mask;
    pl[8] = w->bpc;
    pl[9] = w->flags;
    w->pkt->type = RSP_DATA;
    w->pkt->len = (uint16_t)(DATA_HDR_LEN + (uint16_t)w->n * w->bps);
    /* Never blocks: the tx queue has room for every pool buffer. */
    xQueueSend(s_tx_q, &w->idx, 0);
    w->seq++;
    w->packets++;
    w->pkt = NULL;
}

/* Returns false when no buffer is free (sample must be dropped). */
static bool writer_open(writer_t *w, uint32_t first)
{
    if (xQueueReceive(s_free_q, &w->idx, 0) != pdTRUE) {
        return false;
    }
    w->pkt = &s_pool[w->idx];
    w->wp = pkt_payload(w->pkt) + DATA_HDR_LEN;
    w->first = first;
    w->n = 0;
    return true;
}

static void run_session(const start_cfg_t *cfg, const acq_plan_t *plan)
{
    /* ---- ADC present? (retry the power up if it failed at boot) ---- */
    if (!atomic_load(&s_adc_ok)) {
        adc_init(false);
        if (!atomic_load(&s_adc_ok)) {
            send_ack_queued(CMD_START, ST_ADC_FAIL);
            return;
        }
    }

    /* ---- 1. registers (SDATAC mode, START low) ---- */
    uint8_t gain_code[ADS_MAX_CH] = { 0 };
    uint8_t chset[ADS_MAX_CH];
    uint8_t ch_off[ADS_MAX_CH];          /* byte offset of each enabled channel in the ADC frame */
    uint8_t n_ch = 0;
    bool any_test = false;

    for (int i = 0; i < ADS_MAX_CH; i++) {
        if (cfg->ch_mask & (1u << i)) {
            gain_code[i] = cfg_gain_code(cfg->gain[i]);
            chset[i] = (uint8_t)((gain_code[i] << 4) | (cfg->mux[i] & 0x07));
            if (cfg->mux[i] == 5) {
                any_test = true;
            }
            ch_off[n_ch++] = (uint8_t)(ADS_STATUS_BYTES + i * plan->bytes_per_ch);
        } else {
            chset[i] = ADS_CHSET_OFF;
        }
    }

    const uint8_t config1 = (uint8_t)(ADS_CONFIG1_BASE | (cfg->dr & 0x07));   /* CLK_EN stays 0 */
    const uint8_t config2 = (uint8_t)(ADS_CONFIG2_BASE |
                                      (any_test ? (ADS_CONFIG2_INT_TEST | (cfg->test_cfg & 0x07)) : 0));

    bool ok = ads_wreg_verify(ADS_REG_CONFIG1, config1);
    ok = ok && ads_wreg_verify(ADS_REG_CONFIG2, config2);
    ok = ok && ads_wreg_verify(ADS_REG_CONFIG3, ADS_CONFIG3_VALUE);
    for (int i = 0; i < ADS_MAX_CH && ok; i++) {
        ok = ads_wreg_verify((uint8_t)(ADS_REG_CH1SET + i), chset[i]);
    }
    if (!ok) {
        atomic_store(&s_adc_ok, false);
        send_ack_queued(CMD_START, ST_ADC_FAIL);
        return;
    }

    /* ---- 2. OFFSETCAL: after every gain change (datasheet), or on request.
     *      TI: the device must be converting (START high) when it is sent. ---- */
    bool need_cal = (cfg->flags & START_FLAG_OFFSETCAL) != 0;
    for (int i = 0; i < ADS_MAX_CH; i++) {
        if ((cfg->ch_mask & (1u << i)) && s_cal_gain[i] != gain_code[i]) {
            need_cal = true;
        }
    }
    bool calibrated = false;
    if (need_cal) {
        ads_set_start(true);
        delay_ms(10);                       /* > tSETTLE at 1 kSPS (4.5 ms) */
        ads_cmd(ADS_CMD_OFFSETCAL);
        delay_ms(ADS_OFFSETCAL_MS);         /* 153 ms per datasheet */
        ads_set_start(false);
        delay_ms(3);                        /* let the conversion in progress end */
        /* Make sure the calibration left the data rate as programmed. */
        if (ads_rreg(ADS_REG_CONFIG1) != config1 && !ads_wreg_verify(ADS_REG_CONFIG1, config1)) {
            atomic_store(&s_adc_ok, false);
            send_ack_queued(CMD_START, ST_ADC_FAIL);
            return;
        }
        for (int i = 0; i < ADS_MAX_CH; i++) {
            /* Powered down channels were not measured: forget their value. */
            s_cal_gain[i] = (cfg->ch_mask & (1u << i)) ? gain_code[i] : 0;
        }
        calibrated = true;
    }

    /* ---- 3. ACK with the plan ---- */
    const bool with_status = (cfg->flags & START_FLAG_STATUS) != 0;
    {
        uint8_t a[ACK_LEN_START];
        a[0] = CMD_START;
        a[1] = ST_OK;
        put_u32(&a[2], plan->sample_rate);
        put_u32(&a[6], plan->n_samples);
        a[10] = cfg->ch_mask;
        a[11] = plan->bytes_per_ch;
        a[12] = with_status ? START_FLAG_STATUS : 0;
        a[13] = plan->spp;
        a[14] = calibrated ? 1 : 0;
        put_u16(&a[15], ADS_VREF_MV);
        send_queued(RSP_ACK, a, sizeof a);
    }

    /* ---- 4. acquisition ---- */
    writer_t w = {
        .pkt = NULL,
        .seq = 0,
        .packets = 0,
        .ch_mask = cfg->ch_mask,
        .bpc = plan->bytes_per_ch,
        .bps = plan->bytes_per_sample,
        .flags = with_status ? DATA_FLAG_STATUS : 0,
    };
    const uint32_t n_target = plan->n_samples;           /* 0 = continuous */
    const uint8_t frame_len = plan->frame_len;
    const uint8_t bpc = plan->bytes_per_ch;
    const uint8_t spp = plan->spp;
    const TickType_t drdy_timeout = pdMS_TO_TICKS(100) + 1;

    uint8_t rx[ADS_MAX_FRAME_BYTES];
    uint32_t done = 0;           /* conversions 0..done-1 accounted for (sent, missed or dropped) */
    uint32_t sent = 0, missed = 0, dropped = 0, status_err = 0;
    uint8_t reason = END_DONE;

    /* The device stays in SDATAC mode: data is read with RDATA. */
    ads_drdy_count_reset();               /* START low: no edges now */
    (void)ulTaskNotifyTake(pdTRUE, 0);    /* clear stale notifications */
    ads_bus_acquire();
    ads_drdy_irq_enable(true);
    ads_set_start(true);

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, drdy_timeout) == 0) {
            reason = END_DRDY_TIMEOUT;
            break;
        }

        /* The hardware edge counter (not the notification count) says which
         * conversion is the latest one: index c0 - 1. */
        const uint32_t c0 = ads_drdy_count();
        if (c0 == done) {
            continue;    /* this edge was already consumed by the previous read */
        }

        ads_read_data(rx, frame_len);

        const uint32_t c1 = ads_drdy_count();

        const uint32_t idx = c0 - 1;              /* index of the conversion just read */
        if (n_target != 0 && idx >= n_target) {
            missed += n_target - done;            /* the rest of the window was lost */
            done = n_target;
            break;
        }
        missed += idx - done;                     /* conversions never read */
        done = c0;

        if (c1 != c0) {
            /* A new conversion completed during the read: the data is intact
             * (RDATA) but it may belong to c0 - 1 or c0. Discard it. */
            missed++;
        } else {
            if ((rx[0] & ADS_STATUS_SYNC_MASK) != ADS_STATUS_SYNC) {
                status_err++;
            }

            /* Samples in one DATA frame must be consecutive. */
            if (w.pkt != NULL && idx != w.first + w.n) {
                writer_flush(&w);
            }
            if (w.pkt == NULL && !writer_open(&w, idx)) {
                dropped++;    /* all TX buffers busy: UART slower than the data */
            } else {
                uint8_t *wp = w.wp;
                if (with_status) {
                    wp[0] = rx[0];
                    wp[1] = rx[1];
                    wp[2] = rx[2];
                    wp += ADS_STATUS_BYTES;
                }
                for (uint8_t c = 0; c < n_ch; c++) {
                    const uint8_t *src = &rx[ch_off[c]];
                    wp[0] = src[0];
                    wp[1] = src[1];
                    if (bpc == 3) {
                        wp[2] = src[2];
                    }
                    wp += bpc;
                }
                w.wp = wp;
                w.n++;
                sent++;
                if (w.n >= spp) {
                    writer_flush(&w);
                }
            }
        }

        if (n_target != 0 && done >= n_target) {
            break;
        }
        if (atomic_load(&s_stop_req)) {
            reason = END_STOPPED;
            break;
        }
    }

    /* ---- 5. stop ---- */
    ads_set_start(false);
    ads_drdy_irq_enable(false);
    ads_bus_release();
    /* The conversion in progress completes after START goes low. */
    delay_ms(2000u / plan->sample_rate + 2u);
    (void)ulTaskNotifyTake(pdTRUE, 0);

    writer_flush(&w);

    if (reason == END_DRDY_TIMEOUT) {
        /* No conversions: power or wiring problem, re-initialize next time. */
        atomic_store(&s_adc_ok, false);
    }

    uint8_t e[END_LEN];
    e[0] = reason;
    put_u32(&e[1], done);
    put_u32(&e[5], sent);
    put_u32(&e[9], missed);
    put_u32(&e[13], dropped);
    put_u32(&e[17], status_err);
    put_u32(&e[21], w.packets);
    send_queued(RSP_END, e, END_LEN);
}

static void acq_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(ads_bus_init());
    ESP_ERROR_CHECK(ads_drdy_init(xTaskGetCurrentTaskHandle()));

    adc_init(true);
    atomic_store(&s_state, STATE_IDLE);

    /* Unsolicited INFO: tells the PC the board is ready (and if the ADC answered). */
    uint8_t info[INFO_LEN];
    build_info(info);
    send_queued(RSP_INFO, info, INFO_LEN);

    for (;;) {
        req_t r;
        if (xQueueReceive(s_req_q, &r, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (r.kind == REQ_READ_REGS) {
            do_read_regs();
        } else {
            run_session(&r.cfg, &r.plan);
        }
        atomic_store(&s_state, STATE_IDLE);
    }
}

/* ======================================================================== */

void app_main(void)
{
    /* UART0 carries the binary protocol: no log output on it. */
    esp_log_level_set("*", ESP_LOG_NONE);

    const uart_config_t uc = {
        .baud_rate = PC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PC_UART_NUM, 2048, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PC_UART_NUM, &uc));
    ESP_ERROR_CHECK(uart_set_pin(PC_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_free_q = xQueueCreate(POOL_N, sizeof(uint8_t));
    s_tx_q = xQueueCreate(POOL_N, sizeof(uint8_t));
    s_req_q = xQueueCreate(2, sizeof(req_t));
    configASSERT(s_free_q && s_tx_q && s_req_q);
    for (uint8_t i = 0; i < POOL_N; i++) {
        xQueueSend(s_free_q, &i, 0);
    }

    xTaskCreatePinnedToCore(tx_task, "tx", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(cmd_task, "cmd", 4096, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(acq_task, "acq", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}
