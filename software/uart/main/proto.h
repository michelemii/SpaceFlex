/*
 * proto.h
 * Binary protocol between the PC and the ESP32 (portable C, no ESP-IDF deps,
 * so it can also be compiled and tested on a PC).
 *
 * Every frame, both directions:
 *
 *   byte 0      0xA5            sync
 *   byte 1      0x5A            sync
 *   byte 2      type            command / response code
 *   byte 3..4   len             payload length, uint16 little endian
 *   byte 5..    payload         len bytes
 *   last 2      crc             CRC16 CCITT-FALSE (poly 0x1021, init 0xFFFF,
 *                               no reflection, no final xor) computed over
 *                               type + len (2 bytes) + payload, little endian
 *
 * All multibyte header/payload integers are little endian, EXCEPT the ADC
 * samples inside DATA frames, which are forwarded exactly as the ADS131E04
 * outputs them: big endian, two's complement, 3 bytes (24 bit) or 2 bytes
 * (16 bit at 32 kSPS).
 *
 * ---------------------------------------------------------------------------
 * PC -> ESP32
 * ---------------------------------------------------------------------------
 * CMD_PING (0x01), len 0           -> ESP32 answers RSP_INFO
 * CMD_STOP (0x03), len 0           -> ACK, then RSP_END when acquisition stops
 * CMD_READ_REGS (0x04), len 0      -> RSP_REGS (only when idle, else ACK BUSY)
 * CMD_START (0x02), len 16:
 *   off size field
 *   0   1    dr           ADS131E04 CONFIG1.DR code:
 *                           1 = 32 kSPS (16 bit)   2 = 16 kSPS   3 = 8 kSPS
 *                           4 = 4 kSPS             5 = 2 kSPS    6 = 1 kSPS
 *                           (2..6 are 24 bit; 0 = 64 kSPS is rejected)
 *   1   1    ch_mask      bit0..bit3 = CH1..CH4 enabled (at least one)
 *   2   4    gain[4]      PGA gain per channel: 1, 2, 4, 8 or 12
 *   6   4    mux[4]       input per channel: 0 normal, 1 shorted,
 *                           3 MVDD supply, 4 temperature, 5 test signal
 *   10  4    duration_ms  acquisition length, 0 = until CMD_STOP
 *   14  1    flags        bit0 force OFFSETCAL, bit1 send status word
 *                           with every sample
 *   15  1    test_cfg     CONFIG2 bits: bit2 TEST_AMP, bits1:0 TEST_FREQ
 *                           (used only if some enabled channel has mux 5)
 *   -> ESP32 answers ACK (status), then DATA frames, then END.
 *
 * ---------------------------------------------------------------------------
 * ESP32 -> PC
 * ---------------------------------------------------------------------------
 * RSP_ACK (0x81):  cmd(1) status(1)
 *   if cmd == CMD_START and status == ST_OK, 15 more bytes follow:
 *   sample_rate(4) n_samples(4, 0 = continuous) ch_mask(1)
 *   bytes_per_ch(1) flags(1) samples_per_packet(1) calibrated(1)
 *   vref_mV(2)
 * RSP_INFO (0x82): fw_version(2, major<<8|minor) adc_id(1) adc_ok(1)
 *   uart_baud(4) spi_hz(4) vref_mV(2) state(1: 0 idle, 1 busy, 2 booting)
 * RSP_REGS (0x83): 21 bytes, ADS131E04 registers 0x00..0x14
 * RSP_DATA (0x90):
 *   seq(2)           packet counter, +1 per DATA frame, wraps at 65536
 *   first_sample(4)  index of the first sample (0 = first conversion after
 *                    START). Samples inside one frame are always consecutive;
 *                    a jump between frames means samples were lost.
 *   n_samples(1)
 *   ch_mask(1)
 *   bytes_per_ch(1)  2 or 3
 *   flags(1)         bit1 = status word present in each sample
 *   then n_samples records:
 *     [status 3 bytes, big endian, if flags bit1]
 *     one value per enabled channel, ascending channel order,
 *     bytes_per_ch bytes, big endian two's complement
 * RSP_END (0x91):
 *   reason(1)        0 done, 1 stopped by PC, 2 DRDY timeout
 *   drdy_count(4)    conversions accounted for during the run
 *                    (= samples_sent + missed + dropped)
 *   samples_sent(4)  samples put in DATA frames
 *   missed(4)        conversions not delivered because the ESP32 read them
 *                    too late (skipped, or read overlapping the next update)
 *   dropped(4)       samples read but discarded (TX buffers full)
 *   status_err(4)    frames whose status word did not start with 0b1100
 *   packets(4)       DATA frames sent (compare with what the PC received)
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_SYNC0            0xA5
#define PROTO_SYNC1            0x5A
#define PROTO_HDR_LEN          5
#define PROTO_CRC_LEN          2
#define PROTO_OVERHEAD         (PROTO_HDR_LEN + PROTO_CRC_LEN)
#define PROTO_RX_MAX_PAYLOAD   64

/* PC -> ESP32 */
#define CMD_PING               0x01
#define CMD_START              0x02
#define CMD_STOP               0x03
#define CMD_READ_REGS          0x04

/* ESP32 -> PC */
#define RSP_ACK                0x81
#define RSP_INFO               0x82
#define RSP_REGS               0x83
#define RSP_DATA               0x90
#define RSP_END                0x91

/* ACK status codes */
#define ST_OK                  0
#define ST_BAD_FRAME           1
#define ST_BAD_LEN             2
#define ST_BAD_PARAM           3
#define ST_BUSY                4
#define ST_BANDWIDTH           5
#define ST_ADC_FAIL            6
#define ST_NOT_RUNNING         7
#define ST_UNKNOWN_CMD         8

/* END reasons */
#define END_DONE               0
#define END_STOPPED            1
#define END_DRDY_TIMEOUT       2

/* START payload */
#define START_PAYLOAD_LEN      16
#define START_FLAG_OFFSETCAL   0x01
#define START_FLAG_STATUS      0x02
#define START_FLAGS_MASK       (START_FLAG_OFFSETCAL | START_FLAG_STATUS)

/* Payload lengths of the ESP32 responses */
#define ACK_LEN_SHORT          2
#define ACK_LEN_START          17
#define INFO_LEN               15
#define REGS_LEN               21
#define END_LEN                25

/* DATA payload */
#define DATA_HDR_LEN           10
#define DATA_FLAG_STATUS       0x02
#define DATA_MAX_SAMPLE_BYTES  480   /* sample bytes in one DATA frame */
#define DATA_TARGET_PKT_PER_S  100   /* about 10 ms of data per frame */
#define DATA_FRAME_MAX         (PROTO_OVERHEAD + DATA_HDR_LEN + DATA_MAX_SAMPLE_BYTES)

#define ADS_MAX_CH             4
#define ADS_STATUS_BYTES       3

/* Parsed START request */
typedef struct {
    uint8_t  dr;
    uint8_t  ch_mask;
    uint8_t  gain[ADS_MAX_CH];
    uint8_t  mux[ADS_MAX_CH];
    uint32_t duration_ms;
    uint8_t  flags;
    uint8_t  test_cfg;
} start_cfg_t;

/* Acquisition plan derived from a START request */
typedef struct {
    uint32_t sample_rate;       /* Hz (nominal, internal oscillator 2.048 MHz) */
    uint32_t n_samples;         /* 0 = continuous */
    uint8_t  n_ch;              /* enabled channels */
    uint8_t  bytes_per_ch;      /* 2 or 3 */
    uint8_t  bytes_per_sample;  /* bytes of one record inside a DATA frame */
    uint8_t  spp;               /* samples per DATA frame */
    uint8_t  frame_len;         /* bytes to clock out of the ADC per conversion */
} acq_plan_t;

/* ---------------- little endian helpers ---------------- */
static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---------------- CRC and frames ---------------- */
uint16_t proto_crc16(uint16_t crc, const uint8_t *data, size_t len);

/* The payload is already at frame + PROTO_HDR_LEN. Writes the header and the
 * CRC, returns the total frame length. */
size_t proto_finalize(uint8_t *frame, uint8_t type, uint16_t payload_len);

/* Copies payload into out (out must hold len + PROTO_OVERHEAD bytes). */
size_t proto_build(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t len);

/* ---------------- receive parser ---------------- */
typedef enum {
    PP_NONE = 0,     /* byte consumed, no complete frame yet */
    PP_FRAME,        /* valid frame in parser->type / payload / len */
    PP_BAD_CRC,      /* complete frame with wrong CRC (parser->type valid) */
    PP_BAD_LEN       /* declared length too large, frame discarded */
} pp_result_t;

typedef struct {
    uint8_t  state;
    uint8_t  type;
    uint16_t len;
    uint16_t idx;
    uint16_t crc;
    uint8_t  payload[PROTO_RX_MAX_PAYLOAD];
} proto_parser_t;

void        proto_parser_reset(proto_parser_t *p);
bool        proto_parser_in_frame(const proto_parser_t *p);
pp_result_t proto_parser_feed(proto_parser_t *p, uint8_t b);

/* ---------------- ADS131E04 configuration helpers ---------------- */
uint8_t  cfg_gain_code(uint8_t gain);   /* CHnSET.GAIN code, 0xFF if invalid */
bool     cfg_mux_valid(uint8_t mux);
uint32_t cfg_dr_rate(uint8_t dr);       /* Hz, 0 if invalid */
uint8_t  cfg_dr_bytes(uint8_t dr);      /* 2 or 3, 0 if invalid */

/* Decode the START payload. Returns ST_OK, ST_BAD_LEN or ST_BAD_PARAM. */
int proto_parse_start(const uint8_t *payload, uint16_t len, start_cfg_t *cfg);

/* Validate the request and build the acquisition plan, including the UART
 * bandwidth check. Returns ST_OK, ST_BAD_PARAM or ST_BANDWIDTH. */
int proto_plan(const start_cfg_t *cfg, uint32_t uart_baud, uint32_t max_load_pct, acq_plan_t *plan);

#ifdef __cplusplus
}
#endif
