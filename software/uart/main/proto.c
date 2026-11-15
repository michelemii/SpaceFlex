/*
 * proto.c
 * Portable implementation of the PC <-> ESP32 protocol (see proto.h).
 */
#include "proto.h"
#include <string.h>

/* ---------------- CRC16 CCITT-FALSE ----------------
 * poly 0x1021, init 0xFFFF, no reflection, no final xor.
 * Check value for "123456789" is 0x29B1.
 * Byte wise update without a table (fast enough at the packet rates used). */
uint16_t proto_crc16(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t x = (uint8_t)((crc >> 8) ^ data[i]);
        x ^= (uint8_t)(x >> 4);
        crc = (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ (uint16_t)x);
    }
    return crc;
}

size_t proto_finalize(uint8_t *frame, uint8_t type, uint16_t payload_len)
{
    frame[0] = PROTO_SYNC0;
    frame[1] = PROTO_SYNC1;
    frame[2] = type;
    put_u16(&frame[3], payload_len);
    uint16_t crc = proto_crc16(0xFFFF, &frame[2], (size_t)3 + payload_len);
    put_u16(&frame[PROTO_HDR_LEN + payload_len], crc);
    return (size_t)PROTO_OVERHEAD + payload_len;
}

size_t proto_build(uint8_t *out, uint8_t type, const uint8_t *payload, uint16_t len)
{
    if (len && payload) {
        memmove(&out[PROTO_HDR_LEN], payload, len);
    }
    return proto_finalize(out, type, len);
}

/* ---------------- receive parser ---------------- */
enum {
    PS_SYNC0 = 0,
    PS_SYNC1,
    PS_TYPE,
    PS_LEN0,
    PS_LEN1,
    PS_PAYLOAD,
    PS_CRC0,
    PS_CRC1
};

void proto_parser_reset(proto_parser_t *p)
{
    p->state = PS_SYNC0;
    p->type = 0;
    p->len = 0;
    p->idx = 0;
    p->crc = 0;
}

bool proto_parser_in_frame(const proto_parser_t *p)
{
    return p->state != PS_SYNC0;
}

pp_result_t proto_parser_feed(proto_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case PS_SYNC0:
        if (b == PROTO_SYNC0) {
            p->state = PS_SYNC1;
        }
        break;

    case PS_SYNC1:
        if (b == PROTO_SYNC1) {
            p->state = PS_TYPE;
        } else if (b != PROTO_SYNC0) {
            p->state = PS_SYNC0;
        }
        break;

    case PS_TYPE:
        p->type = b;
        p->state = PS_LEN0;
        break;

    case PS_LEN0:
        p->len = b;
        p->state = PS_LEN1;
        break;

    case PS_LEN1:
        p->len = (uint16_t)(p->len | ((uint16_t)b << 8));
        if (p->len > PROTO_RX_MAX_PAYLOAD) {
            p->state = PS_SYNC0;
            return PP_BAD_LEN;
        }
        p->idx = 0;
        p->state = (p->len != 0) ? PS_PAYLOAD : PS_CRC0;
        break;

    case PS_PAYLOAD:
        p->payload[p->idx++] = b;
        if (p->idx >= p->len) {
            p->state = PS_CRC0;
        }
        break;

    case PS_CRC0:
        p->crc = b;
        p->state = PS_CRC1;
        break;

    case PS_CRC1: {
        p->crc = (uint16_t)(p->crc | ((uint16_t)b << 8));
        p->state = PS_SYNC0;
        uint8_t hdr[3] = { p->type, (uint8_t)p->len, (uint8_t)(p->len >> 8) };
        uint16_t crc = proto_crc16(0xFFFF, hdr, sizeof hdr);
        crc = proto_crc16(crc, p->payload, p->len);
        return (crc == p->crc) ? PP_FRAME : PP_BAD_CRC;
    }

    default:
        proto_parser_reset(p);
        break;
    }
    return PP_NONE;
}

/* ---------------- ADS131E04 configuration helpers ---------------- */

/* CHnSET.GAIN[2:0]: 001 = 1, 010 = 2, 100 = 4, 101 = 8, 110 = 12.
 * 000, 011, 111 are "do not use". */
uint8_t cfg_gain_code(uint8_t gain)
{
    switch (gain) {
    case 1:  return 0x1;
    case 2:  return 0x2;
    case 4:  return 0x4;
    case 8:  return 0x5;
    case 12: return 0x6;
    default: return 0xFF;
    }
}

/* CHnSET.MUX[2:0]: 000 normal, 001 shorted, 011 MVDD, 100 temperature,
 * 101 test signal. 010, 110, 111 are "do not use". */
bool cfg_mux_valid(uint8_t mux)
{
    return mux == 0 || mux == 1 || mux == 3 || mux == 4 || mux == 5;
}

/* CONFIG1.DR[2:0] with the internal 2.048 MHz oscillator:
 * 000 64 kSPS (16 bit), 001 32 kSPS (16 bit), 010 16 kSPS, 011 8 kSPS,
 * 100 4 kSPS, 101 2 kSPS, 110 1 kSPS (24 bit), 111 do not use. */
uint32_t cfg_dr_rate(uint8_t dr)
{
    if (dr > 6) {
        return 0;
    }
    return 64000u >> dr;
}

uint8_t cfg_dr_bytes(uint8_t dr)
{
    if (dr > 6) {
        return 0;
    }
    return (dr <= 1) ? 2 : 3;
}

int proto_parse_start(const uint8_t *payload, uint16_t len, start_cfg_t *cfg)
{
    if (len != START_PAYLOAD_LEN) {
        return ST_BAD_LEN;
    }
    memset(cfg, 0, sizeof *cfg);
    cfg->dr = payload[0];
    cfg->ch_mask = payload[1];
    for (int i = 0; i < ADS_MAX_CH; i++) {
        cfg->gain[i] = payload[2 + i];
        cfg->mux[i] = payload[6 + i];
    }
    cfg->duration_ms = get_u32(&payload[10]);
    cfg->flags = payload[14];
    cfg->test_cfg = payload[15];
    return ST_OK;
}

int proto_plan(const start_cfg_t *cfg, uint32_t uart_baud, uint32_t max_load_pct, acq_plan_t *plan)
{
    memset(plan, 0, sizeof *plan);

    /* 64 kSPS (dr 0) is rejected: one DRDY every 15.6 us cannot be serviced
     * reliably by an interrupt + task on the ESP32. dr 7 is reserved. */
    if (cfg->dr < 1 || cfg->dr > 6) {
        return ST_BAD_PARAM;
    }
    if ((cfg->ch_mask & 0x0F) == 0 || (cfg->ch_mask & 0xF0) != 0) {
        return ST_BAD_PARAM;
    }
    if ((cfg->flags & (uint8_t)~START_FLAGS_MASK) != 0) {
        return ST_BAD_PARAM;
    }
    if ((cfg->test_cfg & (uint8_t)~0x07u) != 0 || (cfg->test_cfg & 0x03) == 0x02) {
        return ST_BAD_PARAM;   /* TEST_FREQ = 10 is "not used" */
    }

    uint8_t n_ch = 0;
    uint8_t highest = 0;
    for (int i = 0; i < ADS_MAX_CH; i++) {
        if (cfg->ch_mask & (1u << i)) {
            if (cfg_gain_code(cfg->gain[i]) == 0xFF || !cfg_mux_valid(cfg->mux[i])) {
                return ST_BAD_PARAM;
            }
            n_ch++;
            highest = (uint8_t)i;
        }
    }

    uint32_t rate = cfg_dr_rate(cfg->dr);
    uint8_t bpc = cfg_dr_bytes(cfg->dr);

    uint64_t n_samples = ((uint64_t)cfg->duration_ms * rate) / 1000u;
    if (n_samples > 0xFFFFFFFFull) {
        return ST_BAD_PARAM;
    }
    if (cfg->duration_ms != 0 && n_samples == 0) {
        return ST_BAD_PARAM;
    }

    uint32_t bps = (uint32_t)n_ch * bpc + ((cfg->flags & START_FLAG_STATUS) ? ADS_STATUS_BYTES : 0);

    uint32_t spp_size = DATA_MAX_SAMPLE_BYTES / bps;
    uint32_t spp_time = rate / DATA_TARGET_PKT_PER_S;
    if (spp_time < 1) {
        spp_time = 1;
    }
    uint32_t spp = (spp_size < spp_time) ? spp_size : spp_time;
    if (spp > 255) {
        spp = 255;
    }
    if (spp < 1) {
        spp = 1;
    }

    /* UART bandwidth (8N1 = 10 bits per byte):
     * rate * bps + (rate / spp) * frame_overhead <= baud / 10 * load
     * multiplied by spp * 100 to stay in integers. */
    uint64_t overhead = PROTO_OVERHEAD + DATA_HDR_LEN;
    uint64_t need = ((uint64_t)rate * bps * spp + (uint64_t)rate * overhead) * 100u * 10u;
    uint64_t have = (uint64_t)uart_baud * max_load_pct * spp;
    if (need > have) {
        return ST_BANDWIDTH;
    }

    plan->sample_rate = rate;
    plan->n_samples = (uint32_t)n_samples;
    plan->n_ch = n_ch;
    plan->bytes_per_ch = bpc;
    plan->bytes_per_sample = (uint8_t)bps;
    plan->spp = (uint8_t)spp;
    /* status word + channels up to the highest enabled one.
     * The channel order in the ADC frame is fixed, so reading fewer bytes
     * never shifts the position of CH1..CH4. */
    plan->frame_len = (uint8_t)(ADS_STATUS_BYTES + (highest + 1u) * bpc);
    return ST_OK;
}
