/*
 * ads131e04.h
 * Low level driver for the TI ADS131E04 (ADS131E0x family, SBAS561C).
 *
 * All functions must be called from ONE task only (the acquisition task):
 * the SPI device is not shared and CS is driven manually.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- SPI opcodes (datasheet Table "Command Definitions") ---- */
#define ADS_CMD_WAKEUP      0x02
#define ADS_CMD_STANDBY     0x04
#define ADS_CMD_RESET       0x06
#define ADS_CMD_START       0x08
#define ADS_CMD_STOP        0x0A
#define ADS_CMD_OFFSETCAL   0x1A
#define ADS_CMD_RDATAC      0x10
#define ADS_CMD_SDATAC      0x11
#define ADS_CMD_RDATA       0x12
#define ADS_CMD_RREG        0x20   /* | address, second byte = count - 1 */
#define ADS_CMD_WREG        0x40   /* | address, second byte = count - 1 */

/* ---------------- Register map ---------------- */
#define ADS_REG_ID          0x00
#define ADS_REG_CONFIG1     0x01
#define ADS_REG_CONFIG2     0x02
#define ADS_REG_CONFIG3     0x03
#define ADS_REG_FAULT       0x04
#define ADS_REG_CH1SET      0x05   /* CHnSET = 0x05 + (n - 1) */
#define ADS_REG_FAULT_STATP 0x12
#define ADS_REG_FAULT_STATN 0x13
#define ADS_REG_GPIO        0x14
#define ADS_NUM_REGS        0x15

/* CONFIG1 = 1 | DAISY_IN | CLK_EN | 1 | 0 | DR[2:0]
 * DAISY_IN = 0 (daisy chain mode, single device)
 * CLK_EN   = 0 (MANDATORY on this board: the CLK pin is tied to GND,
 *              enabling the oscillator output would short it) */
#define ADS_CONFIG1_BASE    0x90
#define ADS_CONFIG1_CLK_EN  0x20

/* CONFIG2 = 1 | 1 | 1 | INT_TEST | 0 | TEST_AMP | TEST_FREQ[1:0] */
#define ADS_CONFIG2_BASE    0xE0
#define ADS_CONFIG2_INT_TEST 0x10

/* CONFIG3 = PDB_REFBUF | 1 | VREF_4V | 0 | OPAMP_REF | PDB_OPAMP | 0 | 0
 * Internal reference buffer ON, VREF = 2.4 V (AVDD is 3.3 V, the 4 V
 * reference needs a 5 V analog supply), op amp powered down. */
#define ADS_CONFIG3_VALUE   0xC0

/* CHnSET = PD | GAIN[2:0] | 0 | MUX[2:0] */
#define ADS_CHSET_PD        0x80
#define ADS_CHSET_OFF       0x91   /* powered down, gain 1, input shorted */

/* Status word: first 4 bits are always 1100 */
#define ADS_STATUS_SYNC_MASK 0xF0
#define ADS_STATUS_SYNC      0xC0

/* Longest frame read per conversion: 24 bit status + 4 channels x 24 bit.
 * Channels 1..4 are always the first ones after the status word, whatever
 * the total readback length of the device is, so reading only this many
 * bytes (or fewer) never shifts the channel positions. */
#define ADS_MAX_FRAME_BYTES 15

/* ID register: REV_ID[2:0] = 110, bit4 = 1, bits3:2 = 00, NU_CH[1:0]
 * NU_CH: 00 = ADS131E04, 01 = ADS131E06, 10 = ADS131E08 */
#define ADS_ID_FAMILY_MASK  0xFC
#define ADS_ID_FAMILY       0xD0
#define ADS_ID_NU_CH_MASK   0x03

esp_err_t ads_bus_init(void);
void      ads_bus_acquire(void);
void      ads_bus_release(void);

/* Power up sequence (datasheet flow chart): wait, RESET pulse, SDATAC,
 * internal reference on, registers to a known idle state. Returns the ID
 * register. Fails with ESP_ERR_NOT_FOUND if the ID does not belong to the
 * ADS131E0x family (wiring / power problem). */
esp_err_t ads_powerup(uint8_t *id_out, bool first_boot);

void      ads_cmd(uint8_t opcode);
uint8_t   ads_rreg(uint8_t addr);
void      ads_rregs(uint8_t addr, uint8_t *dst, uint8_t count);
void      ads_wreg(uint8_t addr, uint8_t value);

/* Writes and reads back. Returns false if the read value differs. */
bool      ads_wreg_verify(uint8_t addr, uint8_t value);

void      ads_set_start(bool high);

/* Data retrieval with the RDATA command (device in SDATAC mode, START high).
 * Datasheet: with RDATA the read can overlap the next DRDY without data
 * corruption. Clocks out the opcode, then len bytes (status + channels). */
void      ads_read_data(uint8_t *dst, size_t len);

/* DRDY falling edges counted in hardware by the PCNT peripheral: exact even
 * if the interrupt is serviced late. Reset only while START is low. */
void      ads_drdy_count_reset(void);
uint32_t  ads_drdy_count(void);

/* DRDY interrupt (falling edge) used only to wake up task_to_notify. Also
 * sets up the PCNT edge counter. Call from the acquisition task. */
esp_err_t ads_drdy_init(void *task_to_notify);
void      ads_drdy_irq_enable(bool enable);

#ifdef __cplusplus
}
#endif
