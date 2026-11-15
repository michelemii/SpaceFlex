/*
 * board.h
 * Pin map and build time configuration.
 *
 * Pin map taken from the schematic (ESP32 DevKitC U8 <-> ADS131E04 U10):
 *
 *   ADS131E04          ESP32 DevKitC
 *   CS      (pin 39) -> GPIO23
 *   SCLK    (pin 40) -> GPIO22
 *   DIN     (pin 34) -> GPIO21   (ESP32 MOSI)
 *   DOUT    (pin 43) -> GPIO19   (ESP32 MISO)
 *   DRDY    (pin 47) -> GPIO18   (active low, interrupt)
 *   START   (pin 38) -> GPIO5
 *   RESET   (pin 36) -> GPIO4    (active low)
 *   PWDN, CLKSEL     -> +3.3V_D  (hardwired: always on, internal oscillator)
 *   CLK              -> GND      (so CONFIG1.CLK_EN must NEVER be set to 1)
 *   DAISY_IN         -> GND
 *
 * The SPI pins are not the native VSPI IO_MUX pins, so the signals go through
 * the GPIO matrix. That is fine up to well above the SCLK used here.
 */
#pragma once

/* ---------------- ADS131E04 pins ---------------- */
#define PIN_ADS_CS      23
#define PIN_ADS_SCLK    22
#define PIN_ADS_DIN     21   /* ESP32 MOSI */
#define PIN_ADS_DOUT    19   /* ESP32 MISO */
#define PIN_ADS_DRDY    18
#define PIN_ADS_START   5
#define PIN_ADS_RESET   4

/* ---------------- SPI ----------------
 * ADS131E0x: tSCLK min = 50 ns (20 MHz) with DVDD 2.7 V to 3.6 V.
 * 8 MHz leaves a large timing margin through the GPIO matrix; a full RDATA
 * read (opcode + status + 4 x 24 bit) takes 16 us, well inside the 31 us
 * period of the fastest supported rate.
 */
#define ADS_SPI_HOST    SPI3_HOST
#define ADS_SPI_HZ      8000000

/* ---------------- UART to PC ----------------
 * UART0 is the USB bridge of the DevKitC (GPIO1 TX, GPIO3 RX).
 * 921600 works with both CP2102 and CP2102N bridges.
 * If your DevKitC has a CP2102N you can use 2000000 (set the same value in
 * the PC script with --baud). Higher baud = higher sample rates allowed.
 */
#define PC_UART_NUM     UART_NUM_0
#define PC_UART_BAUD    921600

/* Fraction of the UART capacity the firmware accepts to use (percent).
 * A START request that would need more is rejected with ST_BANDWIDTH. */
#define PC_UART_MAX_LOAD_PCT  85

/* ---------------- ADS131E04 analog setup ----------------
 * AVDD = 3.3 V on this board, so only the 2.4 V internal reference is legal
 * (VREF_4V = 1 requires a 5 V analog supply).
 */
#define ADS_VREF_MV     2400

/* Power up wait before the first RESET pulse. The datasheet flow chart waits
 * for the supplies and VCAP1 (22 uF) to settle; 1 s is the flow chart value. */
#define ADS_POWERUP_WAIT_MS   1000

/* Internal reference settling after PDB_REFBUF = 1 (datasheet: 150 ms typ). */
#define ADS_REF_SETTLE_MS     200

/* OFFSETCAL duration (datasheet: 153 ms) plus margin. */
#define ADS_OFFSETCAL_MS      200

/* Firmware version reported in the INFO frame. */
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
