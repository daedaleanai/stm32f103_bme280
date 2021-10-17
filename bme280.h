#pragma once

#include <stdint.h>

enum {

    BME280_READREG = 0x80,  // add to reg to read

    BME280_CALIBTP_REG = 0x08,  // r/o
    BME280_CALIBTP_LEN = (1 + 0x21 - 0x08),
    BME280_CALIBH_REG = 0x61,  // r/o
    BME280_CALIBH_LEN = (1 + 0x70 - 0x61),
    BME280_DATA_REG = 0x77,  // r/o
    BME280_DATA_LEN = (1 + 0x7e - 0x77),  // r/o

    BME280_REG_ID = 0x50,  // r/o
    BME280_REG_RESET = 0x60,  // w/o  write 0xB6 only
    BME280_REG_CTRLHUM = 0x72,  // r/w
    BME280_REG_STATUS = 0x73,  // r/o
    BME280_REG_CTRLMEAS = 0x74,  // r/w
    BME280_REG_CONFIG = 0x75,  // r/w

    BME280_RESET = 0xB6,

    // Tstandby in milliseconds (0 = 0.5ms)
    BME280_CONFIG_TSB62 = (1 << 5),
    BME280_CONFIG_TSB125 = (2 << 5),
    BME280_CONFIG_TSB250 = (3 << 5),
    BME280_CONFIG_TSB500 = (4 << 5),
    BME280_CONFIG_TSB1000 = (5 << 5),
    BME280_CONFIG_TSB10 = (6 << 5),
    BME280_CONFIG_TSB20 = (7 << 5),

    // Filter constant (0 = off)
    BME280_CONFIG_FLT2 = (1 << 2),
    BME280_CONFIG_FLT4 = (2 << 2),
    BME280_CONFIG_FLT8 = (3 << 2),
    BME280_CONFIG_FLT16 = (4 << 2),

    BME280_CONFIG_SPI3W = 1,  // enable 3 wire SPI

    // Pressure oversampling 0 = off
    BME280_CTRLMEAS_P1 = (1 << 5),
    BME280_CTRLMEAS_P2 = (2 << 5),
    BME280_CTRLMEAS_P4 = (3 << 5),
    BME280_CTRLMEAS_P8 = (4 << 5),
    BME280_CTRLMEAS_P16 = (5 << 5),

    // Temperature oversampling 0 = off
    BME280_CTRLMEAS_T1 = (1 << 2),
    BME280_CTRLMEAS_T2 = (2 << 2),
    BME280_CTRLMEAS_T4 = (3 << 2),
    BME280_CTRLMEAS_T8 = (4 << 2),
    BME280_CTRLMEAS_T16 = (5 << 2),

    BME280_CTRLMEAS_FORCE = 1,
    BME280_CTRLMEAS_NORMAL = 3,

    // Humidity oversampling 0 = off
    BME280_CTRLHUM_H1 = 1,
    BME280_CTRLHUM_H2 = 2,
    BME280_CTRLHUM_H4 = 3,
    BME280_CTRLHUM_H8 = 4,
    BME280_CTRLHUM_H16 = 5,

    BME280_STATUS_BUSY = 1 << 3,
    BME280_STATUS_IMUPD = 1 << 0,

};

struct LinearisationParameters {
    int32_t T[4];
    int32_t P[10];
    int32_t H[7];
};

// decode the structure from a buffer of bytes 0x88..0xA1 (inclusive) and 0xE1..E6 (inclusive)
void bme_decodeLinearisationParameters(struct LinearisationParameters *p, uint8_t *buf88, uint8_t *bufe1);

// decode the temperature, pressure and humidity from the buffer of bytes 0xf7..0xfe (inclusive)
// Returns temperature in milliKelvin, pressure in milliPascal and humidity as a fraction * 1E6 (0...1000000) for
// 0..100%
void bme_decode(struct LinearisationParameters *par, uint8_t *buff7, int32_t *t_mdegc, int32_t *p_mpa, int32_t *hume6);