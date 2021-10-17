#include "bme280.h"
/*
            0x88 / 0x89        T1 [7:0] / [15:8]        unsigned
            0x8A / 0x8B        T2 [7:0] / [15:8]
            0x8C / 0x8D        T3 [7:0] / [15:8]

            0x8E / 0x8F        P1 [7:0] / [15:8]        unsigned
            0x90 / 0x91        P2 [7:0] / [15:8]
            0x92 / 0x93        P3 [7:0] / [15:8]
            0x94 / 0x95        P4 [7:0] / [15:8]
            0x96 / 0x97        P5 [7:0] / [15:8]
            0x98 / 0x99        P6 [7:0] / [15:8]
            0x9A / 0x9B        P7 [7:0] / [15:8]
            0x9C / 0x9D        P8 [7:0] / [15:8]
            0x9E / 0x9F        P9 [7:0] / [15:8]

            0xA1               H1 [7:0]                 unsigned
            0xE1 / 0xE2        H2 [7:0] / [15:8]
            0xE3               H3 [7:0]                 unsigned
            0xE4 / 0xE5[3:0]   H4 [11:4] / [3:0]   signed!
            0xE5[7:4] / 0xE6   H5 [3:0] / [11:4]   signed!
            0xE7               H6 [7:0]
*/

static inline int8_t read_int8(uint8_t **pp) {
    uint8_t *p = *pp;
    int8_t r = p[0];
    (*pp)++;
    return r;
}
static inline int16_t read_int16(uint8_t **pp) {
    uint8_t *p = *pp;
    int16_t r = p[0];
    r |= ((int16_t)p[1]) << 8;
    (*pp) += 2;
    return r;
}
static inline uint16_t read_uint16(uint8_t **pp) {
    uint8_t *p = *pp;
    uint16_t r = p[0];
    r |= ((uint16_t)p[1]) << 8;
    (*pp) += 2;
    return r;
}

// decode the structure from a buffer of bytes 0x88..0xA1 (inclusive) and 0xE1..E7 (inclusive)
void bme_decodeLinearisationParameters(struct LinearisationParameters *p, uint8_t *buf88, uint8_t *bufe1) {
    p->T[0] = 0;  // not used
    p->T[1] = read_uint16(&buf88);  // t_0 << 4
    p->T[2] = read_int16(&buf88);  // t >> 14
    p->T[3] = read_int16(&buf88);  // t^2 >> 34

    p->P[0] = 0;
    p->P[1] = read_uint16(&buf88);
    p->P[2] = read_int16(&buf88);
    p->P[3] = read_int16(&buf88);
    p->P[4] = read_int16(&buf88);
    p->P[5] = read_int16(&buf88);
    p->P[6] = read_int16(&buf88);
    p->P[7] = read_int16(&buf88);
    p->P[8] = read_int16(&buf88);
    p->P[9] = read_int16(&buf88);
    ++buf88;
    p->H[1] = *buf88;

    p->H[2] = read_int16(&bufe1);
    p->H[3] = *bufe1++;
    int8_t msb = bufe1[0];
    p->H[4] = (((int16_t)(msb)) * 16) | (bufe1[1] & 0xf);
    msb = bufe1[2];
    p->H[5] = (((int16_t)(msb)) * 16) | (bufe1[1] >> 4);
    p->H[6] = (int8_t)bufe1[3];
}

// decode the temperature, pressure and humidity from the buffer of bytes 0xf7..0xfe (inclusive)
// Returns temperature in milliKelvin, pressure in milliPascal and humidity as a fraction * 1E6 (0...1000000) for
// 0..100%
void bme_decode(struct LinearisationParameters *par, uint8_t *buff7, int32_t *t_mdegc, int32_t *p_mpa, int32_t *hume6) {
    int32_t adc_T = ((uint16_t)buff7[3] << 12) | ((uint16_t)buff7[4] << 4) | ((uint16_t)buff7[5] >> 4);
    int32_t adc_P = ((uint16_t)buff7[0] << 12) | ((uint16_t)buff7[1] << 4) | ((uint16_t)buff7[2] >> 4);
    int32_t adc_H = (uint16_t)buff7[6] << 8 | buff7[7];

    // Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
    // t_fine carries fine temperature as global value
    int64_t t_fine;
    {
        double var1, var2, T;
        var1 = (((double)adc_T) - ((double)par->T[1]) * 16) * ((double)par->T[2]) / 16384.0;
        var2 = ((((double)adc_T) - ((double)par->T[1]) * 16) * (((double)adc_T) - ((double)par->T[1]) * 16)) *
                ((double)par->T[3]) / (131072.0 * 131072.0);
        t_fine = (int32_t)(var1 + var2);
        T = (var1 + var2) / 5120.0;
        *t_mdegc = T * 1000.0;
    }

    // Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
    {
        double var1, var2, p;
        var1 = ((double)t_fine / 2.0) - 64000.0;
        var2 = var1 * var1 * ((double)par->P[6]) / 32768.0;
        var2 = var2 + var1 * ((double)par->P[5]) * 2.0;
        var2 = (var2 / 4.0) + (((double)par->P[4]) * 65536.0);
        var1 = (((double)par->P[3]) * var1 * var1 / 524288.0 + ((double)par->P[2]) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * ((double)par->P[1]);
        if (var1 != 0.0) {
            p = 1048576.0 - (double)adc_P;
            p = (p - (var2 / 4096.0)) * 6250.0 / var1;
            var1 = ((double)par->P[9]) * p * p / 2147483648.0;
            var2 = p * ((double)par->P[8]) / 32768.0;
            p = p + (var1 + var2 + ((double)par->P[7])) / 16.0;

            *p_mpa = p * 1000.0;
        } else
            *p_mpa = 0;
    }
    // Returns humidity in %rH as as double. Output value of “46.332” represents  46.332 %rH

    {
        double var_H;

        var_H = (((double)t_fine) - 76800.0);
        var_H = (adc_H - (((double)par->H[4]) * 64.0 + ((double)par->H[5]) / 16384.0 * var_H)) *
                (((double)par->H[2]) / 65536.0 *
                 (1.0 + ((double)par->H[6]) / 67108864.0 * var_H * (1.0 + ((double)par->H[3]) / 67108864.0 * var_H)));
        var_H = var_H * (1.0 - ((double)par->H[1]) * var_H / 524288.0);

        if (var_H > 100.0)
            var_H = 100.0;
        else if (var_H < 0.0)
            var_H = 0.0;
        *hume6 = 10000 * var_H;
    }
}