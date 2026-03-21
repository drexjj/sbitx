/*
 * si570.c
 *
 * Si570 oscillator driver using wiringPi hardware I2C (/dev/i2c-1).
 *
 * Debug output uses the [HW-I2C] prefix so you can separate it from the
 * bit-bang [BB-I2C] and [SI5351] streams:
 *   grep '\[HW-I2C\]'  -- Si570 hardware I2C transactions
 *
 * Enable with:  si570_set_debug(1)   -- errors + frequency calculations
 *               si570_set_debug(2)   -- also individual register reads/writes
 */

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <linux/types.h>
#include <stdint.h>

#define SI570_ADDRESS 0x55

static int fd_si570 = -1;
static unsigned char dco_reg[13];

#define RFREQ_37_32_MASK  0x3f
#define FDCO_MIN  4800000000.0
#define FDCO_MAX  5670000000.0

static unsigned int selected_n1;
static unsigned int selected_hs;
static uint64_t    selected_rf   = 0;
static unsigned long selected_freq = 0;

int fxtal = 114264401;

/* 0=silent 1=errors+calculations 2=register-level */
static int si570_debug_level = 2;

void si570_set_debug(int level) {
    si570_debug_level = level;
    printf("[HW-I2C] Si570 debug level set to %d (addr=0x%02X fd=%d)\n",
           level, SI570_ADDRESS, fd_si570);
}

/*
 * si570_dumpregs - print raw register bytes 7-12 with field decode.
 *
 * Useful to paste into the Si570 datasheet register map or compare
 * before/after a frequency change.
 */
void si570_dumpregs() {
    printf("[HW-I2C] Si570 registers 7-12:\n");
    for (int i = 7; i <= 12; i++)
        printf("[HW-I2C]   reg[%2d] = 0x%02X\n", i, dco_reg[i]);

    /* Decode the fields for easier cross-checking */
    unsigned int hs_raw = (dco_reg[7] >> 5) & 0x7;
    unsigned int n1_raw = ((dco_reg[7] & 0x1F) << 2) | ((dco_reg[8] >> 6) & 0x3);
    uint64_t rfreq_raw  = ((uint64_t)(dco_reg[8] & RFREQ_37_32_MASK) << 32)
                         | ((uint64_t)dco_reg[9]  << 24)
                         | ((uint64_t)dco_reg[10] << 16)
                         | ((uint64_t)dco_reg[11] << 8)
                         |  (uint64_t)dco_reg[12];

    unsigned int hs = hs_raw + 4;
    unsigned int n1 = n1_raw + 1;
    double rfreq_frac = (double)rfreq_raw / 268435456.0;
    double fdco = rfreq_frac * fxtal;
    double fout = fdco / (hs * n1);

    printf("[HW-I2C]   decoded: HS=%u N1=%u RFREQ=%.10f\n",
           hs, n1, rfreq_frac);
    printf("[HW-I2C]   fDCO=%.0f Hz  fOUT=%.0f Hz  fXTAL=%d Hz\n",
           fdco, fout, fxtal);
}

void si570_read() {
    if (fd_si570 < 0) {
        printf("[HW-I2C] ERROR: si570_read called before si570_init\n");
        return;
    }
    for (int i = 7; i <= 12; i++) {
        int val = wiringPiI2CReadReg8(fd_si570, i);
        if (val < 0) {
            printf("[HW-I2C] ERROR: read failed at reg[%d] fd=%d addr=0x%02X\n",
                   i, fd_si570, SI570_ADDRESS);
        } else {
            dco_reg[i] = (unsigned char)val;
            if (si570_debug_level >= 2)
                printf("[HW-I2C] read: reg[%2d] = 0x%02X\n", i, dco_reg[i]);
        }
    }
}

void si570_write() {
    if (fd_si570 < 0) {
        printf("[HW-I2C] ERROR: si570_write called before si570_init\n");
        return;
    }

    /* Freeze the DCO before changing the frequency word */
    int freeze_reg = wiringPiI2CReadReg8(fd_si570, 137);
    if (freeze_reg < 0) {
        printf("[HW-I2C] ERROR: failed to read freeze register (reg 137) "
               "fd=%d addr=0x%02X\n", fd_si570, SI570_ADDRESS);
        return;
    }
    if (si570_debug_level >= 2)
        printf("[HW-I2C] freeze: reg[137]=0x%02X -> writing 0x%02X\n",
               freeze_reg, freeze_reg | 0x10);

    wiringPiI2CWriteReg8(fd_si570, 137, freeze_reg | 0x10);

    for (int i = 7; i <= 12; i++) {
        int ret = wiringPiI2CWriteReg8(fd_si570, i, dco_reg[i]);
        if (ret < 0)
            printf("[HW-I2C] ERROR: write failed at reg[%d] val=0x%02X "
                   "fd=%d addr=0x%02X\n",
                   i, dco_reg[i], fd_si570, SI570_ADDRESS);
        else if (si570_debug_level >= 2)
            printf("[HW-I2C] write: reg[%2d] <- 0x%02X\n", i, dco_reg[i]);
    }

    wiringPiI2CWriteReg8(fd_si570, 137, freeze_reg & 0xef);  /* unfreeze */
    wiringPiI2CWriteReg8(fd_si570, 135, 0x40);               /* apply new freq */

    if (si570_debug_level >= 2)
        printf("[HW-I2C] unfreeze and apply: reg[137]=0x%02X reg[135]=0x40\n",
               freeze_reg & 0xef);
}

/*
 * si570_freq - calculate and apply the HS/N1/RFREQ dividers for frequency f.
 *
 * Prints the chosen HS and N1 values, the raw RFREQ integer, and the
 * resulting fDCO so you can verify the PLL is in range (4.8-5.67 GHz).
 * If no valid combination is found it says so rather than silently failing.
 */
void si570_freq(unsigned long f) {
    unsigned int hs, n1;
    float f_dco;
    int found = 0;

    printf("[HW-I2C] si570_freq: requested=%lu Hz  fXTAL=%d Hz\n", f, fxtal);

    for (hs = 11; hs >= 4; hs--) {
        if (hs == 8 || hs == 10)
            continue;

        n1 = (long)(FDCO_MIN / hs) / f;
        if (!n1 || (n1 & 1))
            n1++;

        while (n1 <= 128) {
            f_dco = (float)f * (float)hs * (float)n1;
            if (f_dco > FDCO_MAX)
                break;

            if (FDCO_MIN < f_dco && f_dco < FDCO_MAX) {
                selected_n1 = n1;
                selected_hs = hs;
                selected_rf  = (uint64_t)((f_dco / fxtal) * 268435456.0);

                dco_reg[7]  = (selected_hs - 4) << 5;
                dco_reg[7] |= ((selected_n1 - 1) >> 2);
                dco_reg[8]  = ((selected_n1 - 1) & 0x3) << 6
                             | (selected_rf >> 32) & RFREQ_37_32_MASK;
                dco_reg[9]  = (selected_rf >> 24) & 0xff;
                dco_reg[10] = (selected_rf >> 16) & 0xff;
                dco_reg[11] = (selected_rf >>  8) & 0xff;
                dco_reg[12] =  selected_rf        & 0xff;

                printf("[HW-I2C] si570_freq: HS=%u N1=%u "
                       "RFREQ_int=0x%010llX (%.10f)  fDCO=%.0f Hz\n",
                       selected_hs, selected_n1,
                       (unsigned long long)selected_rf,
                       (double)selected_rf / 268435456.0,
                       (double)f_dco);

                if (si570_debug_level >= 2)
                    si570_dumpregs();

                si570_write();
                selected_freq = f;
                found = 1;
                return;
            }
            n1 += (n1 == 1 ? 1 : 2);
        }
    }

    if (!found)
        printf("[HW-I2C] ERROR: si570_freq: no valid HS/N1 combination found "
               "for %lu Hz (fDCO range %.0f-%.0f Hz)\n",
               f, FDCO_MIN, FDCO_MAX);
}

/*
 * si570_init - open the hardware I2C device and read the startup registers.
 *
 * Prints the fd so you can confirm the kernel assigned the expected device
 * file.  A negative fd means /dev/i2c-1 is missing or the address is wrong.
 */
void si570_init() {
    fd_si570 = wiringPiI2CSetup(SI570_ADDRESS);
    if (fd_si570 < 0) {
        printf("[HW-I2C] ERROR: wiringPiI2CSetup failed for addr=0x%02X "
               "(is /dev/i2c-1 enabled? check raspi-config)\n",
               SI570_ADDRESS);
        return;
    }
    printf("[HW-I2C] Si570 init: addr=0x%02X fd=%d\n", SI570_ADDRESS, fd_si570);
    si570_read();
    if (si570_debug_level >= 1)
        si570_dumpregs();
}
