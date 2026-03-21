/*
 * si5351v2.c
 *
 * Si5351 clock synthesizer driver using bit-banged I2C on GPIO22/23.
 *
 * Debug output is controlled by i2cbb_set_debug() in i2cbb.c.
 * This file adds its own [SI5351] prefix so you can grep the two layers
 * independently:
 *   grep '\[BB-I2C\]'  -- raw bit-bang bus transactions
 *   grep '\[SI5351\]'  -- frequency and PLL/multisynth calculations
 *
 * Set SI5351_DEBUG to 1 at compile time (or uncomment the define below)
 * to enable the [SI5351] prints without enabling raw bus logging.
 */

#include <stdio.h>
#include <linux/types.h>
#include <stdint.h>
#include <wiringPi.h>
#include "i2cbb.h"
#include "si5351.h"

/* Uncomment to enable SI5351-level debug prints independently of i2cbb: */
/* #define SI5351_DEBUG 1 */

#ifndef SI5351_DEBUG
#define SI5351_DEBUG 1
#endif

#define SI5351_LOG(fmt, ...) \
    do { if (SI5351_DEBUG) printf("[SI5351] " fmt, ##__VA_ARGS__); } while (0)

#define SDA 23
#define SCL 22

#define SI_CLK0_CONTROL  16
#define SI_CLK1_CONTROL  17
#define SI_CLK2_CONTROL  18
#define SI_SYNTH_PLL_A   26
#define SI_SYNTH_PLL_B   34
#define SI_SYNTH_MS_0    42
#define SI_SYNTH_MS_1    50
#define SI_SYNTH_MS_2    58
#define SI_PLL_RESET    177

#define SI_R_DIV_1    0b00000000
#define SI_R_DIV_2    0b00010000
#define SI_R_DIV_4    0b00100000
#define SI_R_DIV_8    0b00110000
#define SI_R_DIV_16   0b01000000
#define SI_R_DIV_32   0b01010000
#define SI_R_DIV_64   0b01100000
#define SI_R_DIV_128  0b01110000

#define SI_CLK_SRC_PLL_A  0b00000000
#define SI_CLK_SRC_PLL_B  0b00100000

#define SI5351_CLK_PLL_SELECT_B     (1<<5)
#define SI5351_CLK_INTEGER_MODE     (1<<6)
#define SI5351_CLK_INPUT_MULTISYNTH_N (3<<2)

#define SI5351_CLK_DRIVE_STRENGTH_MASK  (3<<0)
#define SI5351_CLK_DRIVE_STRENGTH_2MA   (0<<0)
#define SI5351_CLK_DRIVE_STRENGTH_4MA   (1<<0)
#define SI5351_CLK_DRIVE_STRENGTH_6MA   (2<<0)
#define SI5351_CLK_DRIVE_STRENGTH_8MA   (3<<0)

#define PLL_N    35
#define PLLFREQ  (xtal_freq_calibrated * PLL_N)

int xtal_freq_calibrated = 25000000;  /* TCXO; was 25012725 for XO */

uint32_t plla_freq, pllb_freq;

static int i2c_error_count = 0;

#define SI5351_ADDR 0x60

/*
 * i2cSendRegister - write one byte to one Si5351 register.
 *
 * Retries indefinitely on NACK (the original behaviour) but now prints the
 * retry count, the register number, and the value so you can spot a stuck bus
 * vs. an occasional glitch.
 */
static void i2cSendRegister(uint8_t reg, uint8_t val) {
    int attempts = 0;
    while (i2cbb_write_byte_data(SI5351_ADDR, reg, val) < 0) {
        attempts++;
        i2c_error_count++;
        printf("[SI5351] WARNING: I2C write failed on attempt %d "
               "reg=0x%02X val=0x%02X (total errors=%d)\n",
               attempts, reg, val, i2c_error_count);
        if (attempts >= 5) {
            printf("[SI5351] ERROR: giving up on reg=0x%02X after %d attempts - "
                   "check SDA=GPIO%d SCL=GPIO%d wiring and 3.3V rail (U7)\n",
                   reg, attempts, SDA, SCL);
            return;
        }
        delay(1);
    }
    /* Log every successful register write at verbose level */
    SI5351_LOG("reg[0x%02X] <- 0x%02X\n", reg, val);
}

void si5351_reset() {
    SI5351_LOG("PLL reset\n");
    i2cSendRegister(SI_PLL_RESET, 0xA0);
}

void si5351a_clkoff(uint8_t clk) {
    SI5351_LOG("clock %d off\n", clk);
    i2cSendRegister(clk, 0x80);
}

/*
 * setup_pll - program PLL A or B feedback divider.
 *
 * fVCO = fXTAL * (mult + num/denom)
 * Prints the integer mult and fractional num/denom so you can verify the
 * VCO frequency manually: fVCO = xtal * (mult + num/denom).
 */
static void setup_pll(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom) {
    uint32_t P1, P2, P3;

    if (num == 0) {
        P1 = 128 * mult - 512;
        P2 = 0;
        P3 = 1;
    } else {
        P1 = (uint32_t)(128 * ((float)num / (float)denom));
        P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
        P2 = (uint32_t)(128 * ((float)num / (float)denom));
        P2 = (uint32_t)(128 * num - denom * P2);
        P3 = denom;
    }

    SI5351_LOG("setup_pll: pll_reg=0x%02X mult=%u num=%u denom=%u "
               "-> P1=%u P2=%u P3=%u  fVCO~=%u Hz\n",
               pll, mult, num, denom, P1, P2, P3,
               (uint32_t)((float)xtal_freq_calibrated * (mult + (float)num / (float)denom)));

    i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
    i2cSendRegister(pll + 1, (P3 & 0x000000FF));
    i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
    i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
    i2cSendRegister(pll + 4, (P1 & 0x000000FF));
    i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
    i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
    i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

/*
 * setup_multisynth - program output divider for one clock output.
 *
 * fOUT = fVCO / (divider + num/denom) / R
 * Prints the chosen synth register base, divider, and P1/P2/P3 so you can
 * cross-check with the AN619 spreadsheet.
 */
static void setup_multisynth(uint8_t clk, uint8_t pllSource, uint32_t divider,
                               uint32_t num, uint32_t denom, uint32_t rdiv,
                               uint8_t drive_strength) {
    uint8_t synth, control;

    switch (clk) {
        case 0: synth = 42; control = 16; break;
        case 1: synth = 50; control = 17; break;
        default: synth = 58; control = 18; break;
    }

    uint8_t dat;
    uint32_t P1, P2, P3;
    uint32_t div4 = 0;

#define SI5351_DIVBY4 (3<<2)

    if (divider == 4) {
        div4 = SI5351_DIVBY4;
        P1 = P2 = 0;
        P3 = 1;
    } else if (num == 0) {
        P1 = 128 * divider - 512;
        P2 = 0;
        P3 = 1;
    } else {
        P1 = 128 * divider + ((128 * num) / denom) - 512;
        P2 = 128 * num - denom * ((128 * num) / denom);
        P3 = denom;
    }

    SI5351_LOG("setup_multisynth: clk=%d synth_reg=0x%02X ctrl_reg=0x%02X "
               "pllSrc=0x%02X div=%u num=%u denom=%u rdiv=0x%02X "
               "-> P1=%u P2=%u P3=%u\n",
               clk, synth, control, pllSource,
               divider, num, denom, rdiv, P1, P2, P3);

    i2cSendRegister(synth + 0, (P3 & 0x0000FF00) >> 8);
    i2cSendRegister(synth + 1, (P3 & 0x000000FF));
    i2cSendRegister(synth + 2, ((P1 & 0x00030000) >> 16) | div4 | rdiv);
    i2cSendRegister(synth + 3, (P1 & 0x0000FF00) >> 8);
    i2cSendRegister(synth + 4, (P1 & 0x000000FF));
    i2cSendRegister(synth + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
    i2cSendRegister(synth + 6, (P2 & 0x0000FF00) >> 8);
    i2cSendRegister(synth + 7, (P2 & 0x000000FF));

    dat = drive_strength | SI5351_CLK_INPUT_MULTISYNTH_N;
    if (pllSource == SI_SYNTH_PLL_B)
        dat |= SI_CLK_SRC_PLL_B;
    if (num == 0)
        dat |= SI5351_CLK_INTEGER_MODE;

    SI5351_LOG("setup_multisynth: clk=%d control reg=0x%02X <- 0x%02X\n",
               clk, control, dat);
    i2cSendRegister(control, dat);
}

static void set_frequency_fixedpll(int clk, int pll, uint32_t pllfreq,
                                    uint32_t freq, int rdiv,
                                    uint8_t drive_strength) {
    int32_t denom    = 0x80000l;
    uint32_t divider = pllfreq / freq;
    uint32_t integer_part = divider * freq;
    uint32_t reminder    = pllfreq - integer_part;
    uint32_t multi       = pllfreq / xtal_freq_calibrated;
    uint32_t num         = ((uint64_t)reminder * (uint64_t)denom) / freq;

    SI5351_LOG("set_frequency_fixedpll: clk=%d freq=%u pllfreq=%u "
               "divider=%u multi=%u num=%u denom=%u\n",
               clk, freq, pllfreq, divider, multi, num, denom);

    setup_pll(pll, multi, 0, denom);
    setup_multisynth(clk, pll, divider, num, denom, rdiv, drive_strength);
}

static void set_freq_fixeddiv(int clk, int pll, uint32_t frequency,
                               int divider, uint8_t drive_strength) {
    int32_t denom   = 0x80000;
    int32_t pllfreq = frequency * divider;
    int32_t multi   = pllfreq / xtal_freq_calibrated;
    int32_t num     = ((uint64_t)(pllfreq % xtal_freq_calibrated) * 0x80000)
                      / xtal_freq_calibrated;

    SI5351_LOG("set_freq_fixeddiv: clk=%d freq=%u divider=%d "
               "pllfreq=%d multi=%d num=%d denom=%d\n",
               clk, frequency, divider, pllfreq, multi, num, denom);

    setup_pll(pll, multi, num, denom);
    setup_multisynth(clk, pll, divider, 0, 1, SI_R_DIV_1, drive_strength);
}

/*
 * si5351bx_setfreq - top-level frequency-set entry point.
 *
 * CLK1 uses PLL B; all others use PLL A.
 * Prints the requested frequency and the pll_div chosen, which is the most
 * useful number to check when a frequency comes out wrong.
 */
void si5351bx_setfreq(uint8_t clk, uint32_t frequency) {
    int pll = (clk == 1) ? SI_SYNTH_PLL_B : SI_SYNTH_PLL_A;

    int pll_div = 650000000l / frequency;
    if (pll_div * 650000000l != frequency)
        pll_div++;
    if (pll_div & 1)
        pll_div++;

    printf("[SI5351] setfreq: clk=%d freq=%u Hz pll_div=%d "
           "xtal=%d Hz (pll=0x%02X)\n",
           clk, frequency, pll_div, xtal_freq_calibrated, pll);

    set_freq_fixeddiv(clk, pll, frequency, pll_div,
                      SI5351_CLK_DRIVE_STRENGTH_8MA);
}

/*
 * si5351_set_calibration - update the TCXO/XO reference frequency.
 *
 * This is the single most common cause of a frequency being slightly off.
 * Print both old and new so you can confirm the value was applied.
 */
void si5351_set_calibration(int32_t cal) {
    printf("[SI5351] calibration: xtal %d -> %d Hz (delta=%+d Hz)\n",
           xtal_freq_calibrated, cal, cal - xtal_freq_calibrated);
    xtal_freq_calibrated = cal;
}

void si5351bx_init() {
    printf("[SI5351] init: SDA=GPIO%d SCL=GPIO%d addr=0x%02X xtal=%d Hz\n",
           SDA, SCL, SI5351_ADDR, xtal_freq_calibrated);
    i2cbb_init(SDA, SCL);
    delay(10);
    si5351_reset();
    delay(10);
    si5351a_clkoff(SI_CLK0_CONTROL);
    si5351a_clkoff(SI_CLK1_CONTROL);
    si5351a_clkoff(SI_CLK2_CONTROL);
    printf("[SI5351] init complete, all clocks off\n");
}
