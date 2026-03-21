/*
 * i2cbb.c
 *
 *  Created on: 06.03.2015
 *      Author: "Marek Wyborski"
 *
 * Debug instrumentation added:
 *   Call i2cbb_set_debug(level) before i2cbb_init() to enable logging.
 *   Level 1 = errors only, 2 = every transaction, 3 = verbose with pin state.
 *   All output is prefixed [BB-I2C] to distinguish it from hardware I2C logs.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include <unistd.h>
#include <wiringPi.h>
#include <linux/types.h>
#include <stdint.h>
#include <time.h>
#include "i2cbb.h"

static uint8_t PIN_SDA;
static uint8_t PIN_SCL;
static uint32_t sleepTimeNanos;
static struct timespec nanoSleepTime;
static uint32_t delayTicks;
int i2c_started = 0;

/* 0=silent 1=errors 2=transactions 3=verbose+pin state */
static int i2cbb_debug_level = 0;

void i2cbb_set_debug(int level) {
    i2cbb_debug_level = level;
    printf("[BB-I2C] debug level set to %d (SDA=GPIO%d SCL=GPIO%d)\n",
           level, PIN_SDA, PIN_SCL);
}

void i2cbb_init(uint8_t pin_number_sda, uint8_t pin_number_scl)
{
    PIN_SDA = pin_number_sda;
    PIN_SCL = pin_number_scl;
    sleepTimeNanos = 0;
    nanoSleepTime.tv_sec = 0;
    nanoSleepTime.tv_nsec = 0;
    delayTicks = 400;   /* empirically chosen: 2x the value that starts causing NACKs - N3SB */
    i2c_started = 0;

    nanoSleepTime.tv_sec = 0;
    nanoSleepTime.tv_nsec = 1;

    if (i2cbb_debug_level >= 1)
        printf("[BB-I2C] init: SDA=GPIO%d SCL=GPIO%d delayTicks=%u\n",
               PIN_SDA, PIN_SCL, delayTicks);
}

/* ---------- low-level GPIO helpers ---------- */

static int read_SCL() {
    pinMode(PIN_SCL, INPUT);
    return digitalRead(PIN_SCL);
}

static int read_SDA() {
    pinMode(PIN_SDA, INPUT);
    return digitalRead(PIN_SDA);
}

static void clear_SCL() {
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SCL, 0);
}

static void clear_SDA() {
    pinMode(PIN_SDA, OUTPUT);
    digitalWrite(PIN_SDA, 0);
}

/*
 * arbitration_lost - called when SDA is unexpectedly low while we're trying
 * to drive it high.  At level 3 we also sample the actual pin states so you
 * can confirm whether the bus is stuck low (held by a slave or a wiring fault)
 * vs. a software race.
 */
static void arbitration_lost(const char *where) {
    if (i2cbb_debug_level >= 1) {
        if (i2cbb_debug_level >= 3) {
            /* Re-read pins without changing direction so we see actual state */
            int sda_state = digitalRead(PIN_SDA);
            int scl_state = digitalRead(PIN_SCL);
            printf("[BB-I2C] ERROR: arbitration lost in %s "
                   "(SDA=GPIO%d reads %d, SCL=GPIO%d reads %d)\n",
                   where, PIN_SDA, sda_state, PIN_SCL, scl_state);
        } else {
            printf("[BB-I2C] ERROR: arbitration lost in %s\n", where);
        }
    }
}

static void i2c_sleep() {
    if (sleepTimeNanos)
#ifdef NO_NANOSLEEP
        usleep(sleepTimeNanos / 1000);
#else
        nanosleep(&nanoSleepTime, NULL);
#endif
}

static void i2c_delay() {
    volatile unsigned int index;
    for (index = 0; index < delayTicks; index++)
        ;
}

/* ---------- I2C protocol primitives ---------- */

static void i2c_start_cond() {
    if (i2c_started) {
        read_SDA();
        i2c_delay();
        while (read_SCL() == 0)
            i2c_sleep();
        i2c_delay();
    }
    if (read_SDA() == 0)
        arbitration_lost("i2c_start_cond");
    clear_SDA();
    i2c_delay();
    clear_SCL();
    i2c_started = 1;
}

static void i2c_stop_cond(void) {
    clear_SDA();
    i2c_delay();
    while (read_SCL() == 0)
        i2c_sleep();
    i2c_delay();
    read_SDA();
    if (read_SDA() == 0)
        arbitration_lost("i2c_stop_cond");
    i2c_delay();
    i2c_started = 0;
}

static void i2c_write_bit(int bit) {
    if (bit)
        read_SDA();
    else
        clear_SDA();
    i2c_delay();
    while (read_SCL() == 0)
        i2c_sleep();
    if (bit && read_SDA() == 0)
        arbitration_lost("i2c_write_bit");
    i2c_delay();
    clear_SCL();
}

static int i2c_read_bit() {
    int bit;
    read_SDA();
    i2c_delay();
    while (read_SCL() == 0)
        i2c_sleep();
    bit = read_SDA();
    i2c_delay();
    clear_SCL();
    return bit;
}

static int i2c_write_byte(int send_start, int send_stop, uint8_t byte) {
    unsigned bit;
    int nack;
    if (send_start)
        i2c_start_cond();
    for (bit = 0; bit < 8; bit++) {
        i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    nack = i2c_read_bit();
    if (send_stop)
        i2c_stop_cond();
    return nack;
}

static uint8_t i2c_read_byte(int nack, int send_stop) {
    unsigned char byte = 0;
    unsigned bit;
    for (bit = 0; bit < 8; bit++)
        byte = (byte << 1) | i2c_read_bit();
    i2c_write_bit(nack);
    if (send_stop)
        i2c_stop_cond();
    return byte;
}

/* ---------- public SMBus-style API ---------- */

/*
 * i2cbb_write_byte_data - write one register byte to a device.
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK | value | ACK | STOP
 *
 * Each of the three phases (address, register, value) produces a distinct
 * error message so you can tell exactly which byte the slave refused.
 */
int32_t i2cbb_write_byte_data(uint8_t i2c_address, uint8_t command, uint8_t value) {
    uint8_t address = (i2c_address << 1) | 0;  /* write flag = 0 */

    if (i2cbb_debug_level >= 2)
        printf("[BB-I2C] write_byte: addr=0x%02X reg=0x%02X val=0x%02X\n",
               i2c_address, command, value);

    if (!i2c_write_byte(1, 0, address)) {
        if (!i2c_write_byte(0, 0, command)) {
            if (!i2c_write_byte(0, 1, value)) {
                return 0;   /* success */
            }
            /* value byte NACK'd */
            if (i2cbb_debug_level >= 1)
                printf("[BB-I2C] ERROR: NACK on value byte "
                       "addr=0x%02X reg=0x%02X val=0x%02X\n",
                       i2c_address, command, value);
        } else {
            /* register/command byte NACK'd */
            if (i2cbb_debug_level >= 1)
                printf("[BB-I2C] ERROR: NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
        }
        i2c_stop_cond();
    } else {
        /* address NACK'd - device not present or bus fault */
        if (i2cbb_debug_level >= 1)
            printf("[BB-I2C] ERROR: NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        i2c_stop_cond();
    }

    return -1;
}

/*
 * i2cbb_read_byte_data - read one register byte from a device.
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK |
 *                START | addr+R | ACK | byte | NACK | STOP
 */
int32_t i2cbb_read_byte_data(uint8_t i2c_address, uint8_t command) {
    uint8_t address = (i2c_address << 1) | 0;

    if (i2cbb_debug_level >= 2)
        printf("[BB-I2C] read_byte: addr=0x%02X reg=0x%02X\n",
               i2c_address, command);

    if (!i2c_write_byte(1, 0, address)) {
        if (!i2c_write_byte(0, 0, command)) {
            address = (i2c_address << 1) | 1;  /* read flag = 1 */
            if (!i2c_write_byte(1, 0, address)) {
                uint8_t result = i2c_read_byte(1, 1);
                if (i2cbb_debug_level >= 2)
                    printf("[BB-I2C] read_byte: addr=0x%02X reg=0x%02X -> 0x%02X\n",
                           i2c_address, command, result);
                return result;
            } else {
                if (i2cbb_debug_level >= 1)
                    printf("[BB-I2C] ERROR: NACK on repeated-start read address "
                           "addr=0x%02X reg=0x%02X\n",
                           i2c_address, command);
                i2c_stop_cond();
            }
        } else {
            if (i2cbb_debug_level >= 1)
                printf("[BB-I2C] ERROR: NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
            i2c_stop_cond();
        }
    } else {
        if (i2cbb_debug_level >= 1)
            printf("[BB-I2C] ERROR: NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        i2c_stop_cond();
    }

    return -1;
}

/*
 * i2cbb_write_i2c_block_data - write a contiguous block of registers.
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK | val[0] | ACK | ... | STOP
 *
 * At level 2 the full byte array is printed before the transaction so you
 * can compare it with what a logic analyser sees on the wire.
 */
int32_t i2cbb_write_i2c_block_data(uint8_t i2c_address, uint8_t command,
                                    uint8_t length, const uint8_t *values) {
    uint8_t address = (i2c_address << 1) | 0;

    if (i2cbb_debug_level >= 2) {
        printf("[BB-I2C] block_write: addr=0x%02X reg=0x%02X len=%u data:",
               i2c_address, command, length);
        for (uint8_t k = 0; k < length; k++)
            printf(" %02X", values[k]);
        printf("\n");
    }

    if (!i2c_write_byte(1, 0, address)) {
        if (!i2c_write_byte(0, 0, command)) {
            int errors = 0;
            size_t i;
            for (i = 0; i < length; i++) {
                if (!errors)
                    errors = i2c_write_byte(0, 0, values[i]);
            }

            i2c_stop_cond();

            if (!errors)
                return 0;

            /* errors is non-zero: i still holds the failing index */
            if (i2cbb_debug_level >= 1)
                printf("[BB-I2C] ERROR: block_write NACK at byte index %zu "
                       "addr=0x%02X reg=0x%02X val=0x%02X\n",
                       i, i2c_address, command, values[i]);
        } else {
            i2c_stop_cond();
            if (i2cbb_debug_level >= 1)
                printf("[BB-I2C] ERROR: block_write NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
        }
    } else {
        i2c_stop_cond();
        if (i2cbb_debug_level >= 1)
            printf("[BB-I2C] ERROR: block_write NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
    }

    return -1;
}

/*
 * i2cbb_read_i2c_block_data - read a contiguous block of registers.
 *
 * Note: the original code skipped the write-pointer phase and went straight
 * to a read-addressed repeated start.  That works because Si5351 register
 * auto-increment starts from the last written address.  The error suppression
 * flag has been removed; errors always print at level >= 1.
 */
int32_t i2cbb_read_i2c_block_data(uint8_t i2c_address, uint8_t command,
                                   uint8_t length, uint8_t *values) {
    uint8_t address = (i2c_address << 1) | 1;  /* read flag = 1 */

    if (i2cbb_debug_level >= 2)
        printf("[BB-I2C] block_read: addr=0x%02X reg=0x%02X len=%u\n",
               i2c_address, command, length);

    if (i2c_write_byte(1, 0, address)) {
        i2c_stop_cond();
        if (i2cbb_debug_level >= 1)
            printf("[BB-I2C] ERROR: block_read NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        return -1;
    }

    uint8_t i;
    for (i = 0; i < length - 1; i++)
        values[i] = i2c_read_byte(0, 0);
    values[i] = i2c_read_byte(1, 1);

    i2c_stop_cond();

    if (i2cbb_debug_level >= 2) {
        printf("[BB-I2C] block_read: addr=0x%02X reg=0x%02X -> data:",
               i2c_address, command);
        for (i = 0; i < length; i++)
            printf(" %02X", values[i]);
        printf("\n");
    }

    return length;
}
