/*
 * i2cBitBangingBus.cpp
 *
 *  Created on: 06.03.2015
 *      Author: "Marek Wyborski"
 *
 * Debug instrumentation added. Call setDebug(level) after construction.
 * All output uses the [BB-I2C] prefix — same as the C i2cbb.c driver —
 * so grep patterns are identical regardless of which version is linked.
 *
 * The original arbitration_lost() threw a runtime_error, which silently
 * swallowed the location string unless the caller caught and printed it.
 * At debug level >= 1 it now prints before throwing so the message always
 * reaches the console even if the exception is caught higher up.
 */

#include "i2cBitBangingBus.h"

#include <wiringPi.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <cstdio>

using namespace std;

i2cBitBangingBus::i2cBitBangingBus(uint8_t pin_number_sda, uint8_t pin_number_scl,
                                     uint32_t sleepTimeNanos_, uint32_t delayTicks_)
    : PIN_SDA(pin_number_sda), PIN_SCL(pin_number_scl),
      sleepTimeNanos(sleepTimeNanos_), nanoSleepTime(),
      delayTicks(delayTicks_), i2c_started(false), debugLevel(0)
{
    nanoSleepTime.tv_sec  = 0;
    nanoSleepTime.tv_nsec = 1;
}

void i2cBitBangingBus::setDebug(int level)
{
    debugLevel = level;
    printf("[BB-I2C] debug level set to %d (SDA=GPIO%d SCL=GPIO%d delayTicks=%u)\n",
           level, PIN_SDA, PIN_SCL, delayTicks);
}

/* ---------- low-level GPIO ---------- */

bool i2cBitBangingBus::read_SCL()
{
    pinMode(PIN_SCL, INPUT);
    return digitalRead(PIN_SCL);
}

bool i2cBitBangingBus::read_SDA()
{
    pinMode(PIN_SDA, INPUT);
    return digitalRead(PIN_SDA);
}

void i2cBitBangingBus::clear_SCL()
{
    pinMode(PIN_SCL, OUTPUT);
    digitalWrite(PIN_SCL, 0);
}

void i2cBitBangingBus::clear_SDA()
{
    pinMode(PIN_SDA, OUTPUT);
    digitalWrite(PIN_SDA, 0);
}

/*
 * arbitration_lost - print before throwing so the location is never lost
 * even if the caller catches the exception without logging it.
 * At level 3 we also sample the actual pin states so you can tell whether
 * the bus is stuck low (held by a slave or wiring fault) vs. a race.
 */
void i2cBitBangingBus::arbitration_lost(string where)
{
    if (debugLevel >= 1) {
        if (debugLevel >= 3) {
            int sda_state = digitalRead(PIN_SDA);
            int scl_state = digitalRead(PIN_SCL);
            printf("[BB-I2C] ERROR: arbitration lost in %s "
                   "(SDA=GPIO%d reads %d, SCL=GPIO%d reads %d)\n",
                   where.c_str(), PIN_SDA, sda_state, PIN_SCL, scl_state);
        } else {
            printf("[BB-I2C] ERROR: arbitration lost in %s\n", where.c_str());
        }
    }
    throw runtime_error("[BB-I2C] connection lost: " + where);
}

/* ---------- timing ---------- */

void i2cBitBangingBus::i2c_sleep()
{
    if (sleepTimeNanos)
#ifdef NO_NANOSLEEP
        usleep(sleepTimeNanos / 1000);
#else
        nanosleep(&nanoSleepTime, NULL);
#endif
}

void i2cBitBangingBus::i2c_delay()
{
    unsigned int index;
    for (index = 0; index < delayTicks; index++)
        ;
}

/* ---------- I2C protocol primitives ---------- */

void i2cBitBangingBus::i2c_start_cond()
{
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
    i2c_started = true;
}

void i2cBitBangingBus::i2c_stop_cond()
{
    clear_SDA();
    i2c_delay();
    while (read_SCL() == 0)
        i2c_sleep();
    i2c_delay();
    read_SDA();
    if (read_SDA() == 0)
        arbitration_lost("i2c_stop_cond");
    i2c_delay();
    i2c_started = false;
}

void i2cBitBangingBus::i2c_write_bit(bool bit)
{
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

bool i2cBitBangingBus::i2c_read_bit()
{
    bool bit;
    read_SDA();
    i2c_delay();
    while (read_SCL() == 0)
        i2c_sleep();
    bit = read_SDA();
    i2c_delay();
    clear_SCL();
    return bit;
}

bool i2cBitBangingBus::i2c_write_byte(bool send_start, bool send_stop, uint8_t byte)
{
    unsigned bit;
    bool nack;
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

uint8_t i2cBitBangingBus::i2c_read_byte(bool nack, bool send_stop)
{
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
 * i2c_smbus_write_byte_data
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK | value | ACK | STOP
 *
 * Each of the three phases gets a distinct error message so you can tell
 * exactly which byte the slave refused.
 */
int32_t i2cBitBangingBus::i2c_smbus_write_byte_data(uint8_t i2c_address,
                                                      uint8_t command,
                                                      uint8_t value)
{
    uint8_t address = (i2c_address << 1) | 0;

    if (debugLevel >= 2)
        printf("[BB-I2C] write_byte: addr=0x%02X reg=0x%02X val=0x%02X\n",
               i2c_address, command, value);

    if (!i2c_write_byte(true, false, address)) {
        if (!i2c_write_byte(false, false, command)) {
            if (!i2c_write_byte(false, true, value)) {
                return 0;
            }
            if (debugLevel >= 1)
                printf("[BB-I2C] ERROR: NACK on value byte "
                       "addr=0x%02X reg=0x%02X val=0x%02X\n",
                       i2c_address, command, value);
        } else {
            if (debugLevel >= 1)
                printf("[BB-I2C] ERROR: NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
            i2c_stop_cond();
        }
    } else {
        if (debugLevel >= 1)
            printf("[BB-I2C] ERROR: NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        i2c_stop_cond();
    }

    return -1;
}

/*
 * i2c_smbus_read_byte_data
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK |
 *                START | addr+R | ACK | byte | NACK | STOP
 */
int32_t i2cBitBangingBus::i2c_smbus_read_byte_data(uint8_t i2c_address,
                                                     uint8_t command)
{
    uint8_t address = (i2c_address << 1) | 0;

    if (debugLevel >= 2)
        printf("[BB-I2C] read_byte: addr=0x%02X reg=0x%02X\n",
               i2c_address, command);

    if (!i2c_write_byte(true, false, address)) {
        if (!i2c_write_byte(false, false, command)) {
            address = (i2c_address << 1) | 1;
            if (!i2c_write_byte(true, false, address)) {
                uint8_t result = i2c_read_byte(true, true);
                if (debugLevel >= 2)
                    printf("[BB-I2C] read_byte: addr=0x%02X reg=0x%02X -> 0x%02X\n",
                           i2c_address, command, result);
                return result;
            } else {
                if (debugLevel >= 1)
                    printf("[BB-I2C] ERROR: NACK on repeated-start read address "
                           "addr=0x%02X reg=0x%02X\n",
                           i2c_address, command);
                i2c_stop_cond();
            }
        } else {
            if (debugLevel >= 1)
                printf("[BB-I2C] ERROR: NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
            i2c_stop_cond();
        }
    } else {
        if (debugLevel >= 1)
            printf("[BB-I2C] ERROR: NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        i2c_stop_cond();
    }

    return -1;
}

/*
 * i2c_smbus_write_i2c_block_data
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK | val[0..n] | ACK each | STOP
 *
 * At level 2 the full byte array is printed before the transaction so you
 * can compare it against a logic analyser capture.
 */
int32_t i2cBitBangingBus::i2c_smbus_write_i2c_block_data(uint8_t i2c_address,
                                                           uint8_t command,
                                                           uint8_t length,
                                                           const uint8_t *values)
{
    uint8_t address = (i2c_address << 1) | 0;

    if (debugLevel >= 2) {
        printf("[BB-I2C] block_write: addr=0x%02X reg=0x%02X len=%u data:",
               i2c_address, command, length);
        for (uint8_t k = 0; k < length; k++)
            printf(" %02X", values[k]);
        printf("\n");
    }

    if (!i2c_write_byte(true, false, address)) {
        if (!i2c_write_byte(false, false, command)) {
            bool errors = false;
            size_t failed_at = 0;
            for (size_t i = 0; i < length; i++) {
                if (!errors) {
                    errors = i2c_write_byte(false, false, values[i]);
                    if (errors)
                        failed_at = i;
                }
            }
            i2c_stop_cond();
            if (!errors)
                return 0;
            if (debugLevel >= 1)
                printf("[BB-I2C] ERROR: block_write NACK at byte index %zu "
                       "addr=0x%02X reg=0x%02X val=0x%02X\n",
                       failed_at, i2c_address, command, values[failed_at]);
        } else {
            if (debugLevel >= 1)
                printf("[BB-I2C] ERROR: block_write NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
            i2c_stop_cond();
        }
    } else {
        if (debugLevel >= 1)
            printf("[BB-I2C] ERROR: block_write NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        i2c_stop_cond();
    }

    return -1;
}

/*
 * i2c_smbus_read_i2c_block_data
 *
 * Wire sequence: START | addr+W | ACK | reg | ACK |
 *                START | addr+R | ACK | val[0..n-1] ACK each | val[n] NACK | STOP
 *
 * At level 2 the returned bytes are printed after the transaction.
 */
int32_t i2cBitBangingBus::i2c_smbus_read_i2c_block_data(uint8_t i2c_address,
                                                          uint8_t command,
                                                          uint8_t length,
                                                          uint8_t *values)
{
    uint8_t address = (i2c_address << 1) | 0;

    if (debugLevel >= 2)
        printf("[BB-I2C] block_read: addr=0x%02X reg=0x%02X len=%u\n",
               i2c_address, command, length);

    if (!i2c_write_byte(true, false, address)) {
        if (!i2c_write_byte(false, false, command)) {
            address = (i2c_address << 1) | 1;
            if (!i2c_write_byte(true, false, address)) {
                for (uint8_t i = 0; i < length; i++)
                    values[i] = i2c_read_byte(i == (length - 1),
                                              i == (length - 1));

                if (debugLevel >= 2) {
                    printf("[BB-I2C] block_read: addr=0x%02X reg=0x%02X -> data:",
                           i2c_address, command);
                    for (uint8_t i = 0; i < length; i++)
                        printf(" %02X", values[i]);
                    printf("\n");
                }

                return length;
            } else {
                if (debugLevel >= 1)
                    printf("[BB-I2C] ERROR: block_read NACK on repeated-start "
                           "read address addr=0x%02X reg=0x%02X\n",
                           i2c_address, command);
                i2c_stop_cond();
            }
        } else {
            if (debugLevel >= 1)
                printf("[BB-I2C] ERROR: block_read NACK on register byte "
                       "addr=0x%02X reg=0x%02X\n",
                       i2c_address, command);
            i2c_stop_cond();
        }
    } else {
        if (debugLevel >= 1)
            printf("[BB-I2C] ERROR: block_read NACK on address byte "
                   "addr=0x%02X (wire byte=0x%02X) - device missing?\n",
                   i2c_address, address);
        i2c_stop_cond();
    }

    return -1;
}


// Uncomment to compile with main method to test MPU 9X50
//#define TEST_MPU9250 1

#ifdef TEST_MPU9250
#include <signal.h>
#include <stdlib.h>

#define MPU_9150_I2C_ADDRESS_1   0x69
#define MPU_9150_I2C_ADDRESS_2   0x68
#define MPU_9150_SMPRT_DIV       0x19
#define MPU_9150_ACCEL_XOUT_H    0x3B
#define MPU_9150_GYRO_XOUT_H     0x43
#define I2C_AUTO_INCREMENT       0x80

bool stopI2c = false;

void handleSigInt(int param)
{
    cout << "CTRL-C" << endl;
    stopI2c = true;
}

float valueToFloat(int16_t value)
{
    if (value >= 0)
        return static_cast<float>(value) / static_cast<float>(SHRT_MAX);
    else
        return static_cast<float>(value) / static_cast<float>(-SHRT_MIN);
}

int main(void)
{
    signal(SIGINT, handleSigInt);
    wiringPiSetup();

    auto p0_p2 = make_shared<i2cBitBangingBus>(0, 2, 0);
    auto p8_p9 = make_shared<i2cBitBangingBus>(8, 9, 0);

    /* Enable debug on both buses for testing */
    p0_p2->setDebug(2);
    p8_p9->setDebug(2);

    uint64_t count = 0;
    uint8_t block[6];
    auto address = MPU_9150_I2C_ADDRESS_1;
    auto bus = p0_p2;

    while (!stopI2c) {
        count++;
        try {
            if      (count % 4 == 0) { cout << "\nBUS0_2 ADD1: "; address = MPU_9150_I2C_ADDRESS_1; bus = p0_p2; }
            else if (count % 4 == 1) { cout << "BUS8_9 ADD1: "; address = MPU_9150_I2C_ADDRESS_1; bus = p8_p9; }
            else if (count % 4 == 2) { cout << "BUS0_2 ADD2: "; address = MPU_9150_I2C_ADDRESS_2; bus = p0_p2; }
            else                     { cout << "BUS8_9 ADD2: "; address = MPU_9150_I2C_ADDRESS_2; bus = p8_p9; }

            if (bus->i2c_smbus_write_byte_data(address, MPU_9150_SMPRT_DIV, 0) < 0)
                throw runtime_error("I2C Write Error");

            float acc_scale  = 2.0f;
            float gyro_scale = 250.0f;

            if (bus->i2c_smbus_read_i2c_block_data(address,
                    I2C_AUTO_INCREMENT | MPU_9150_ACCEL_XOUT_H, 6, block) != 6)
                throw runtime_error("i2c_smbus_read_i2c_block_data Error");

            cout << "ax: " << valueToFloat((int16_t)(block[0] << 8 | block[1])) * acc_scale
                 << " ay: " << valueToFloat((int16_t)(block[2] << 8 | block[3])) * acc_scale
                 << " az: " << valueToFloat((int16_t)(block[4] << 8 | block[5])) * acc_scale
                 << endl;
        }
        catch (exception &ex) {
            cout << "EX: " << ex.what() << endl;
        }
    }
    cout << count << endl;
    return 0;
}
#endif
