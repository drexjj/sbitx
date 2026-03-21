/*
 * i2cBitBangingBus.h
 *
 *  Created on: 06.03.2015
 *      Author: "Marek Wyborski"
 *
 * Debug instrumentation added. Call setDebug(level) after construction:
 *   0 = silent (default)
 *   1 = errors only: NACKs, arbitration loss
 *   2 = every transaction: address, register, value
 *   3 = verbose: level 2 plus live SDA/SCL pin state at error points
 *
 * Output is prefixed [BB-I2C] to match the C i2cbb.c convention so the
 * same grep patterns work regardless of which driver is in use.
 */

#ifndef I2C_BIT_BANG_H_
#define I2C_BIT_BANG_H_

#include <stdint.h>
#include <unistd.h>
#include <ctime>
#include <climits>
#include <string>

using namespace std;

class i2cBitBangingBus
{
private:
    uint8_t PIN_SDA;
    uint8_t PIN_SCL;
    uint32_t sleepTimeNanos;
    struct timespec nanoSleepTime;
    uint32_t delayTicks;
    bool i2c_started;
    int debugLevel;

public:
    i2cBitBangingBus(uint8_t pin_number_sda, uint8_t pin_number_scl,
                     uint32_t sleepTimeNanos_ = 0, uint32_t delayTicks_ = 0);

    void setDebug(int level);

private:
    bool read_SCL();
    bool read_SDA();
    void clear_SCL();
    void clear_SDA();
    void arbitration_lost(string where);

    void i2c_sleep();
    void i2c_delay();

    void i2c_start_cond();
    void i2c_stop_cond();
    void i2c_write_bit(bool bit);
    bool i2c_read_bit();
    bool i2c_write_byte(bool send_start, bool send_stop, uint8_t byte);
    uint8_t i2c_read_byte(bool nack, bool send_stop);

public:
    int32_t i2c_smbus_write_byte_data(uint8_t i2c_address, uint8_t command, uint8_t value);
    int32_t i2c_smbus_read_byte_data(uint8_t i2c_address, uint8_t command);
    int32_t i2c_smbus_write_i2c_block_data(uint8_t i2c_address, uint8_t command,
                                            uint8_t length, const uint8_t *values);
    int32_t i2c_smbus_read_i2c_block_data(uint8_t i2c_address, uint8_t command,
                                           uint8_t length, uint8_t *values);
};

#endif /* I2C_BIT_BANG_H_ */
