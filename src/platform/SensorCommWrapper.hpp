/**
 *
 * @license MIT License
 *
 * Copyright (c) 2026 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      SensorCommWrapper.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-29
 *
 */
#pragma once
#include "SensorErr.hpp"
#include "SensorCommInterface.hpp"
#include "SensorCommBase.hpp"

class SensorCommWrapper : public SensorCommInterface
{
public:
    SensorCommWrapper() = default;

    int readReg(uint8_t reg) const
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->readRegister(reg);
    }

    int readRegBuff(uint8_t reg, uint8_t *buf, size_t len) const
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->readRegister(reg, buf, len);
    }

    int readBuff(uint8_t *buf, size_t len) const
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->readBuffer(buf, len);
    }

    int writeReg(uint8_t reg, uint8_t val)
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->writeRegister(reg, val);
    }

    int writeRegBuff(uint8_t reg, uint8_t *buf, size_t len)
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->writeRegister(reg, buf, len);
    }

    int writeBuff(uint8_t *buf, size_t len)
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->writeBuffer(buf, len);
    }

    int writeThenRead(const uint8_t *write_buffer, size_t write_len,
                      uint8_t *read_buffer, size_t read_len)
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->writeThenRead(write_buffer, write_len, read_buffer, read_len);
    }

    bool setRegBit(uint8_t reg, uint8_t bit)
    {
        if (!ensureValid()) return false;
        return getComm()->setRegisterBit(reg, bit);
    }

    bool clrRegBit(uint8_t reg, uint8_t bit)
    {
        if (!ensureValid()) return false;
        return getComm()->clrRegisterBit(reg, bit);
    }

    bool getRegBit(uint8_t reg, uint8_t bit)
    {
        if (!ensureValid()) return false;
        return getComm()->getRegisterBit(reg, bit);
    }

    int updateBits(uint8_t reg, uint8_t mask, uint8_t value_shifted)
    {
        if (!ensureValid()) return SENSOR_ERR_NOT_INITIALIZED;
        return getComm()->updateBits(reg, mask, value_shifted);
    }

    void setAddress(uint8_t address)
    {
        if (!ensureValid()) return;
        I2CParam params(I2CParam::I2C_SET_ADDR, address);
        getComm()->setParams(params);
    }

    void setAck(bool enable)
    {
        if (!ensureValid()) return;
        I2CParam params(I2CParam::I2C_SET_FLAG, enable);
        getComm()->setParams(params);
    }
};
