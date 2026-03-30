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
 * @file      DeviceBeginCommon.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-10
 *
 */
#pragma once
#include "../../SensorPlatform.hpp"
#include "../SensorCommWrapper.hpp"

class DeviceBeginCommon : public SensorCommWrapper
{
public:
    virtual ~DeviceBeginCommon()
    {
        if (comm) {
            comm->deinit();
        }
    }

    uint8_t getAddress() const { return _addr; }
    uint8_t getInterface() const { return _iface; }

protected:
    DeviceBeginCommon() = default;

    virtual bool initImpl(uint8_t param) = 0;

    virtual bool ensureValid() const override
    {
        if (!comm) return false;
        return true;
    }

    virtual SensorCommBase* getComm() const override
    {
        return comm.get();
    }

    // ---- Hooks functions  ----
    
    // 1) Before comm/hal/staticComm is created (some parameters can be prepared).
    virtual void beforeBegin() {}

    // 2) comm/hal/staticComm has been created + comm->init() has been completed, 
    //    but initImpl has not yet run
    //    Typical use case: Touch driver sets hal->setCustomMode/Read/Write
    virtual void afterCommReady() {}

    // 3) initImpl has succeeded (default register configuration, cache reset, etc. can be done here)
    virtual void afterInitSuccess(uint8_t param) {}

    // 4) If any begin operation fails (error code can be logged).
    virtual void onBeginFail() {}

    // 5) Unified fail handling to ensure consistent error paths for each begin branch
    bool fail()
    {
        onBeginFail();
        return false;
    }

protected:
    std::unique_ptr<SensorCommBase>   comm;
    std::unique_ptr<SensorHal>        hal;
    std::unique_ptr<SensorCommStatic> staticComm;

    uint8_t _addr  = 0;
    uint8_t _iface = COMM_CUSTOM;
};
