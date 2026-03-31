/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
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
 * @file      TouchDrvCSTXXX.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 *
 */
#include "TouchDrvCSTXXX.hpp"

TouchDrvCSTXXX::DriverCreator TouchDrvCSTXXX::driverCreators[TouchDrvCSTXXX::driverCreatorMaxNum] = {
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST226>(); },
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST816>(); },
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST92xx>(); },
    []() -> std::unique_ptr<TouchDrvInterface> { return std::make_unique<TouchDrvCST3530>(); }
};

TouchDrvCSTXXX::TouchDrvCSTXXX() : _touchType(TouchDrv_UNKNOWN) {}


void TouchDrvCSTXXX::setTouchDrvModel(TouchDrvType model)
{
    _touchType = model;
}

void TouchDrvCSTXXX::setupDriver()
{
    if (_drv) {
        _drv->setPins(_pinsCfg.rstPin, _pinsCfg.irqPin);
        _drv->setGpioCallback(_halModeCallback, _halWriteCallback, _halReadCallback);
    }
}

#if defined(ARDUINO)
bool TouchDrvCSTXXX::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    return beginImpl(wire, addr, sda, scl);
}
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
bool TouchDrvCSTXXX::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    return beginImpl(port_num, addr, sda, scl);
}
#else
bool TouchDrvCSTXXX::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    return beginImpl(handle, addr);
}
#endif  // ESP_PLATFORM
#endif  // ARDUINO


bool TouchDrvCSTXXX::begin(SensorCommCustom::CustomCallback callback,
                           SensorCommCustomHal::CustomHalCallback hal_callback,
                           uint8_t addr)
{
    return beginImpl(callback, hal_callback, addr);
}


void TouchDrvCSTXXX::reset()
{
    if (!_drv)return;
    _drv->reset();
}

const TouchPoints &TouchDrvCSTXXX::getTouchPoints()
{
    return _drv ? _drv->getTouchPoints() : _touchPoints;
}

bool TouchDrvCSTXXX::isPressed(uint32_t filter_ms)
{
    if (!_drv)return false;
    return _drv->isPressed(filter_ms);
}

const char *TouchDrvCSTXXX::getModelName()
{
    return _drv ? _drv->getModelName() : "NULL";
}

uint32_t TouchDrvCSTXXX::getChipID()
{
    return _drv ? _drv->getChipID() : 0;
}

void TouchDrvCSTXXX::sleep()
{
    if (!_drv)return;
    _drv->sleep();
}

void TouchDrvCSTXXX::wakeup()
{
    if (!_drv)return;
    _drv->wakeup();
}

uint8_t TouchDrvCSTXXX::getSupportTouchPoint()
{
    return _drv ? _drv->getSupportTouchPoint() : 0;
}

void TouchDrvCSTXXX::getResolution(uint16_t &x, uint16_t &y)
{
    if (!_drv)return;
    _drv->getResolution(x, y);
}


uint16_t TouchDrvCSTXXX::getResolutionY()
{
    return _drv ? _drv->getResolutionY() : 0;
}

uint16_t TouchDrvCSTXXX::getResolutionX()
{
    return _drv ? _drv->getResolutionX() : 0;
}


void TouchDrvCSTXXX::setCenterButtonCoordinate(int16_t x, int16_t y)
{
    if (!_drv)return ;
    if (_touchType == TouchDrv_CST8XX) {
        static_cast<TouchDrvCST816 *>(_drv.get())->setCenterButtonCoordinate(x, y);
    }
}

void TouchDrvCSTXXX::setHomeButtonCallback(TouchDrvInterface::HomeButtonCallback callback, void *user_data)
{
    if (!_drv)return ;
    if (_touchType == TouchDrv_CST8XX) {
        static_cast<TouchDrvCST816 *>(_drv.get())->setHomeButtonCallback(callback, user_data);
    } else if (_touchType == TouchDrv_CST226) {
        static_cast<TouchDrvCST226 *>(_drv.get())->setHomeButtonCallback(callback, user_data);
    }
}

void TouchDrvCSTXXX::disableAutoSleep()
{
    if (!_drv)return ;
    if (_touchType == TouchDrv_CST8XX) {
        static_cast<TouchDrvCST816 *>(_drv.get())->disableAutoSleep();
    }
}

void TouchDrvCSTXXX::enableAutoSleep()
{
    if (!_drv)return ;
    if (_touchType == TouchDrv_CST8XX) {
        static_cast<TouchDrvCST816 *>(_drv.get())->enableAutoSleep();
    }
}

void TouchDrvCSTXXX::setSwapXY(bool swap)
{
    if (!_drv)return ;
    _drv->setSwapXY(swap);
}

void TouchDrvCSTXXX::setMirrorXY(bool mirrorX, bool mirrorY)
{
    if (!_drv)return ;
    _drv->setMirrorXY(mirrorX, mirrorY);
}

void TouchDrvCSTXXX::setMaxCoordinates(uint16_t x, uint16_t y)
{
    if (!_drv)return ;
    _drv->setMaxCoordinates(x, y);
}
