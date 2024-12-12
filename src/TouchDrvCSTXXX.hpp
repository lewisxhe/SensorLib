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
 * @file      TouchDrvCSTXXX.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 * @date      last 2023-10-05
 *
 */
#pragma once

#include "REG/CSTxxxConstants.h"
#include "touch/TouchClassCST226.h"
#include "touch/TouchClassCST816.h"
#include "touch/TouchDrvCST92xx.h"
#include "SensorCommon.tpp"

enum TouchDrvType {
    TouchDrv_UNKOWN,
    TouchDrv_CST8XX,
    TouchDrv_CST226,
    TouchDrv_CST92XX,
};

#if defined(ARDUINO)
template<typename CST_TouchClass>
CST_TouchClass *newTouchClassArduino(PLATFORM_WIRE_TYPE &wire,
                                     uint8_t address,
                                     int sda,
                                     int scl,
                                     int rst,
                                     int irq,
                                     gpio_write_fptr_t set_level_ptr,
                                     gpio_read_fptr_t get_level_ptr,
                                     gpio_mode_fptr_t set_mode_ptr
                                    )
{
    CST_TouchClass *ptr = new CST_TouchClass();
    ptr->setGpioCallback(set_mode_ptr, set_level_ptr, get_level_ptr);
    ptr->setPins(rst, irq);
    if (!ptr->begin(wire, address, sda, scl)) {
        delete ptr;
        ptr = NULL;
    }
    return ptr;
}

#elif defined(ESP_PLATFORM)

#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))

template<typename CST_TouchClass>
CST_TouchClass *newTouchClassIDF(i2c_master_bus_handle_t i2c_dev_bus_handle,
                                 uint8_t addr,
                                 int rst,
                                 int irq,
                                 gpio_write_fptr_t set_level_ptr,
                                 gpio_read_fptr_t get_level_ptr,
                                 gpio_mode_fptr_t set_mode_ptr)
{
    CST_TouchClass *ptr = new CST_TouchClass();
    ptr->setGpioCallback(set_mode_ptr, set_level_ptr, get_level_ptr);
    ptr->setPins(rst, irq);
    if (!ptr->begin(i2c_dev_bus_handle, addr)) {
        delete ptr;
        ptr = NULL;
    }
    return ptr;
}

#else   /*ESP_IDF_VERSION*/

template<typename CST_TouchClass>
CST_TouchClass *newTouchClassIDF(i2c_port_t port_num,
                                 uint8_t addr,
                                 int sda,
                                 int scl,
                                 int rst,
                                 int irq,
                                 gpio_write_fptr_t set_level_ptr,
                                 gpio_read_fptr_t get_level_ptr,
                                 gpio_mode_fptr_t set_mode_ptr)
{
    CST_TouchClass *ptr = new CST_TouchClass();
    ptr->setGpioCallback(set_mode_ptr, set_level_ptr, get_level_ptr);
    ptr->setPins(rst, irq);
    if (!ptr->begin(port_num, addr, sda, scl)) {
        delete ptr;
        ptr = NULL;
    }
    return ptr;
}

#endif  /*ESP_IDF_VERSION*/
#endif /*ESP_PLATFORM*/

template<typename CST_TouchClass>
CST_TouchClass *newTouchClassCallFunction(uint8_t address,
        iic_fptr_t readRegCallback,
        iic_fptr_t writeRegCallback,
        int rst,
        int irq,
        gpio_write_fptr_t set_level_ptr,
        gpio_read_fptr_t get_level_ptr,
        gpio_mode_fptr_t set_mode_ptr)
{
    CST_TouchClass *ptr = new CST_TouchClass();
    ptr->setGpioCallback(set_mode_ptr, set_level_ptr, get_level_ptr);
    ptr->setPins(rst, irq);
    if (!ptr->begin(address, readRegCallback, writeRegCallback)) {
        delete ptr;
        ptr = NULL;
    }
    return ptr;
}




class TouchDrvCSTXXX : public TouchDrvInterface
{
public:
    TouchDrvCSTXXX(): drv(NULL), __touch_type(TouchDrv_UNKOWN)
    {
    }

    ~TouchDrvCSTXXX()
    {
        if (drv) {
            delete drv;
            drv = NULL;
        }
    }

    void setTouchDrvModel(TouchDrvType model)
    {
        __touch_type = model;
    }

#if defined(ARDUINO)
    bool begin(PLATFORM_WIRE_TYPE &wire,
               uint8_t address,
               int sda,
               int scl)
    {
        if (__touch_type == TouchDrv_UNKOWN) {
            drv = newTouchClassArduino<TouchClassCST816>(wire, address, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST8XX;
                return true;
            }
            drv = newTouchClassArduino<TouchClassCST226>(wire, address, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST226;
                return true;
            }
            drv = newTouchClassArduino<TouchDrvCST92xx>(wire, address, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST92XX;
                return true;
            }
        } else if (__touch_type == TouchDrv_CST8XX) {
            drv = newTouchClassArduino<TouchClassCST816>(wire, address, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST226) {
            drv = newTouchClassArduino<TouchClassCST226>(wire, address, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST92XX) {
            drv = newTouchClassArduino<TouchDrvCST92xx>(wire, address, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        }
        if (!drv) {
            __touch_type = TouchDrv_UNKOWN;
            return true;
        }
        return drv != NULL;
    }
#elif defined(ESP_PLATFORM)
#if ((ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)) && defined(CONFIG_SENSORLIB_ESP_IDF_NEW_API))
    bool begin(i2c_master_bus_handle_t i2c_dev_bus_handle, uint8_t addr)
    {
        if (__touch_type == TouchDrv_UNKOWN) {
            drv = newTouchClassIDF<TouchClassCST816>(i2c_dev_bus_handle, addr, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST8XX;
                return true;
            }
            drv = newTouchClassIDF<TouchClassCST226>(i2c_dev_bus_handle, addr, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST226;
                return true;
            }
            drv = newTouchClassIDF<TouchDrvCST92xx>(i2c_dev_bus_handle, addr, __rst, __irq,
                                                    __set_gpio_level, __get_gpio_level,
                                                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST92XX;
                return true;
            }
        } else if (__touch_type == TouchDrv_CST8XX) {
            drv = newTouchClassIDF<TouchClassCST816>(i2c_dev_bus_handle, addr, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST226) {
            drv = newTouchClassIDF<TouchClassCST226>(i2c_dev_bus_handle, addr, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST92XX) {
            drv = newTouchClassIDF<TouchDrvCST92xx>(i2c_dev_bus_handle, addr, __rst, __irq,
                                                    __set_gpio_level, __get_gpio_level,
                                                    __set_gpio_mode);
        }
        if (!drv) {
            __touch_type = TouchDrv_UNKOWN;
            return true;
        }
        return drv != NULL;
    }
#else
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
    {

        if (__touch_type == TouchDrv_UNKOWN) {
            drv = newTouchClassIDF<TouchClassCST816>(port_num, addr, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST8XX;
                return true;
            }
            drv = newTouchClassIDF<TouchClassCST226>(port_num, addr, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST226;
                return true;
            }
            drv = newTouchClassIDF<TouchDrvCST92xx>(port_num, addr, sda, scl, __rst, __irq,
                                                    __set_gpio_level, __get_gpio_level,
                                                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST92XX;
                return true;
            }
        } else if (__touch_type == TouchDrv_CST8XX) {
            drv = newTouchClassIDF<TouchClassCST816>(port_num, addr, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST226) {
            drv = newTouchClassIDF<TouchClassCST226>(port_num, addr, sda, scl, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST92XX) {
            drv = newTouchClassIDF<TouchDrvCST92xx>(port_num, addr, sda, scl, __rst, __irq,
                                                    __set_gpio_level, __get_gpio_level,
                                                    __set_gpio_mode);
        }
        if (!drv) {
            __touch_type = TouchDrv_UNKOWN;
            return true;
        }
        return drv != NULL;
    }
#endif //ESP_IDF_VERSION
#endif//ARDUINO


    void setGpioCallback(gpio_mode_fptr_t mode_cb,
                         gpio_write_fptr_t write_cb,
                         gpio_read_fptr_t read_cb)
    {
        __set_gpio_level = write_cb;
        __get_gpio_level = read_cb;
        __set_gpio_mode = mode_cb;
    }

    bool begin(uint8_t address, iic_fptr_t readRegCallback, iic_fptr_t writeRegCallback)
    {
        if (__touch_type == TouchDrv_UNKOWN) {
            drv = newTouchClassCallFunction<TouchClassCST816>(address, readRegCallback, writeRegCallback, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST8XX;
                return true;
            }
            drv = newTouchClassCallFunction<TouchClassCST226>(address, readRegCallback, writeRegCallback, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST226;
                return true;
            }
            drv = newTouchClassCallFunction<TouchDrvCST92xx>(address, readRegCallback, writeRegCallback, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
            if (drv) {
                __touch_type = TouchDrv_CST92XX;
                return true;
            }
        } else if (__touch_type == TouchDrv_CST8XX) {
            drv = newTouchClassCallFunction<TouchClassCST816>(address, readRegCallback, writeRegCallback, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST226) {
            drv = newTouchClassCallFunction<TouchClassCST226>(address, readRegCallback, writeRegCallback, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        } else if (__touch_type == TouchDrv_CST92XX) {
            drv = newTouchClassCallFunction<TouchDrvCST92xx>(address, readRegCallback, writeRegCallback, __rst, __irq,
                    __set_gpio_level, __get_gpio_level,
                    __set_gpio_mode);
        }
        if (!drv) {
            __touch_type = TouchDrv_UNKOWN;
            return true;
        }
        return drv != NULL;
    }


    void reset()
    {
        if (!drv)return;
        drv->reset();
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)
    {
        if (!drv)return 0;
        return drv->getPoint(x_array, y_array, get_point);
    }

    bool isPressed()
    {
        if (!drv)return false;
        return drv->isPressed();
    }

    const char *getModelName()
    {
        if (!drv)return "NULL";
        return drv->getModelName();
    }

    void sleep()
    {
        if (!drv)return;
        drv->sleep();
    }

    void wakeup()
    {
        if (!drv)return;
        drv->reset();
    }

    void idle()
    {
        if (!drv)return;
        drv->idle();
    }

    uint8_t getSupportTouchPoint()
    {
        if (!drv)return 0;
        return drv->getSupportTouchPoint();
    }

    bool getResolution(int16_t *x, int16_t *y)
    {
        if (!drv)return false;
        return drv->getResolution(x, y);
    }

    void setCenterButtonCoordinate(uint16_t x, uint16_t y)
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->setCenterButtonCoordinate(x, y);
        }
    }

    void setHomeButtonCallback(home_button_callback_t callback, void *user_data = NULL)
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->setHomeButtonCallback(callback, user_data);
            // pT->setCenterButtonCoordinate(600, 120);  // Only suitable for AMOLED 1.91 inch

        } if (strncmp(model, "CST2", 3) == 0) {
            TouchClassCST226 *pT = static_cast<TouchClassCST226 *>(drv);
            pT->setHomeButtonCallback(callback, user_data);
        }
    }

    void disableAutoSleep()
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->disableAutoSleep();
        }
    }

    void enableAutoSleep()
    {
        if (!drv)return ;
        const char *model = drv->getModelName();
        if (strncmp(model, "CST8", 3) == 0) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->enableAutoSleep();
        }
    }

    void setSwapXY(bool swap)
    {
        if (!drv)return ;
        drv->setSwapXY(swap);
    }

    void setMirrorXY(bool mirrorX, bool mirrorY)
    {
        if (!drv)return ;
        drv->setMirrorXY(mirrorX, mirrorY);
    }

    void setMaxCoordinates(uint16_t x, uint16_t y)
    {
        if (!drv)return ;
        drv->setMaxCoordinates(x, y);
    }

private:
    gpio_write_fptr_t   __set_gpio_level        = NULL;
    gpio_read_fptr_t    __get_gpio_level        = NULL;
    gpio_mode_fptr_t    __set_gpio_mode         = NULL;
    TouchDrvInterface *drv = NULL;
    TouchDrvType        __touch_type;
};



