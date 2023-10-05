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
#include "REG/CSTxxxConstants.h"
#include "touch/TouchClassCST226.h"
#include "touch/TouchClassCST816.h"

class TouchDrvCSTXXX
{
public:
    TouchDrvCSTXXX(): drv(NULL)
    {
    }

    ~TouchDrvCSTXXX()
    {
        if (drv) {
            delete drv;
            drv = NULL;
        }
    }

    void setPins(int rst, int irq)
    {
        _rst = rst;
        _irq = irq;
    }

    bool init(PLATFORM_WIRE_TYPE &wire,
              int sda,
              int scl,
              uint8_t address)
    {
        if (!drv) {
            drv = new TouchClassCST816(wire, sda, scl, address);
            drv->setPins(_rst, _irq);
            if (!drv->init()) {
                delete drv;
                drv = NULL;
            }
        }

        if (!drv) {
            drv = new TouchClassCST226(wire, sda, scl, address);
            drv->setPins(_rst, _irq);
            if (!drv->init()) {
                delete drv;
                drv = NULL;
            }
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

    void setHomeButtonCallback(home_button_callback_t callback, void *user_data = NULL)
    {
        if (!drv)return ;
        String model = drv->getModelName();
        if (model.startsWith("CST8")) {
            TouchClassCST816 *pT = static_cast<TouchClassCST816 *>(drv);
            pT->setHomeButtonCallback(callback, user_data);
            pT->setCenterButtonCoordinate(600, 120);  // Only suitable for AMOLED 1.91 inch

        } else if (model.startsWith("CST2")) {
            TouchClassCST226 *pT = static_cast<TouchClassCST226 *>(drv);
            pT->setHomeButtonCallback(callback, user_data);
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
    TouchDrvInterface *drv = NULL;
    int _irq;
    int _rst;
};



