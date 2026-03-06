#include "TouchDrvHI8561.hpp"

void TouchDrvHI8561::reset()
{
    if (_rst != -1) {
        hal->digitalWrite(_rst, LOW);
        hal->delay(3);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(5);
    }
}

void TouchDrvHI8561::sleep()
{
}

void TouchDrvHI8561::wakeup()
{
    reset();
}

const TouchPoints &TouchDrvHI8561::getTouchPoints()
{
    static TouchPoints points;

    points.clear(); // Clear cached touch points

    uint8_t buffer[53] = {0};

    if (makePacketThenRead(TOUCH_INFO_START_ADDR, buffer, arraySize(buffer))) {

        uint8_t numPoints = buffer[0];
        if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
            return points;
        }

        for (uint8_t i = 0; i < numPoints; i++) {
            const uint8_t idx = BYTES_OFFSET + i * BYTES_PER_POINT;
            uint16_t x = buffer[idx + 1] | (buffer[idx + 0] << 8);
            uint16_t y = buffer[idx + 3] | (buffer[idx + 2] << 8);
            uint8_t  pressure = buffer[idx + 4];
            // Edge trigger detection
            if (x == UINT16_MAX || y == UINT16_MAX) {
                log_e("Edge trigger detected, id %d set to invalid coordinates", i);
                break;
            }
            points.addPoint(x, y, pressure, i);
        }
        // Swap XY or mirroring coordinates,if set
        updateXY(points);
    }
    return points;
}

bool TouchDrvHI8561::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    }
    return getTouchPoints().hasPoints();
}

const char *TouchDrvHI8561::getModelName()
{
    return "HI8561";
}

bool TouchDrvHI8561::initImpl(uint8_t addr)
{

    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    reset();

    // Disable I2C stop bit
    I2CParam params(I2CParam::I2C_SET_FLAG, false);
    comm->setParams(params);

    // Check if the starting address matches
    uint8_t buffer[6];
    if (!makePacketThenRead(SECTION_INFO_START_ADDR + 8, buffer, sizeof(buffer))) {
        return false;
    }
    uint32_t reg = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
    if (reg != TOUCH_INFO_START_ADDR) {
        log_e("Invalid touch info start address: 0x%08" PRIu32, reg);
        return false;
    }

    // Use fixed values ​​to identify chip models.
    _chipID = 0x8561;

    log_d("HI8561 touch start address: 0x%08" PRIu32, reg);

    return true;
}
