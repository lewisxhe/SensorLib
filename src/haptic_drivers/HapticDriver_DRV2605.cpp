/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      HapticDriver_DRV2605.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-04-03
 */
#include "HapticDriver_DRV2605.hpp"
#include "HapticDriver_DRV2605_Reg.hpp"

HapticDriver_DRV2605::HapticDriver_DRV2605()
    : HapticBase("DRV2605")
    , _currentLibrary(1)
    , _waveformCount(0)
    , _isERM(true)
    , _isStandby(false)
{
    for (int i = 0; i < 8; i++) {
        _waveformSeq[i] = 0;
    }
}

HapticDriver_DRV2605::~HapticDriver_DRV2605()
{
    end();
}

void HapticDriver_DRV2605::end()
{
    stop();
    setReady(false);
}

bool HapticDriver_DRV2605::isReady() const
{
    return _isReady;
}

const char *HapticDriver_DRV2605::getChipName() const
{
    return "DRV2605";
}

bool HapticDriver_DRV2605::run()
{
    if (_isStandby) {
        // Standby can't run
        return false;
    }
    return writeReg(DRV2605_REG_GO, 1) == 0;
}

bool HapticDriver_DRV2605::stop()
{
    return writeReg(DRV2605_REG_GO, 0) == 0;
}

bool HapticDriver_DRV2605::isPlaying() const
{
    int value = readReg(DRV2605_REG_GO);
    return value > 0 && value != -1;
}

HapticStatus HapticDriver_DRV2605::getStatus() const
{
    if (_isStandby) return HapticStatus::STANDBY;
    if (!_isReady) return HapticStatus::ERROR;
    if (isPlaying()) return HapticStatus::PLAYING;
    return HapticStatus::IDLE;
}

void HapticDriver_DRV2605::emergencyStop()
{
    stop();
}

bool HapticDriver_DRV2605::setGain(uint8_t gain)
{
    // DRV2605 doesn't have a dedicated gain register
    // For ROM library effects, the intensity is pre-defined
    // We can only use RTP mode for direct amplitude control
    // Store the value for reference, but return false to indicate
    // this chip doesn't support runtime gain adjustment
    (void)gain;
    return false;
}

uint8_t HapticDriver_DRV2605::getGain() const
{
    // DRV2605 doesn't have a gain register for ROM effects
    // Return stored value or 0 to indicate not applicable
    return 0;
}

bool HapticDriver_DRV2605::playEffect(HapticEffectId effect)
{
    if (!_isReady) return false;
    if (_isStandby)return false;
    setWaveform(0, static_cast<uint8_t>(effect));
    setWaveform(1, 0);
    return run();
}

bool HapticDriver_DRV2605::playEffectAsync(HapticEffectId effect)
{
    return playEffect(effect);
}

void HapticDriver_DRV2605::continuousVibration(uint32_t duration_ms, bool blocking)
{
    (void)duration_ms;
    (void)blocking;
}

void HapticDriver_DRV2605::vibrationUpdate()
{
}

bool HapticDriver_DRV2605::setSequence(uint8_t slot, HapticEffectId effect)
{
    if (slot >= 8) return false;
    _waveformSeq[slot] = static_cast<uint8_t>(effect);
    setWaveform(slot, _waveformSeq[slot]);
    return true;
}

bool HapticDriver_DRV2605::clearSequence()
{
    for (uint8_t i = 0; i < 8; i++) {
        _waveformSeq[i] = 0;
        setWaveform(i, 0);
    }
    return true;
}

bool HapticDriver_DRV2605::playSequence()
{
    return run();
}

HapticCapabilities HapticDriver_DRV2605::getCapabilities() const
{
    HapticCapabilities cap;
    cap.hasF0Calibration = 0;
    cap.hasVbatCompensation = 0;
    cap.hasRTP = 1;
    cap.hasSequence = 1;
    cap.hasAutoCalibration = 1;
    cap.hasBreakEffect = 1;
    cap.hasOverdrive = 1;
    cap.hasContinuousMode = 0;
    cap.maxEffectCount = 117;  // ROM library has many effects
    cap.maxSequenceLength = 8;
    cap.maxDurationMs = 0;  // Duration depends on waveform, not continuous
    return cap;
}

uint8_t HapticDriver_DRV2605::getChipID() const
{
    int chipID = readReg(DRV2605_REG_STATUS);
    if (chipID < 0) return 0;
    return static_cast<uint8_t>(chipID >> 5);
}

HapticActuatorType HapticDriver_DRV2605::getActuatorType() const
{
    return _isERM ? HapticActuatorType::ERM : HapticActuatorType::LRA;
}

bool HapticDriver_DRV2605::setActuatorType(HapticActuatorType type)
{
    _isERM = (type == HapticActuatorType::ERM);

    if (_isERM) {
        writeReg(DRV2605_REG_LIBRARY, 1);
        updateBits(DRV2605_REG_FEEDBACK, 0x80, 0x00);
    } else {
        writeReg(DRV2605_REG_LIBRARY, 6);
        updateBits(DRV2605_REG_FEEDBACK, 0x80, 0x80);
    }
    return true;
}

bool HapticDriver_DRV2605::setMode(HapticMode mode)
{
    uint8_t value = 0;
    switch (mode) {
    case HapticMode::INTERNAL_TRIGGER:
        value = 0x00;
        break;
    case HapticMode::EXT_TRIGGER_EDGE:
        value = 0x01;
        break;
    case HapticMode::EXT_TRIGGER_LEVEL:
        value = 0x02;
        break;
    case HapticMode::PWM_ANALOG:
        value = 0x03;
        break;
    case HapticMode::AUDIO_TO_VIBE:
        value = 0x04;
        break;
    case HapticMode::REAL_TIME_PLAYBACK:
        value = 0x05;
        break;
    case HapticMode::DIAGNOSTICS:
        value = 0x06;
        break;
    case HapticMode::AUTO_CALIBRATE:
        value = 0x07;
        break;
    case HapticMode::STANDBY:
        value = 0x40;
        break;
    default:
        return false;
    }
    if (writeReg(DRV2605_REG_MODE, value) == 0) {
        _isStandby = (mode == HapticMode::STANDBY);
        return true;
    }
    return false;
}

HapticMode HapticDriver_DRV2605::getMode() const
{
    if (_isStandby) {
        return HapticMode::STANDBY;
    }
    int value = readReg(DRV2605_REG_MODE);
    if (value < 0) {
        return HapticMode::INTERNAL_TRIGGER;
    }
    return static_cast<HapticMode>(value & 0x07);
}

bool HapticDriver_DRV2605::setRealtimeValue(uint8_t value)
{
    return writeReg(DRV2605_REG_RTPIN, value) == 0;
}

uint8_t HapticDriver_DRV2605::getRealtimeValue() const
{
    int value = readReg(DRV2605_REG_RTPIN);
    return value >= 0 ? static_cast<uint8_t>(value) : 0;
}

bool HapticDriver_DRV2605::calibrate()
{
    int result = autoCal();
    return result >= 0;
}

bool HapticDriver_DRV2605::needsCalibration() const
{
    return true;  // DRV2605 typically needs calibration
}

uint32_t HapticDriver_DRV2605::getF0() const
{
    int value = readReg(DRV2605_REG_LRARESON);
    if (value < 0) return 0;

    uint8_t period = static_cast<uint8_t>(value & 0x7F);
    if (period == 0) return 0;

    return static_cast<uint32_t>(1000000 / (period * 75));
}

uint32_t HapticDriver_DRV2605::getVbat()
{
    int vbat = readReg(DRV2605_REG_VBAT);
    if (vbat < 0) {
        return 0;
    }
    uint32_t mv =  (static_cast<float>(vbat)  * 5.6 / 255) * 1000;
    return mv;
}

void HapticDriver_DRV2605::setWaveform(uint8_t slot, uint8_t w)
{
    writeReg(static_cast<uint8_t>(DRV2605_REG_WAVESEQ1 + slot), w);
}

void HapticDriver_DRV2605::selectLibrary(uint8_t lib)
{
    _currentLibrary = lib;
    writeReg(DRV2605_REG_LIBRARY, lib);
}

bool HapticDriver_DRV2605::isDone() const
{
    int value = readReg(DRV2605_REG_GO);
    return value == 0;
}

void HapticDriver_DRV2605::trigger()
{
    run();
}

void HapticDriver_DRV2605::setRatedVoltage(uint8_t value)
{
    writeReg(DRV2605_REG_RATEDV, value);
}

void HapticDriver_DRV2605::setOverdriveClamp(uint8_t value)
{
    writeReg(DRV2605_REG_CLAMPV, value);
}

int HapticDriver_DRV2605::autoCal()
{
    setMode(HapticMode::AUTO_CALIBRATE);
    run();

    uint32_t start = millis();
    while (!isDone() && (millis() - start < 1000)) {
        delay(10);
    }

    if (isDone()) {
        int comp = readReg(DRV2605_REG_AUTOCALCOMP);
        int backEmf = readReg(DRV2605_REG_AUTOCALEMP);

        setMode(HapticMode::INTERNAL_TRIGGER);

        if (comp >= 0 && backEmf >= 0) {
            return comp;
        }
    }

    return -1;
}

void HapticDriver_DRV2605::setAutoBrake(bool enable)
{
    int value = readReg(DRV2605_REG_CONTROL2);
    if (value >= 0) {
        if (enable) {
            value |= 0x10;
        } else {
            value &= ~0x10;
        }
        writeReg(DRV2605_REG_CONTROL2, static_cast<uint8_t>(value));
    }
}

void HapticDriver_DRV2605::setOverdriveTime(uint8_t cycles)
{
    writeReg(DRV2605_REG_OVERDRIVE, cycles);
}

void HapticDriver_DRV2605::setBrakeTime(uint8_t time)
{
    writeReg(DRV2605_REG_BREAK, time);
}

void HapticDriver_DRV2605::setRtpOverdrive(bool enable)
{
    int value = readReg(DRV2605_REG_CONTROL3);
    if (value >= 0) {
        if (enable) {
            value |= 0x80;
        } else {
            value &= ~0x80;
        }
        writeReg(DRV2605_REG_CONTROL3, static_cast<uint8_t>(value));
    }
}

void HapticDriver_DRV2605::setERMLRA(bool erm)
{
    _isERM = erm;
    setActuatorType(erm ? HapticActuatorType::ERM : HapticActuatorType::LRA);
}

void HapticDriver_DRV2605::setCtrl4(uint8_t value)
{
    writeReg(DRV2605_REG_CONTROL4, value);
}

uint8_t HapticDriver_DRV2605::getLRAFrequency() const
{
    int value = readReg(DRV2605_REG_LRARESON);
    return value >= 0 ? static_cast<uint8_t>(value & 0x7F) : 0;
}

bool HapticDriver_DRV2605::initImpl(uint8_t param)
{
    (void)param;

    int chipID = readReg(DRV2605_REG_STATUS);
    if (chipID < 0) {
        return false;
    }

    chipID >>= 5;

    switch (chipID) {
    case DRV2604_CHIP_ID:
    case DRV2605_CHIP_ID:
    case DRV2604L_CHIP_ID:
    case DRV2605L_CHIP_ID:
    case DRV2605X_CHIP_ID:
        break;
    default:
        return false;
    }

    writeReg(DRV2605_REG_MODE, 0x00);
    writeReg(DRV2605_REG_RTPIN, 0x00);

    writeReg(DRV2605_REG_WAVESEQ1, 1);
    writeReg(DRV2605_REG_WAVESEQ2, 0);

    writeReg(DRV2605_REG_OVERDRIVE, 0);
    writeReg(DRV2605_REG_SUSTAINPOS, 0);
    writeReg(DRV2605_REG_SUSTAINNEG, 0);
    writeReg(DRV2605_REG_BREAK, 0);
    writeReg(DRV2605_REG_AUDIOMAX, 0x64);

    int value = readReg(DRV2605_REG_FEEDBACK);
    if (value == -1) {
        return false;
    }
    writeReg(DRV2605_REG_FEEDBACK, static_cast<uint8_t>(value & 0x7F));

    int control3 = readReg(DRV2605_REG_CONTROL3);
    if (control3 == -1) {
        return false;
    }
    writeReg(DRV2605_REG_CONTROL3, static_cast<uint8_t>(control3 | 0x20));

    return true;
}
