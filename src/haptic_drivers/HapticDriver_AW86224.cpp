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
 * @file      HapticDriver_AW86224.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-06
 *
 */

#include "HapticDriver_AW86224.hpp"
#include "HapticDriver_AW86224_Reg.hpp"
#include "AW86224_RamData.hpp"

HapticDriver_AW86224::HapticDriver_AW86224() :
    HapticBase("AW86224"),
    _rstPin(-1),
    _chipID(0),
    _playMode(PlayMode::STANDBY),
    _gain(0x80),
    _f0(0), _vbat(0), _lra(0),
    _ramInit(false),
    _cont_drv2_lvl(0),
    _f0_cali_data(0),
    _f0_pre(1700),
    _ramBaseAddr(0), _ramNum(0),
    _lra_vrms(1000),
    _cont_brk_time(0x06), _cont_brk_gain(0x08),
    _cont_drv1_lvl(0x7F), _cont_drv1_time(0x04), _cont_drv2_time(0x14),
    _cont_track_margin(0x0F),
    _d2s_gain(0x05),
    _pwmcfg4_prtime(0x32), _pwmcfg3_prlvl(0x3F),
    _auto_brk_enabled(false),
    _f0_cali_percent(7),
    _duration(0), _vibrationStartTime(0),
    _isCalibrated(false)
{
}

void HapticDriver_AW86224::setPins(uint8_t resetPin, uint8_t intPin)
{
    _rstPin = resetPin;
    _intPin = intPin;
}

void HapticDriver_AW86224::end()
{
    stop();
    setReady(false);
}

bool HapticDriver_AW86224::isReady() const
{
    return _isReady && _ramInit;
}

const char *HapticDriver_AW86224::getChipName() const
{
    switch (_chipID) {
    case static_cast<uint8_t>(ChipID::AW86214):
        return "AW86214";
    case static_cast<uint8_t>(ChipID::AW86223):
        return "AW86223";
    case static_cast<uint8_t>(ChipID::AW86224):
        return "AW86224";
    default:
        break;
    }
    return "UNKNOWN";
}

// ==================== HapticBase Interface ====================

HapticCapabilities HapticDriver_AW86224::getCapabilities() const
{
    HapticCapabilities cap;
    cap.hasF0Calibration = 1;
    cap.hasVbatCompensation = 1;
    cap.hasRTP = 1;
    cap.hasSequence = 1;
    cap.hasAutoCalibration = 0;
    cap.hasBreakEffect = 1;
    cap.hasOverdrive = 0;
    cap.hasContinuousMode = 1;
    cap.maxEffectCount = _ramNum > 0 ? _ramNum : 4;
    cap.maxSequenceLength = 8;
    cap.maxDurationMs = 0;  // Unlimited
    return cap;
}

HapticStatus HapticDriver_AW86224::getStatus() const
{
    if (_playMode == PlayMode::STANDBY) return HapticStatus::STANDBY;
    if (!_isReady) return HapticStatus::ERROR;
    if (isPlaying()) return HapticStatus::PLAYING;
    return HapticStatus::IDLE;
}

bool HapticDriver_AW86224::isPlaying() const
{
    if (_playMode == PlayMode::STANDBY) return false;
    uint8_t state = getGlobalState() & 0x0F;
    return state != 0x00 && state != 0x07;
}

uint8_t HapticDriver_AW86224::getGain() const
{
    return _gain;
}

bool HapticDriver_AW86224::playEffect(HapticEffectId effect)
{
    if (!_ramInit) return false;
    shortVibration(static_cast<uint8_t>(effect), _gain, 1);
    return true;
}

bool HapticDriver_AW86224::playEffectAsync(HapticEffectId effect)
{
    if (!_ramInit) return false;
    if (isPlaying()) {
        stop();
        hal->delay(20);
    }
    cancelVibration();
    softReset();
    hal->delay(10);

    setTrimLRA(static_cast<uint8_t>(_f0_cali_data));
    setWaveSeq(0, static_cast<uint8_t>(effect));
    setWaveSeq(1, 0);
    setWaveLoop(0, 0);
    setGain(_gain);
    playMode(PlayMode::RAM);
    run();
    return true;
}

void HapticDriver_AW86224::continuousVibration(uint32_t duration_ms, bool blocking)
{
    if (!_ramInit) return;
    longVibration(2, _gain, duration_ms, blocking);
}

void HapticDriver_AW86224::vibrationUpdate()
{
    if (!isPlaying()) return;

    if (_duration > 0 && hal->millis() - _vibrationStartTime >= _duration) {
        stop();
        return;
    }

    if ((getGlobalState() & 0x0F) != 0x07) run();
}

bool HapticDriver_AW86224::setSequence(uint8_t slot, HapticEffectId effect)
{
    if (slot >= 8) return false;
    setWaveSeq(slot, static_cast<uint8_t>(effect));
    return true;
}

bool HapticDriver_AW86224::clearSequence()
{
    for (uint8_t i = 0; i < 8; i++) {
        setWaveSeq(i, 0);
    }
    return true;
}

bool HapticDriver_AW86224::playSequence()
{
    if (!_ramInit) return false;
    playMode(PlayMode::RAM);
    return run();
}

bool HapticDriver_AW86224::calibrate()
{
    if (!_isReady) return false;
    f0Calibration();
    _isCalibrated = true;
    return _f0 > 0;
}

bool HapticDriver_AW86224::needsCalibration() const
{
    return !_isCalibrated || _f0 == 0;
}

// ==================== Internal Helper Functions ====================

void HapticDriver_AW86224::cancelVibration()
{
    playStop();
}

void HapticDriver_AW86224::ram_vbat_comp(bool flag)
{
    if (!flag) return;

    uint32_t vbat = _vbat < 3000 ? 3000 : _vbat;
    int temp_gain = static_cast<int>(_gain) * AW_VBAT_REFER / vbat;
    if (temp_gain > 255) temp_gain = 255;

#ifdef HAPTIC_DEBUG
    Serial.printf("[VBAT_COMP] vbat=%d, orig=0x%02X, comp=0x%02X\n", vbat, _gain, temp_gain);
#endif

    setGain(static_cast<uint8_t>(temp_gain));
}

void HapticDriver_AW86224::ramInit(bool enable)
{
    if (enable) {
        updateBits(REG_SYSCTRL1, MASK_EN_RAMINIT, MASK_EN_RAMINIT);
        hal->delayMicroseconds(500);
    } else {
        updateBits(REG_SYSCTRL1, MASK_EN_RAMINIT, 0);
    }
}

void HapticDriver_AW86224::playStop()
{
    _playMode = PlayMode::STANDBY;
    ramInit(true);
    updateBits(REG_PLAYCFG3, MASK_PLAY_MODE, MASK_PLAY_MODE);
    writeReg(REG_PLAYCFG4, PLAYCFG4_GO_ON);
    ramInit(false);

    uint8_t val = 0;
    uint8_t cnt = 40;
    while (cnt--) {
        val = readReg(REG_GLBRD5);
        if ((val & MASK_GLB_STATE) == STATE_STANDBY) break;
        hal->delay(5);
    }

    if ((val & MASK_GLB_STATE) != STATE_STANDBY) {
        updateBits(REG_SYSCTRL2, MASK_STANDBY, MASK_STANDBY);
        hal->delay(2);
        updateBits(REG_SYSCTRL2, MASK_STANDBY, 0);
    }
}

bool HapticDriver_AW86224::checkQualify()
{
    return (readReg(REG_CHIPID) & 0x80) == 0x80;
}

bool HapticDriver_AW86224::offsetCalibration()
{
    if (_d2s_gain == 0) return false;

    updateBits(REG_SYSCTRL7, MASK_D2S_GAIN, _d2s_gain & MASK_D2S_GAIN);
    int d2s_gain = selectD2sGain(_d2s_gain & MASK_D2S_GAIN);
    if (d2s_gain < 0) return false;

    ramInit(true);
    updateBits(REG_DETCFG1, MASK_RL_OS, 0);
    updateBits(REG_DETCFG2, MASK_DIAG_GO, MASK_DIAG_GO);
    hal->delay(3);

    uint8_t reg_val[2];
    reg_val[0] = readReg(REG_DET_OS);
    reg_val[1] = readReg(REG_DET_LO);
    ramInit(false);

    int os_code = (reg_val[1] & 0xFC) >> 2;
    os_code = (reg_val[0] << 2) | os_code;
    os_code = os_formula(os_code, d2s_gain);

    return (os_code <= 15 && os_code >= -15);
}

int HapticDriver_AW86224::selectD2sGain(uint8_t reg)
{
    switch (reg) {
    case 0: return 1;
    case 1: return 2;
    case 2: return 4;
    case 3: return 5;
    case 4: return 8;
    case 5: return 10;
    case 6: return 20;
    case 7: return 40;
    default: return -1;
    }
}

void HapticDriver_AW86224::setSramSize(SramSize size)
{
    switch (size) {
    case SramSize::SRAM_1K:
        updateBits(REG_RTPCFG1, _BV(5), 0);
        updateBits(REG_RTPCFG1, _BV(4), _BV(4));
        break;
    case SramSize::SRAM_2K:
        updateBits(REG_RTPCFG1, _BV(5), _BV(5));
        updateBits(REG_RTPCFG1, _BV(4), 0);
        break;
    case SramSize::SRAM_3K:
        updateBits(REG_RTPCFG1, _BV(4) | _BV(5), _BV(4) | _BV(5));
        break;
    }
}

void HapticDriver_AW86224::autoBrkConfig(bool enable)
{
    updateBits(REG_PLAYCFG3, MASK_BRK_EN, enable ? MASK_BRK_EN : 0);
}

void HapticDriver_AW86224::setPwm(PwmMode mode)
{
    switch (mode) {
    case PwmMode::PWM_48K: updateBits(REG_SYSCTRL2, MASK_WAVDAT_MODE, 1); break;
    case PwmMode::PWM_24K: updateBits(REG_SYSCTRL2, MASK_WAVDAT_MODE, 0); break;
    case PwmMode::PWM_12K: updateBits(REG_SYSCTRL2, MASK_WAVDAT_MODE, 2); break;
    }
}

void HapticDriver_AW86224::playMode(PlayMode mode)
{
    _playMode = mode;
    switch (mode) {
    case PlayMode::STANDBY:
        playStop();
        break;
    case PlayMode::RAM:
        setPwm(PwmMode::PWM_12K);
        autoBrkConfig(false);
        updateBits(REG_PLAYCFG3, MASK_PLAY_MODE, 0);
        break;
    case PlayMode::RAM_LOOP:
        setPwm(PwmMode::PWM_12K);
        autoBrkConfig(true);
        updateBits(REG_PLAYCFG3, MASK_PLAY_MODE, 0);
        break;
    case PlayMode::CONT:
        autoBrkConfig(true);
        updateBits(REG_PLAYCFG3, MASK_PLAY_MODE, 2);
        break;
    case PlayMode::RTP:
        setPwm(PwmMode::PWM_24K);
        autoBrkConfig(true);
        updateBits(REG_PLAYCFG3, MASK_PLAY_MODE, 1);
        break;
    }
}

void HapticDriver_AW86224::config()
{
    setSramSize(SramSize::SRAM_3K);
    updateBits(REG_TRGCFG8, MASK_TRG1_MODE, 0x10);
    updateBits(REG_ANACFG8, MASK_TRTF_CTRL_HDRV, MASK_TRTF_CTRL_HDRV);

    if (_cont_brk_time) writeReg(REG_CONTCFG10, _cont_brk_time);
    else _cont_brk_time = 1;

    if (_cont_brk_gain) updateBits(REG_CONTCFG5, MASK_BRK_GAIN, _cont_brk_gain);
    else _cont_brk_gain = 1;
}

void HapticDriver_AW86224::protectConfig(uint8_t prtime, uint8_t prlvl)
{
    updateBits(REG_PWMCFG1, MASK_PRC_EN, 0);
    if (prlvl != 0) {
        writeReg(REG_PWMCFG3, MASK_PR_EN | (prlvl & ~MASK_PRLVL));
        writeReg(REG_PWMCFG4, prtime);
    } else {
        updateBits(REG_PWMCFG3, MASK_PR_EN, 0);
    }
}

void HapticDriver_AW86224::miscParaInit()
{
    updateBits(REG_SYSCTRL7, MASK_GAIN_BYPASS, GAIN_BYPASS_CHANGEABLE);
    _cont_drv2_lvl = drv2_lvl_formula(_f0_pre, _lra_vrms);
    if (_cont_drv2_lvl > 127) _cont_drv2_lvl = 127;
    config();
    setPwm(PwmMode::PWM_12K);
    protectConfig(_pwmcfg4_prtime, _pwmcfg3_prlvl);
}

void HapticDriver_AW86224::trigInit()
{
    updateBits(REG_TRGCFG8, MASK_TRG1_MODE, 0x10);
    updateBits(REG_SYSCTRL2, MASK_INTN_PIN, 0);
}

void HapticDriver_AW86224::setTrimLRA(uint8_t val)
{
    updateBits(REG_TRIMCFG3, MASK_TRIM_LRA, val);
}

bool HapticDriver_AW86224::measureF0()
{
    uint8_t val[2] = {0};
    uint8_t brk_en_temp = 0;
    uint32_t f0_reg = 0;
    int cnt = 200;
    bool success = false;

    playStop();
    updateBits(REG_SYSCTRL7, MASK_D2S_GAIN, _d2s_gain & MASK_D2S_GAIN);

    _playMode = PlayMode::CONT;
    autoBrkConfig(true);
    updateBits(REG_PLAYCFG3, MASK_PLAY_MODE, 2);
    updateBits(REG_CONTCFG1, MASK_EN_F0_DET, MASK_EN_F0_DET);
    updateBits(REG_CONTCFG6, MASK_TRACK_EN, MASK_TRACK_EN);

    brk_en_temp = readReg(REG_PLAYCFG3) & MASK_BRK_EN;
    updateBits(REG_CONTCFG6, MASK_DRV1_LVL, _cont_drv1_lvl);
    writeReg(REG_CONTCFG7, _cont_drv2_lvl);
    writeReg(REG_CONTCFG8, _cont_drv1_time);
    writeReg(REG_CONTCFG9, _cont_drv2_time);
    writeReg(REG_CONTCFG11, _cont_track_margin);

    if (_f0_pre == 0) return false;
    int drv_width = (((_f0_pre / 10) * 15) / (_cont_track_margin + 3)) / 100;
    if (drv_width < 8) drv_width = 8;
    if (drv_width > 255) drv_width = 255;
    writeReg(REG_CONTCFG3, static_cast<uint8_t>(drv_width));

    run();
    hal->delay(20);

    while (cnt--) {
        if ((getGlobalState() & MASK_GLB_STATE) == STATE_STANDBY) {
            success = true;
            break;
        }
        hal->delay(10);
    }

    if (success) {
        readRegBuff(REG_CONTRD14, val, 2);
        f0_reg = (val[0] << 8) | val[1];
        if (f0_reg) _f0 = f0_formula(f0_reg);
    }

    updateBits(REG_CONTCFG1, MASK_EN_F0_DET, 0);
    updateBits(REG_PLAYCFG3, MASK_BRK_EN, brk_en_temp);
    playStop();
    return success;
}

bool HapticDriver_AW86224::measureVbat()
{
    playStop();
    ramInit(true);
    updateBits(REG_DETCFG2, MASK_VBAT_GO, MASK_VBAT_GO);
    hal->delay(2);

    uint32_t vbat_code = readReg(REG_DET_VBAT) << 2;
    vbat_code |= (readReg(REG_DET_LO) & MASK_VBAT_LO) >> SHIFT_VBAT_LO;

    _vbat = vbat_formula(vbat_code);
    if (_vbat > 4500) _vbat = 4500;
    if (_vbat < 3000) _vbat = 3000;

    ramInit(false);
    return true;
}

void HapticDriver_AW86224::getLraResistance()
{
    uint8_t d2s_gain_temp = readReg(REG_SYSCTRL7) & MASK_D2S_GAIN;
    int d2s_gain = selectD2sGain(5);
    if (d2s_gain < 0) return;

    playStop();
    ramInit(true);
    hal->delay(2);
    updateBits(REG_SYSCTRL2, MASK_STANDBY, 0);
    updateBits(REG_DETCFG1, MASK_RL_OS, MASK_RL_OS);
    updateBits(REG_DETCFG2, MASK_DIAG_GO, MASK_DIAG_GO);
    hal->delay(3);

    uint32_t lra_code = readReg(REG_DET_RL) << 2;
    lra_code |= readReg(REG_DET_LO) & MASK_RL_LO;
    _lra = rl_formula(lra_code, d2s_gain) * 10;

    ramInit(false);
    updateBits(REG_SYSCTRL7, MASK_D2S_GAIN, d2s_gain_temp);
}

void HapticDriver_AW86224::setPlayMode(PlayMode mode)
{
    playMode(mode);
}

bool HapticDriver_AW86224::setGain(uint8_t gain)
{
    _gain = gain;
    writeReg(REG_PLAYCFG2, gain);
    return true;
}

bool HapticDriver_AW86224::run()
{
    ramInit(true);
    hal->delayMicroseconds(50);
    int ret = writeReg(REG_PLAYCFG4, PLAYCFG4_GO_ON);
    hal->delayMicroseconds(50);
    ramInit(false);
    return ret == 0;
}

bool HapticDriver_AW86224::stop()
{
    playStop();
    _duration = 0;
    return true;
}

void HapticDriver_AW86224::shortVibration(uint8_t index, uint8_t gain, uint8_t loop)
{
    if (!_ramInit || index == 0) return;

    if (isPlaying()) {
        stop();
        hal->delay(20);
    }

    cancelVibration();
    softReset();
    hal->delay(10);

    setTrimLRA(static_cast<uint8_t>(_f0_cali_data));
    setWaveSeq(0, index);
    setWaveSeq(1, 0);
    setWaveLoop(0, loop > 0 ? loop - 1 : 0);
    setGain(gain);
    playMode(PlayMode::RAM);
    run();

    uint32_t playTime = loop * 50;
    if (playTime < 100) playTime = 100;
    hal->delay(playTime);
}

void HapticDriver_AW86224::longVibration(uint8_t index, uint8_t gain, uint32_t duration_ms, bool blocking)
{
    if (!_ramInit || index == 0 || duration_ms == 0) return;

    if (isPlaying()) {
        stop();
        hal->delay(20);
    }

    cancelVibration();
    softReset();
    hal->delay(10);

    _gain = gain;
    _duration = duration_ms;
    _vibrationStartTime = hal->millis();
    setTrimLRA(static_cast<uint8_t>(_f0_cali_data));
    setGain(gain);
    ram_vbat_comp(true);

    setRepeatSeq(index);
    playMode(PlayMode::RAM_LOOP);
    run();

    if (blocking) {
        uint32_t start = hal->millis();
        while (hal->millis() - start < duration_ms) {
            if ((getGlobalState() & 0x0F) != 0x07) run();
            hal->delay(10);
        }
        stop();
    }
}

void HapticDriver_AW86224::stopVibration()
{
    stop();
}

void HapticDriver_AW86224::setWaveSeq(uint8_t wav, uint8_t seq)
{
    writeReg(REG_WAVCFG1 + wav, seq);
}

void HapticDriver_AW86224::setWaveLoop(uint8_t wav, uint8_t loop)
{
    uint8_t reg = REG_WAVCFG9 + (wav / 2);
    if (wav % 2) {
        updateBits(reg, MASK_SEQ2LOOP, loop << SHIFT_SEQ2LOOP);
    } else {
        updateBits(reg, MASK_SEQ1LOOP, loop << SHIFT_SEQ1LOOP);
    }
}

void HapticDriver_AW86224::setFifoAddr()
{
    uint8_t ae_addr_h = (_ramBaseAddr >> 1) >> 4 & 0xF0;
    uint8_t af_addr_h = ((_ramBaseAddr - (_ramBaseAddr >> 2)) >> 8) & 0x0F;
    uint8_t val[3] = {
        static_cast<uint8_t>(ae_addr_h | af_addr_h),
        static_cast<uint8_t>((_ramBaseAddr >> 1) & 0xFF),
        static_cast<uint8_t>((_ramBaseAddr - (_ramBaseAddr >> 2)) & 0xFF)
    };
    writeRegBuff(REG_RTPCFG3, val, 3);
}

void HapticDriver_AW86224::setBaseAddr()
{
    updateBits(REG_RTPCFG1, MASK_BASE_ADDR_H, (_ramBaseAddr >> 8) & MASK_BASE_ADDR_H);
    writeReg(REG_RTPCFG2, _ramBaseAddr & 0xFF);
}

void HapticDriver_AW86224::setRamAddr()
{
    writeReg(REG_RAMADDRH, (_ramBaseAddr >> 8) & 0x3F);
    writeReg(REG_RAMADDRL, _ramBaseAddr & 0xFF);
}

void HapticDriver_AW86224::setRtpData(uint8_t *data, uint32_t len)
{
    writeRegBuff(REG_RTPDATA, data, len);
}

bool HapticDriver_AW86224::playRtp(const uint8_t *data, uint32_t len, uint8_t gain)
{
    if (!data || len == 0) return false;

    playStop();
    irqClear();
    setRtpAei(false);
    setGain(gain);
    setTrimLRA(0x00);
    setPwm(PwmMode::PWM_24K);
    autoBrkConfig(true);
    playMode(PlayMode::RTP);

    ramInit(true);
    updateBits(REG_RTPCFG1, MASK_BASE_ADDR_H, (_ramBaseAddr >> 8) & MASK_BASE_ADDR_H);
    writeReg(REG_RTPCFG2, _ramBaseAddr & 0xFF);
    ramInit(false);

    if (!run()) return false;
    hal->delay(1);

    if (!waitRtpGo(200)) {
        playStop();
        return false;
    }

    writeRegBuff(REG_RTPDATA, const_cast<uint8_t*>(data), len);

    if (len > 256) {
        uint32_t cnt = 256;
        while (cnt < len && _playMode == PlayMode::RTP) {
            uint32_t buf_len = (len - cnt) > 64 ? 64 : (len - cnt);
            writeRegBuff(REG_RTPDATA, const_cast<uint8_t*>(data + cnt), buf_len);
            cnt += buf_len;
        }
    }
    return true;
}

bool HapticDriver_AW86224::waitRtpGo(uint32_t timeout_ms)
{
    uint32_t start = hal->millis();
    while (hal->millis() - start < timeout_ms) {
        if ((readReg(REG_GLBRD5) & 0x0F) == STATE_RTP_GO) return true;
        hal->delayMicroseconds(1000);
    }
    return false;
}

void HapticDriver_AW86224::setRepeatSeq(uint8_t seq)
{
    setWaveSeq(0, seq);
    setWaveSeq(1, 0);
    setWaveLoop(0, WAVLOOP_INIFINITELY);
}

void HapticDriver_AW86224::contConfig()
{
    _playMode = PlayMode::CONT;
    updateBits(REG_CONTCFG6, MASK_TRACK_EN | MASK_DRV1_LVL, MASK_TRACK_EN | (_cont_drv1_lvl & MASK_DRV1_LVL));
    writeReg(REG_CONTCFG7, _cont_drv2_lvl);
    writeReg(REG_CONTCFG9, 0xFF);
    run();
}

void HapticDriver_AW86224::interruptSetup()
{
    updateBits(REG_SYSCTRL7, INT_MODE_EDGE, INT_MODE_EDGE);
    updateBits(REG_SYSCTRL7, _BV(5), INT_EDGE_MODE_POS);
    updateBits(REG_SYSINTM, MASK_UVLM | MASK_FF_AEM | MASK_FF_AFM | MASK_OCDM | MASK_OTM | MASK_DONEM, MASK_UVLM);
}

uint8_t HapticDriver_AW86224::getGlobalState() const
{
    return readReg(REG_GLBRD5);
}

bool HapticDriver_AW86224::waitStandby(uint32_t timeout_ms)
{
    uint32_t start = hal->millis();
    while (hal->millis() - start < timeout_ms) {
        if ((getGlobalState() & MASK_GLB_STATE) == STATE_STANDBY) return true;
        hal->delay(2);
    }
    return false;
}

int HapticDriver_AW86224::getIrqState()
{
    uint8_t reg_val = readReg(REG_SYSINT);
    if (reg_val & MASK_UVLI) return IRQ_UVL;
    if (reg_val & MASK_OCDI) return IRQ_OCD;
    if (reg_val & MASK_OTI) return IRQ_OT;
    if (reg_val & MASK_DONEI) return IRQ_DONE;
    if (reg_val & MASK_FF_AFI) return IRQ_ALMOST_FULL;
    if (reg_val & MASK_FF_AEI) return IRQ_ALMOST_EMPTY;
    return IRQ_NULL;
}

void HapticDriver_AW86224::irqClear()
{
    readReg(REG_SYSINT);
}

uint8_t HapticDriver_AW86224::rtpGetFifoAfs()
{
    return (readReg(REG_SYSST) & MASK_FF_AFS) >> 3;
}

void HapticDriver_AW86224::setRtpAei(bool enable)
{
    updateBits(REG_SYSINTM, MASK_FF_AEM, enable ? 0 : MASK_FF_AEM);
}

bool HapticDriver_AW86224::loadRamData(const uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return false;
    if (!getBaseAddr(data, len)) return false;

    playStop();
    ramInit(true);
    setBaseAddr();
    setFifoAddr();
    setRamAddr();

    uint32_t i = 0;
    while (i < len) {
        uint32_t chunk = (len - i) > 32 ? 32 : (len - i);
        writeRegBuff(REG_RAMDATA, const_cast<uint8_t*>(data + i), chunk);
        i += chunk;
        hal->delayMicroseconds(100);
    }

    ramInit(false);
    trigInit();
    _ramInit = true;
    return true;
}

bool HapticDriver_AW86224::getBaseAddr(const uint8_t *data, uint32_t len)
{
    uint16_t last_end = 0, next_start = 0;
    int ram_num = 1;

    for (int i = 3; i < static_cast<int>(len); i += 4) {
        last_end = (data[i] << 8) | data[i + 1];
        next_start = (data[i + 2] << 8) | data[i + 3];
        if ((next_start - last_end) == 1) ram_num++;
        else break;
    }

    for (int i = ram_num * 4; i >= 4; i -= 4) {
        last_end = (data[i - 1] << 8) | data[i];
        _ramBaseAddr = static_cast<int>((data[1] << 8) | data[2]) - ram_num * 4 - 1;
        if ((last_end - _ramBaseAddr + 1) == static_cast<int>(len)) {
            _ramNum = ram_num;
            return true;
        }
        ram_num--;
    }

    _ramBaseAddr = (data[1] << 8) | data[2];
    _ramNum = ((_ramBaseAddr - _ramBaseAddr) - 1) / 4;
    return true;
}

void HapticDriver_AW86224::vbatModeConfig(uint8_t flag)
{
    if (flag == CONT_VBAT_HW_COMP_MODE) {
        writeReg(REG_GLBCFG2, 0x0C);
        updateBits(REG_SYSCTRL1, MASK_VBAT_MODE, MASK_VBAT_MODE);
    } else {
        updateBits(REG_SYSCTRL1, MASK_VBAT_MODE, 0);
    }
}

void HapticDriver_AW86224::calculateCaliData()
{
    int f0_cali_step = 100000 * (static_cast<int>(_f0) - static_cast<int>(_f0_pre)) / (static_cast<int>(_f0_pre) * F0_CALI_ACCURACY);

    if (f0_cali_step >= 0) {
        f0_cali_step = (f0_cali_step % 10 >= 5) ? (32 + f0_cali_step / 10 + 1) : (32 + f0_cali_step / 10);
    } else {
        f0_cali_step = (f0_cali_step % 10 <= -5) ? (32 + f0_cali_step / 10 - 1) : (32 + f0_cali_step / 10);
    }

    int8_t f0_cali_lra = (f0_cali_step > 31) ? static_cast<int8_t>(f0_cali_step - 32) : static_cast<int8_t>(f0_cali_step + 32);
    _f0_cali_data = f0_cali_lra;
}

void HapticDriver_AW86224::reset()
{
    if (_rstPin != -1) {
        hal->pinMode(_rstPin, OUTPUT);
        hal->digitalWrite(_rstPin, LOW);
        hal->delay(2);
        hal->digitalWrite(_rstPin, HIGH);
        hal->delay(8);
    }
}

bool HapticDriver_AW86224::setActuatorType(HapticActuatorType type)
{
    (void)type;
    return false;
}

HapticActuatorType HapticDriver_AW86224::getActuatorType() const
{
    return HapticActuatorType::LRA;
}

bool HapticDriver_AW86224::setMode(HapticMode mode)
{
    switch (mode) {
    case HapticMode::INTERNAL_TRIGGER:
        playMode(PlayMode::RAM);
        break;
    case HapticMode::REAL_TIME_PLAYBACK:
        playMode(PlayMode::RTP);
        break;
    case HapticMode::STANDBY:
        playStop();
    default:
        return false;
    }
    return true;
}

HapticMode HapticDriver_AW86224::getMode() const
{
    switch (_playMode) {
    case PlayMode::RTP:
        return HapticMode::REAL_TIME_PLAYBACK;
    case PlayMode::CONT:
        return HapticMode::AUTO_CALIBRATE;
    case PlayMode::STANDBY:
        return HapticMode::STANDBY;
        break;
    default:
        return HapticMode::INTERNAL_TRIGGER;
    }
}

bool HapticDriver_AW86224::setRealtimeValue(uint8_t value)
{
    if (_playMode != PlayMode::RTP) {
        playMode(PlayMode::RTP);
    }
    setGain(value);
    return true;
}

uint32_t HapticDriver_AW86224::getVbat()
{
    measureVbat();
    return _vbat;
}

uint32_t HapticDriver_AW86224::getF0() const
{
    return _f0;
}

void HapticDriver_AW86224::softReset()
{
    writeReg(REG_SRST, SOFTRESET_VAL);
    hal->delay(2);
}

uint8_t HapticDriver_AW86224::getChipID() const
{
    return _chipID;
}

bool HapticDriver_AW86224::initImpl(uint8_t param)
{
    (void)param;

    if (_rstPin != -1) {
        hal->pinMode(_rstPin, OUTPUT);
        hal->digitalWrite(_rstPin, LOW);
        hal->delay(2);
        hal->digitalWrite(_rstPin, HIGH);
        hal->delay(8);
    }

    if (!parseChipID()) return false;

    softReset();

    if (!offsetCalibration()) {
        softReset();
        offsetCalibration();
    }

    interruptSetup();
    hapticInit();
    trigInit();

    if (!loadRamData(aw862xx_ram_data, aw862xx_ram_len)) return false;
    playMode(PlayMode::STANDBY);
    return true;
}

bool HapticDriver_AW86224::parseChipID()
{
    uint8_t reg = 0;
    for (int i = 0; i < 5; i++) {
        reg = readReg(0x00);
        if (reg != 0) break;
        hal->delay(2);
    }

    if (reg == 0x00) {
        uint8_t ef_id = readReg(REG_CHIPID);
        if ((ef_id & 0x41) == 0x01) {
            _chipID = static_cast<uint8_t>(ChipID::AW86223);
        } else if ((ef_id & 0x41) == 0x00) {
            _chipID = static_cast<uint8_t>(ChipID::AW86224);
        } else {
            return false;
        }
    } else if (reg == 0x01) {
        uint8_t ef_id = readReg(REG_CHIPID);
        if ((ef_id & 0x41) == 0x41) {
            _chipID = static_cast<uint8_t>(ChipID::AW86214);
        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

void HapticDriver_AW86224::hapticInit()
{
    _f0_pre = F0_PRE_VAL;
    playMode(PlayMode::STANDBY);
    miscParaInit();
    measureVbat();
    vbatModeConfig(CONT_VBAT_HW_COMP_MODE);
    f0Calibration();
    playMode(PlayMode::STANDBY);
}

void HapticDriver_AW86224::f0Calibration()
{
    setTrimLRA(0x00);

    if (!measureF0()) {
        _f0 = _f0_pre;
    }

    if (!judgeF0WithinRange()) return;

    calculateCaliData();
    setTrimLRA(static_cast<uint8_t>(_f0_cali_data));
    _isCalibrated = true;
}

bool HapticDriver_AW86224::judgeF0WithinRange()
{
    uint32_t f0_cali_min = _f0_pre * (100 - _f0_cali_percent) / 100;
    uint32_t f0_cali_max = _f0_pre * (100 + _f0_cali_percent) / 100;
    return (_f0 >= f0_cali_min && _f0 <= f0_cali_max);
}

void HapticDriver_AW86224::printRegs()
{
#ifdef HAPTIC_DEBUG
    Serial.println("\n=== Register Dump ===");
    Serial.printf("PLAYCFG2: 0x%02X\n", readReg(0x07));
    Serial.printf("PLAYCFG3: 0x%02X\n", readReg(0x08));
    Serial.printf("PLAYCFG4: 0x%02X\n", readReg(0x09));
    Serial.printf("WAVCFG1:  0x%02X\n", readReg(0x0A));
    Serial.printf("WAVCFG2:  0x%02X\n", readReg(0x0B));
    Serial.printf("WAVCFG9:  0x%02X\n", readReg(0x12));
    Serial.printf("GLBRD5:   0x%02X\n", readReg(0x3F));
    Serial.printf("SYSCTRL7: 0x%02X\n", readReg(0x49));
    Serial.printf("TRIMCFG3: 0x%02X\n", readReg(0x5A));
    Serial.println("======================");
#endif
}
