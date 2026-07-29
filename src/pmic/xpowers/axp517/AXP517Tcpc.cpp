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
 * @file      AXP517Tcpc.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 */
#include "../../../SensorBuildOpt.h"
#if !SENSORLIB_EXCLUDE_PMIC_AXP517

#include "AXP517Tcpc.hpp"
#include "AXP517Regs.hpp"
#include "AXP517TcpcRegs.hpp"

using namespace axp517::tcpc;

namespace
{
static constexpr uint8_t POWER_STATUS_UNINIT      = (1u << 6);
static constexpr uint8_t POWER_STATUS_VBUS_PRESENT = (1u << 2);

static constexpr uint8_t ROLE_CTRL_SINK_RD_RD     = 0x0A;
static constexpr uint8_t ROLE_CTRL_OPEN_OPEN      = 0x0F;
static constexpr uint8_t ALERT_MASK_IRQ_VALUE     = 0x77;
static constexpr uint8_t POWER_STATUS_MASK_VALUE  = 0x04;
static constexpr uint8_t MSG_HDR_PD20_SINK_UFP    = 0x02;
static constexpr uint8_t RX_DETECT_SOP            = 0x01;
static constexpr uint8_t ADC_VBUS_ENABLE          = 0x04;
} // namespace

bool AXP517Tcpc::r8(uint8_t reg, uint8_t &out)
{
    int v = _core.readReg(reg);
    if (v < 0) return false;
    out = static_cast<uint8_t>(v);
    return true;
}

bool AXP517Tcpc::w8(uint8_t reg, uint8_t v)
{
    return _core.writeReg(reg, v) >= 0;
}

bool AXP517Tcpc::readNoInc(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _core.setAck(false);
    bool ok = _core.readRegBuff(reg, buf, len) >= 0;
    _core.setAck(true);
    return ok;
}

bool AXP517Tcpc::writeNoInc(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    if (!buf && len) return false;

    for (uint8_t i = 0; i < len; ++i) {
        if (!w8(reg, buf[i])) return false;
    }
    return true;
}

bool AXP517Tcpc::init()
{
    if (!_core.enableModule(AXP517Core::Module::TYPEC, true)) {
        return false;
    }
    _core.delayMs(20);

    if (!w8(CC_GENERAL_CONTROL, SW_RESET)) return false;
    _core.delayMs(50);
    if (!w8(CC_GENERAL_CONTROL, 0x00)) return false;
    _core.delayMs(300);

    for (uint8_t i = 0; i < 40; ++i) {
        uint8_t status = 0;
        if (!r8(POWER_STATUS, status)) return false;
        if ((status & POWER_STATUS_UNINIT) == 0) break;
        _core.delayMs(50);
    }

    uint8_t status = 0;
    if (!r8(POWER_STATUS, status)) return false;
    if (status & POWER_STATUS_UNINIT) return false;

    if (!w8(FAULT_STATUS, 0xFF)) return false;
    if (!w8(CC_CONNECTION_STATUS, 0xFF)) return false;

    if (!w8(ROLE_CONTROL, ROLE_CTRL_SINK_RD_RD)) return false;
    if (!w8(TCPC_CONTROL, 0x00)) return false;

    // Do not write PD_STATE/AWAKE_EN here. The verified Python flow leaves the
    // low-power/awake registers untouched so the BMC PHY keeps receiving PD frames.
    if (!w8(TWI_ADDR_STATIC, 0x01)) return false;

    if (!clearAllAlerts()) return false;
    if (!w8(POWER_STATUS_MASK, POWER_STATUS_MASK_VALUE)) return false;
    if (!w8(COMMAND, cmd::ENABLE_VBUS_DETECT)) return false;
    if (!w8(PD_ALERT_MASKL, ALERT_MASK_IRQ_VALUE)) return false;
    if (!w8(PD_ALERT_MASKH, 0x00)) return false;

    if (!w8(POWER_CONTROL, 0x10)) return false;

    uint8_t adc = 0;
    if (!r8(axp517_regs::adc::ENABLE, adc)) return false;
    if (!w8(axp517_regs::adc::ENABLE, static_cast<uint8_t>(adc | ADC_VBUS_ENABLE))) return false;

    if (!clearTx()) return false;
    if (!clearAllAlerts()) return false;
    if (!setSinkMessageHeaderAndRx()) return false;

    // Force a Type-C detach/attach when the charger is already plugged in, so
    // the Source re-sends Source_Capabilities after RX is enabled.
    if (!w8(ROLE_CONTROL, ROLE_CTRL_OPEN_OPEN)) return false;
    _core.delayMs(400);
    if (!w8(ROLE_CONTROL, ROLE_CTRL_SINK_RD_RD)) return false;
    _core.delayMs(150);

    if (!setPolarityFromCc()) return false;
    return clearAllAlerts();
}

bool AXP517Tcpc::checkVendorId()
{
    uint8_t lo = 0, hi = 0;
    if (!r8(TCPC_VENDOR_ID, lo)) return false;
    if (!r8(TCPC_VENDOR_ID + 1, hi)) return false;

    uint16_t vid = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    return vid == VENDOR_ID || vid == 0xE0C5;
}

bool AXP517Tcpc::readAlert(uint16_t &out)
{
    uint8_t lo = 0, hi = 0;
    if (!r8(PD_ALERTL, lo)) return false;
    if (!r8(PD_ALERTH, hi)) return false;
    out = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
    return true;
}

bool AXP517Tcpc::clearAlert(uint16_t maskRw1c)
{
    if (!w8(PD_ALERTL, static_cast<uint8_t>(maskRw1c & 0xFF))) return false;
    return w8(PD_ALERTH, static_cast<uint8_t>((maskRw1c >> 8) & 0xFF));
}

bool AXP517Tcpc::readFaultStatus(uint8_t &out)
{
    return r8(FAULT_STATUS, out);
}

bool AXP517Tcpc::clearFaultStatus(uint8_t maskRw1c)
{
    return w8(FAULT_STATUS, maskRw1c);
}

bool AXP517Tcpc::clearPmicIrqStatus()
{
    uint8_t status = 0;
    for (uint8_t reg = axp517_regs::irq::STATUS0; reg <= axp517_regs::irq::STATUS3; ++reg) {
        if (!r8(reg, status)) return false;
        if (status && !w8(reg, status)) return false;
    }
    return true;
}

bool AXP517Tcpc::writeMessageHeaderInfo(uint8_t v)
{
    return w8(MESSAGE_HEADER_INFO, v);
}

bool AXP517Tcpc::writeReceiveDetect(uint8_t v)
{
    return w8(RECEIVE_DETECT, v);
}

bool AXP517Tcpc::clearTx()
{
    if (!w8(TX_BYTE_CNT, 0x00)) return false;
    return w8(TX_BUF_TRANSMIT, 0x00);
}

bool AXP517Tcpc::setSinkMessageHeaderAndRx()
{
    if (!w8(MESSAGE_HEADER_INFO, MSG_HDR_PD20_SINK_UFP)) return false;
    return w8(RECEIVE_DETECT, RX_DETECT_SOP);
}

bool AXP517Tcpc::setPolarityFromCc()
{
    uint8_t cc = 0;
    if (!r8(CC_STATUS, cc)) return false;

    uint8_t cc1 = cc & 0x03;
    uint8_t cc2 = (cc >> 2) & 0x03;
    return w8(TCPC_CONTROL, (cc2 != 0 && cc1 == 0) ? 0x01 : 0x00);
}

bool AXP517Tcpc::getCc(CcStatus &cc1, CcStatus &cc2)
{
    uint8_t cc = 0;
    if (!r8(CC_STATUS, cc)) return false;

    auto decodeSinkCc = [](uint8_t raw) -> CcStatus {
        switch (raw & 0x03) {
        case 1: return CcStatus::RP_DEF;
        case 2: return CcStatus::RP_1_5;
        case 3: return CcStatus::RP_3_0;
        default: return CcStatus::OPEN;
        }
    };

    cc1 = decodeSinkCc(cc);
    cc2 = decodeSinkCc(cc >> 2);
    return true;
}

bool AXP517Tcpc::isAttached()
{
    CcStatus cc1 = CcStatus::OPEN;
    CcStatus cc2 = CcStatus::OPEN;
    if (!getCc(cc1, cc2)) return false;
    return cc1 != CcStatus::OPEN || cc2 != CcStatus::OPEN;
}

bool AXP517Tcpc::getPowerStatus(uint8_t &status)
{
    return r8(POWER_STATUS, status);
}

bool AXP517Tcpc::isVbusPresent()
{
    uint8_t status = 0;
    if (!getPowerStatus(status)) return false;
    return (status & POWER_STATUS_VBUS_PRESENT) != 0;
}

bool AXP517Tcpc::readVbusMv(uint16_t &mv)
{
    uint8_t lo = 0, hi = 0;
    if (!r8(VBUS_VOLTAGE_L, lo)) return false;
    if (!r8(VBUS_VOLTAGE_H, hi)) return false;

    uint16_t raw = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi & 0x07) << 8);
    mv = static_cast<uint16_t>(raw * 14);
    return true;
}

bool AXP517Tcpc::rxReadMessage(RxFifoMsg &out)
{
    out = RxFifoMsg{};

    uint8_t buf[32] = {0};
    if (!readNoInc(RX_BYTE_CNT, buf, sizeof(buf))) return false;

    uint8_t count = buf[0];
    uint8_t frameType = buf[1];
    if (count == 0 || count > 30 || count < 3) return false;
    if (frameType != 0x00 && frameType != 0x01) return false;

    out.byteCount = count;
    out.frameType = frameType;
    out.headerLE = static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8);

    uint8_t payloadLen = static_cast<uint8_t>(count - 3);
    if (payloadLen > sizeof(out.payload)) payloadLen = sizeof(out.payload);
    out.payloadLen = payloadLen;
    for (uint8_t i = 0; i < payloadLen; ++i) {
        out.payload[i] = buf[4 + i];
    }

    return true;
}

bool AXP517Tcpc::txSend(TransmitType type, RetryCount retry, uint16_t headerLE,
                        const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > 28) payloadLen = 28;
    if (payloadLen && !payload) return false;

    bool orderedSet = type == TransmitType::HARD_RESET || type == TransmitType::CABLE_RESET;
    if (!orderedSet) {
        uint8_t buf[31] = {0};
        buf[0] = static_cast<uint8_t>(payloadLen + 2);
        buf[1] = static_cast<uint8_t>(headerLE & 0xFF);
        buf[2] = static_cast<uint8_t>((headerLE >> 8) & 0xFF);
        for (uint8_t i = 0; i < payloadLen; ++i) {
            buf[3 + i] = payload[i];
        }
        if (!writeNoInc(TX_BYTE_CNT, buf, static_cast<uint8_t>(payloadLen + 3))) {
            return false;
        }
    }

    uint8_t retryValue = orderedSet ? 0 : static_cast<uint8_t>(retry);
    uint8_t tx = static_cast<uint8_t>(((retryValue & 0x03) << 4) |
                                      (static_cast<uint8_t>(type) & 0x07));
    return w8(TX_BUF_TRANSMIT, tx);
}

#endif
