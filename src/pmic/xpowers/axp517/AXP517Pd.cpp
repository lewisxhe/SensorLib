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
 * @file      AXP517Pd.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 * @brief     Minimal USB Power Delivery protocol helper for the AXP517 TCPC.
 */
#include "AXP517Pd.hpp"

bool AXP517Pd::configureHeaderInfo(const HeaderInfoCfg &cfg)
{
    _cfg = cfg;
    // AXP517_TCPC_MSG_HDR_INFO:
    // bit3 data role, bits[2:1] rev, bit0 pwr role
    uint8_t v = 0;
    v |= (uint8_t)(((uint8_t)cfg.spec & 0x03) << 1);
    if (cfg.pr == PowerRole::Source) v |= 0x01;
    if (cfg.dr == DataRole::DFP)     v |= 0x08;
    return _tcpc.writeMessageHeaderInfo(v);
}

bool AXP517Pd::setReceiveDetectSop(bool enable)
{
    return _tcpc.writeReceiveDetect(enable ? 0x01 : 0x00); // SOP only
}

uint16_t AXP517Pd::buildHeader(uint8_t msgType, uint8_t objCnt)
{
    // PD header:
    // bit4:0 = msgType, bit5 = dataRole, bit6:7 = specRev, bit8 = powerRole
    // bit9:11 = msgId, bit12:14 = numDataObjects
    uint16_t h = 0;
    h |= (uint16_t)(msgType & 0x1F);
    h |= (uint16_t)(((uint8_t)_cfg.dr & 0x1) << 5);
    h |= (uint16_t)(((uint8_t)_cfg.spec & 0x03) << 6);
    h |= (uint16_t)(((uint8_t)_cfg.pr & 0x1) << 8);
    h |= (uint16_t)((_msgId & 0x7) << 9);
    h |= (uint16_t)((objCnt & 0x7) << 12);
    return h;
}

bool AXP517Pd::sendControl(ControlMsg msgType)
{
    uint16_t hdr = buildHeader((uint8_t)msgType, 0);
    bool ok = _tcpc.txSend(AXP517Tcpc::TransmitType::SOP,
                           AXP517Tcpc::RetryCount::R3,
                           hdr, nullptr, 0);
    if (ok) _msgId = (uint8_t)((_msgId + 1) & 0x7);
    return ok;
}

bool AXP517Pd::sendGetSourceCap()
{
    return sendControl(ControlMsg::GetSourceCap);
}

bool AXP517Pd::sendHardReset()
{
    // Hard Reset is a special ordered set - no TX buffer writes, retryCount=0.
    // Matches Linux axp517_tcpci_pd_transmit(type=TCPC_TX_HARD_RESET, msg=NULL):
    //   reg = (0 << TRANSMIT_RETRY_SHIFT) | (5 << TRANSMIT_TYPE_SHIFT)  => 0x05
    return _tcpc.txSend(AXP517Tcpc::TransmitType::HARD_RESET,
                        AXP517Tcpc::RetryCount::R0,  // retryCount=0 for Hard Reset
                        0, nullptr, 0);
}

bool AXP517Pd::sendRaw(uint8_t msgType, const uint8_t *payload, uint8_t payloadLen)
{
    uint8_t objCnt = (uint8_t)(payloadLen / 4);
    uint16_t hdr = buildHeader(msgType, objCnt);
    bool ok = _tcpc.txSend(AXP517Tcpc::TransmitType::SOP,
                           AXP517Tcpc::RetryCount::R3,
                           hdr, payload, payloadLen);
    if (ok) _msgId = (uint8_t)((_msgId + 1) & 0x7);
    return ok;
}

bool AXP517Pd::tryReceive(AXP517Tcpc::RxFifoMsg &out)
{
    return _tcpc.rxReadMessage(out);
}

bool AXP517Pd::handleFaultOnce(uint8_t &faultStatus)
{
    faultStatus = 0;
    uint16_t alert = 0;
    if (!_tcpc.readAlert(alert)) {
        return false;
    }
    if (!(alert & (1u << 9))) {
        return true;
    }

    if (!_tcpc.readFaultStatus(faultStatus)) {
        return false;
    }
    (void)_tcpc.clearFaultStatus(faultStatus);
    (void)_tcpc.clearAlert(1u << 9);
    return true;
}
