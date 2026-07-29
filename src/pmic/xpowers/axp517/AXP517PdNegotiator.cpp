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
 * @file      AXP517PdNegotiator.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 */
#include "AXP517PdNegotiator.hpp"
#include "UsbPdDefs.hpp"
#include "../../../platform/SensorLibLog.hpp"

using namespace usbpd;

namespace
{
static constexpr uint8_t MSG_SOURCE_CAP = 0x01;
static constexpr uint8_t MSG_REQUEST    = 0x02;
static constexpr uint8_t MSG_ACCEPT     = 0x03;
static constexpr uint8_t MSG_REJECT     = 0x04;
static constexpr uint8_t MSG_PS_RDY     = 0x06;

#if defined(INPUT_PULLUP)
static constexpr uint8_t IRQ_INPUT_MODE = INPUT_PULLUP;
#elif defined(INPUT)
static constexpr uint8_t IRQ_INPUT_MODE = INPUT;
#else
static constexpr uint8_t IRQ_INPUT_MODE = 0;
#endif
static constexpr uint8_t IRQ_ACTIVE_LOW = 0;
static constexpr uint8_t MAX_TX_RETRY   = 100;
static constexpr uint16_t ACTIONABLE_ALERT_MASK =
    AXP517Tcpc::AlertCcStatus |
    AXP517Tcpc::AlertPowerStatus |
    AXP517Tcpc::AlertRxStatus |
    AXP517Tcpc::AlertRxHardReset |
    AXP517Tcpc::AlertTxFailed |
    AXP517Tcpc::AlertTxDiscarded |
    AXP517Tcpc::AlertTxSuccess |
    AXP517Tcpc::AlertVbusDisconnect;

static bool configureSinkRx(AXP517Pd &pd)
{
    AXP517Pd::HeaderInfoCfg hi{};
    return pd.configureHeaderInfo(hi) && pd.setReceiveDetectSop(true);
}
} // namespace

AXP517PdNegotiator::AXP517PdNegotiator(AXP517Tcpc &tcpc, AXP517Pd &pd)
    : _tcpc(tcpc), _pd(pd)
{
}

bool AXP517PdNegotiator::setIrqPin(int irqPin)
{
    if (irqPin < 0) {
        _irqPin = -1;
        return false;
    }

    _irqPin = irqPin;
    _tcpc.pinMode(static_cast<uint8_t>(_irqPin), IRQ_INPUT_MODE);
    return true;
}

bool AXP517PdNegotiator::initSink(int irqPin)
{
    if (!setIrqPin(irqPin)) {
        SENSORLIB_LOG_E("AXP517 PD requires an IRQ pin");
        return false;
    }

    if (!_tcpc.checkVendorId()) {
        SENSORLIB_LOG_E("AXP517 TCPC vendor id mismatch");
        return false;
    }

    if (!_tcpc.init()) {
        SENSORLIB_LOG_E("AXP517 TCPC init failed");
        return false;
    }

    if (!configureSinkRx(_pd)) {
        SENSORLIB_LOG_E("AXP517 PD sink config failed");
        return false;
    }

    _pd.resetMsgId();
    _tcpc.clearPmicIrqStatus();
    _state = State::WaitSourceCap;
    _sinkReady = true;
    _txPending = false;
    _txRetry = 0;
    return true;
}

void AXP517PdNegotiator::reset()
{
    _pd.resetMsgId();
    _state = _sinkReady ? State::WaitSourceCap : State::Idle;
    _txPending = false;
    _txRetry = 0;
}

bool AXP517PdNegotiator::negotiate(const RequestParams &req, SourceCaps *outCaps,
                                   uint32_t timeoutMs)
{
    if (!hasIrqPin()) {
        SENSORLIB_LOG_E("AXP517 PD request skipped: IRQ pin not configured");
        return false;
    }

    if (!_sinkReady) {
        if (!initSink(_irqPin)) return false;
    } else {
        if (!_tcpc.init()) {
            SENSORLIB_LOG_E("AXP517 TCPC init failed");
            return false;
        }
        if (!configureSinkRx(_pd)) {
            SENSORLIB_LOG_E("AXP517 PD sink config failed");
            return false;
        }
        _tcpc.clearPmicIrqStatus();
    }

    reset();

    uint16_t pending = 0;
    if (_tcpc.readAlert(pending) && pending) {
        Event event = Event::None;
        if (!handleIrq(req, event, outCaps)) return false;
        if (isDone()) return isSuccess();
    }

    uint32_t deadline = _tcpc.millis() + timeoutMs;
    while (_tcpc.millis() < deadline) {
        if (_tcpc.digitalRead(static_cast<uint8_t>(_irqPin)) == IRQ_ACTIVE_LOW) {
            Event event = Event::None;
            if (!service(req, event, outCaps)) return false;
            if (isDone()) return isSuccess();
        }
        _tcpc.delayMs(1);
    }

    SENSORLIB_LOG_W("AXP517 PD negotiation timeout");
    return false;
}

bool AXP517PdNegotiator::service(const RequestParams &req, Event &outEvent,
                                 SourceCaps *outCaps)
{
    outEvent = Event::None;
    if (!hasIrqPin()) {
        SENSORLIB_LOG_E("AXP517 PD IRQ pin not configured");
        return false;
    }

    if (_tcpc.digitalRead(static_cast<uint8_t>(_irqPin)) != IRQ_ACTIVE_LOW) {
        return true;
    }

    while (true) {
        if (!handleIrq(req, outEvent, outCaps)) return false;
        if (isDone()) return true;

        uint16_t alert = 0;
        if (!_tcpc.readAlert(alert)) return false;
        if (alert == 0) break;
        if ((alert & ACTIONABLE_ALERT_MASK) == 0) break;
        if (_tcpc.digitalRead(static_cast<uint8_t>(_irqPin)) != IRQ_ACTIVE_LOW) break;
    }

    return true;
}

bool AXP517PdNegotiator::handleIrq(const RequestParams &req, Event &outEvent,
                                   SourceCaps *outCaps)
{
    outEvent = Event::None;
    if (!hasIrqPin()) {
        SENSORLIB_LOG_E("AXP517 PD IRQ pin not configured");
        return false;
    }

    if (!_tcpc.clearPmicIrqStatus()) return false;

    uint16_t alert = 0;
    if (!_tcpc.readAlert(alert)) return false;
    if (alert == 0) return true;

    uint16_t clearMask = alert & ~AXP517Tcpc::AlertRxStatus;
    if (clearMask && !_tcpc.clearAlert(clearMask)) return false;

    if (alert & AXP517Tcpc::AlertFault) {
        uint8_t fault = 0;
        if (!_tcpc.readFaultStatus(fault)) return false;
        if (fault && !_tcpc.clearFaultStatus(fault)) return false;
    }

    if (alert & AXP517Tcpc::AlertRxHardReset) {
        _tcpc.clearTx();
        _tcpc.setSinkMessageHeaderAndRx();
        _tcpc.clearAllAlerts();
        reset();
        outEvent = Event::HardReset;
        return true;
    }

    if (alert & (AXP517Tcpc::AlertCcStatus |
                 AXP517Tcpc::AlertPowerStatus |
                 AXP517Tcpc::AlertVbusDisconnect)) {
        if (!_tcpc.isVbusPresent() || !_tcpc.isAttached()) {
            _tcpc.clearTx();
            reset();
            outEvent = Event::CcDetached;
            return true;
        }
    }

    if (alert & AXP517Tcpc::AlertCcStatus) {
        _tcpc.writeReceiveDetect(0x01);
        if (!_tcpc.setPolarityFromCc()) return false;
        outEvent = Event::CcChanged;
    }

    if (alert & AXP517Tcpc::AlertTxSuccess) {
        _txPending = false;
        _txRetry = 0;
        _tcpc.writeReceiveDetect(0x01);
        outEvent = Event::TxSuccess;
    }

    if (alert & AXP517Tcpc::AlertTxDiscarded) {
        _tcpc.clearTx();
        _txPending = false;
        if (++_txRetry >= MAX_TX_RETRY) {
            reset();
        } else {
            _tcpc.writeReceiveDetect(0x01);
        }
        outEvent = Event::TxDiscarded;
    }

    if (alert & AXP517Tcpc::AlertTxFailed) {
        _tcpc.clearTx();
        _txPending = false;
        if (++_txRetry >= MAX_TX_RETRY) {
            reset();
        } else {
            _tcpc.writeReceiveDetect(0x01);
        }
        outEvent = Event::TxFailed;
    }

    if (alert & AXP517Tcpc::AlertRxStatus) {
        AXP517Tcpc::RxFifoMsg rx{};
        bool rxOk = _tcpc.rxReadMessage(rx);
        (void)_tcpc.clearAlert(AXP517Tcpc::AlertRxStatus);
        if (!rxOk) return true;

        Header hdr(rx.headerLE);
        uint8_t msgType = hdr.msgType();
        uint8_t objCount = hdr.objCount();

        if (msgType == MSG_SOURCE_CAP && objCount > 0) {
            outEvent = Event::SourceCapabilities;
            if (_state == State::WaitSourceCap || (_state == State::WaitAccept && !_txPending)) {
                SourceCaps caps{};
                if (!parseSourceCaps(rx, caps)) {
                    _state = State::Failed;
                    return true;
                }
                if (outCaps) *outCaps = caps;

                if (!requestFromCaps(caps, req)) {
                    _state = State::Failed;
                    return true;
                }

                _txPending = true;
                _state = State::WaitAccept;
                outEvent = Event::RequestSent;
            }
        } else if (_state == State::WaitAccept && msgType == MSG_ACCEPT && objCount == 0) {
            _txPending = false;
            _state = State::WaitPsRdy;
            _tcpc.writeReceiveDetect(0x01);
            outEvent = Event::Accept;
        } else if (_state == State::WaitAccept && msgType == MSG_REJECT && objCount == 0) {
            _state = State::Failed;
            outEvent = Event::Reject;
        } else if (_state == State::WaitPsRdy && msgType == MSG_PS_RDY && objCount == 0) {
            _state = State::Success;
            outEvent = Event::PsRdy;
        }
    }

    return true;
}

bool AXP517PdNegotiator::parseSourceCaps(const AXP517Tcpc::RxFifoMsg &rx, SourceCaps &outCaps)
{
    outCaps = SourceCaps{};

    Header hdr(rx.headerLE);
    if (hdr.msgType() != MSG_SOURCE_CAP || hdr.objCount() == 0) return false;

    outCaps.rawHeaderLE = rx.headerLE;
    outCaps.totalPdoCount = hdr.objCount();

    uint8_t parseCount = static_cast<uint8_t>(rx.payloadLen / 4);
    if (parseCount > outCaps.totalPdoCount) parseCount = outCaps.totalPdoCount;
    if (parseCount > 7) parseCount = 7;

    for (uint8_t i = 0; i < parseCount; ++i) {
        uint32_t pdo = le32(&rx.payload[i * 4]);
        FixedSourcePdo fixed{};
        if (!parseFixedSourcePdo(pdo, fixed)) continue;

        FixedOffer &offer = outCaps.fixed[outCaps.fixedCount++];
        offer.mv = fixed.mv;
        offer.maMax = fixed.ma;
        offer.pdoIndex1Based = static_cast<uint8_t>(i + 1);
        offer.rawPdo = pdo;
        if (outCaps.fixedCount >= 7) break;
    }

    return outCaps.fixedCount > 0;
}

bool AXP517PdNegotiator::selectFixedOffer(const SourceCaps &caps, const RequestParams &req,
                                          FixedOffer &outOffer)
{
    if (caps.fixedCount == 0) return false;

    if (req.pdoIndex1Based > 0) {
        for (uint8_t i = 0; i < caps.fixedCount; ++i) {
            if (caps.fixed[i].pdoIndex1Based == req.pdoIndex1Based) {
                outOffer = caps.fixed[i];
                return true;
            }
        }
        return false;
    }

    if (req.targetMv > 0) {
        for (uint8_t i = 0; i < caps.fixedCount; ++i) {
            if (caps.fixed[i].mv == req.targetMv) {
                outOffer = caps.fixed[i];
                return true;
            }
        }
        return false;
    }

    outOffer = caps.fixed[0];
    return true;
}

bool AXP517PdNegotiator::requestFromCaps(const SourceCaps &caps, const RequestParams &req)
{
    FixedOffer offer{};
    if (!selectFixedOffer(caps, req, offer)) return false;

    if (!_tcpc.clearTx()) return false;
    if (!_tcpc.clearAllAlerts()) return false;
    return sendRequest(offer, req);
}

bool AXP517PdNegotiator::sendRequest(const FixedOffer &offer, const RequestParams &req)
{
    uint16_t opMa = req.opMa ? req.opMa : offer.maMax;
    if (offer.maMax && opMa > offer.maMax) opMa = offer.maMax;

    uint16_t maxMa = req.maxMa ? req.maxMa : opMa;
    if (offer.maMax && maxMa > offer.maMax) maxMa = offer.maMax;

    uint32_t rdo = buildRdoFixedCurrent(offer.pdoIndex1Based, opMa, maxMa,
                                        req.capMismatch, req.usbCommCapable,
                                        req.noUsbSuspend);
    uint8_t payload[4] = {0};
    wr_le32(payload, rdo);

    return _pd.sendRaw(MSG_REQUEST, payload, sizeof(payload));
}
