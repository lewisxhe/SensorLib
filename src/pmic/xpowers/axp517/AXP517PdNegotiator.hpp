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
 * @file      AXP517PdNegotiator.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 * @brief     Simple USB-PD sink voltage negotiator for AXP517.
 */
#pragma once

#include <stdint.h>
#include "AXP517Pd.hpp"
#include "AXP517Tcpc.hpp"

/**
 * @brief Minimal interrupt-driven USB-PD sink negotiation state machine.
 *
 * The AXP517 TCPC delivers USB-PD messages through an IRQ-driven RX FIFO. A
 * valid interrupt pin is therefore required before requesting a PD voltage.
 * This class requests a fixed PDO by voltage or PDO index, waits for Accept and
 * PS_RDY, and exposes parsed source capabilities for diagnostics.
 */
class AXP517PdNegotiator
{
public:
    /**
     * @brief Construct a negotiator from the low-level TCPC and PD helpers.
     * @param tcpc TCPC register/FIFO access object.
     * @param pd USB-PD message helper.
     */
    explicit AXP517PdNegotiator(AXP517Tcpc &tcpc, AXP517Pd &pd);

    /**
     * @brief One fixed supply PDO offered by the source.
     */
    struct FixedOffer {
        uint16_t mv = 0;             ///< Fixed supply voltage in millivolts.
        uint16_t maMax = 0;          ///< Maximum current in milliamps.
        uint8_t pdoIndex1Based = 0;  ///< USB-PD object position, 1 through 7.
        uint32_t rawPdo = 0;         ///< Raw 32-bit PDO value.
    };

    /**
     * @brief Parsed Source_Capabilities message.
     */
    struct SourceCaps {
        FixedOffer fixed[7] = {}; ///< Parsed fixed PDO offers.
        uint8_t fixedCount = 0;   ///< Number of valid entries in fixed[].
        uint8_t totalPdoCount = 0; ///< Total PDO count advertised in the message.
        uint16_t rawHeaderLE = 0; ///< Raw Source_Capabilities header.
    };

    /**
     * @brief Requested sink operating point.
     */
    struct RequestParams {
        uint8_t pdoIndex1Based = 0; ///< Optional explicit source PDO index, 1 through 7.
        uint16_t targetMv = 0;      ///< Optional fixed PDO voltage to select.
        uint16_t opMa = 0;         ///< Optional operating current; 0 uses source maximum.
        uint16_t maxMa = 0;        ///< Optional maximum current; 0 uses opMa.
        bool capMismatch = false;  ///< Set Capability Mismatch in the RDO.
        bool usbCommCapable = false; ///< Set USB Communications Capable in the RDO.
        bool noUsbSuspend = false; ///< Set No USB Suspend in the RDO.
    };

    /**
     * @brief Negotiation state.
     */
    enum class State : uint8_t {
        Idle,          ///< Not initialized or inactive.
        WaitSourceCap, ///< Waiting for Source_Capabilities.
        WaitAccept,    ///< Request sent; waiting for Accept.
        WaitPsRdy,     ///< Accept received; waiting for PS_RDY.
        Success,       ///< Requested contract is ready.
        Failed,        ///< Negotiation failed.
    };

    /**
     * @brief Event reported by service() and handleIrq().
     */
    enum class Event : uint8_t {
        None,               ///< No meaningful state change.
        SourceCapabilities, ///< Source_Capabilities received.
        RequestSent,        ///< Request message transmitted.
        TxSuccess,          ///< TCPC reported TX success.
        TxFailed,           ///< TCPC reported TX failure.
        TxDiscarded,        ///< TCPC discarded TX due to RX activity.
        Accept,             ///< Accept control message received.
        Reject,             ///< Reject control message received.
        PsRdy,              ///< PS_RDY received; contract is active.
        HardReset,          ///< Hard Reset received.
        CcDetached,         ///< CC or VBUS detach detected.
        CcChanged,          ///< CC state changed while attached.
    };

    /**
     * @brief Configure the PMIC/TCPC IRQ pin used for PD negotiation.
     * @param irqPin Platform GPIO number connected to PMIC_IRQ.
     * @retval true on success, false when irqPin is invalid.
     */
    bool setIrqPin(int irqPin);

    /**
     * @brief Check whether a valid IRQ pin is configured.
     * @retval true if an IRQ pin has been configured.
     */
    bool hasIrqPin() const { return _irqPin >= 0; }

    /**
     * @brief Initialize the TCPC sink and configure the PD receive path.
     * @param irqPin Platform GPIO number connected to PMIC_IRQ.
     * @retval true on success, false on invalid IRQ pin or TCPC setup failure.
     */
    bool initSink(int irqPin);

    /**
     * @brief Run a blocking fixed PDO negotiation.
     * @param req Requested operating point.
     * @param outCaps Optional destination for parsed source capabilities.
     * @param timeoutMs Timeout in milliseconds.
     * @retval true when a PD contract reaches PS_RDY, false otherwise.
     */
    bool negotiate(const RequestParams &req, SourceCaps *outCaps = nullptr,
                   uint32_t timeoutMs = 6000);

    /**
     * @brief Service pending IRQ-driven PD events.
     * @param req Requested operating point.
     * @param outEvent Receives the most recent negotiator event.
     * @param outCaps Optional destination for parsed source capabilities.
     * @retval true on success, false on register/FIFO access failure.
     */
    bool service(const RequestParams &req, Event &outEvent,
                 SourceCaps *outCaps = nullptr);

    /**
     * @brief Process one TCPC IRQ status sample.
     * @param req Requested operating point.
     * @param outEvent Receives the event produced by this IRQ sample.
     * @param outCaps Optional destination for parsed source capabilities.
     * @retval true on success, false on register/FIFO access failure.
     */
    bool handleIrq(const RequestParams &req, Event &outEvent,
                   SourceCaps *outCaps = nullptr);

    /**
     * @brief Send a Request message selected from parsed source capabilities.
     * @param caps Parsed source capabilities.
     * @param req Requested operating point.
     * @retval true on success, false when no matching offer exists or TX fails.
     */
    bool requestFromCaps(const SourceCaps &caps, const RequestParams &req);

    /**
     * @brief Parse fixed PDOs from a Source_Capabilities RX message.
     * @param rx Raw TCPC RX FIFO message.
     * @param outCaps Destination for parsed capabilities.
     * @retval true when at least one fixed PDO was parsed.
     */
    bool parseSourceCaps(const AXP517Tcpc::RxFifoMsg &rx, SourceCaps &outCaps);

    /**
     * @brief Select one fixed PDO offer matching the request.
     * @param caps Parsed source capabilities.
     * @param req Requested operating point.
     * @param outOffer Receives the selected offer.
     * @retval true when a matching fixed offer was found.
     */
    bool selectFixedOffer(const SourceCaps &caps, const RequestParams &req, FixedOffer &outOffer);

    /**
     * @brief Return the current negotiation state.
     */
    State state() const { return _state; }

    /**
     * @brief Check whether negotiation completed successfully.
     */
    bool isSuccess() const { return _state == State::Success; }

    /**
     * @brief Check whether negotiation reached a terminal state.
     */
    bool isDone() const { return _state == State::Success || _state == State::Failed; }

    /**
     * @brief Reset message IDs and return to the initial wait state.
     */
    void reset();

private:
    /**
     * @brief Build and transmit a fixed-current Request Data Object.
     * @param offer Selected source fixed PDO.
     * @param req Requested operating point.
     * @retval true on success, false on TX failure.
     */
    bool sendRequest(const FixedOffer &offer, const RequestParams &req);

private:
    AXP517Tcpc &_tcpc;           ///< TCPC register and FIFO transport.
    AXP517Pd &_pd;               ///< USB-PD message helper.
    State _state = State::Idle;  ///< Current negotiation state.
    int _irqPin = -1;            ///< Platform GPIO number connected to PMIC_IRQ.
    bool _sinkReady = false;     ///< true after initSink() succeeds.
    bool _txPending = false;     ///< true while waiting for a TX completion alert.
    uint8_t _txRetry = 0;        ///< Current TX retry attempt count.
};
