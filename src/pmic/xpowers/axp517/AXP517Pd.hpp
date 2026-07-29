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
 * @file      AXP517Pd.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 * @brief     Minimal USB Power Delivery protocol helper for the AXP517 TCPC.
 */
#pragma once

#include <stdint.h>
#include "AXP517Tcpc.hpp"

/**
 * @brief USB-PD message builder and FIFO helper for AXP517.
 *
 * This class owns only the protocol-level details such as message headers,
 * message IDs, control messages, raw data messages, and TCPC RX/TX FIFO access.
 * It does not run a complete USB-PD policy engine. Use AXP517PdNegotiator for
 * sink voltage negotiation.
 */
class AXP517Pd
{
public:
    /**
     * @brief Construct a PD helper that uses an initialized TCPC instance.
     * @param tcpc AXP517 TCPC register/FIFO access object.
     */
    explicit AXP517Pd(AXP517Tcpc &tcpc) : _tcpc(tcpc) {}

    /**
     * @brief USB-PD specification revision encoded in the message header.
     */
    enum class SpecRev : uint8_t {
        Rev10 = 0, ///< USB-PD 1.0.
        Rev20 = 1, ///< USB-PD 2.0.
        Rev30 = 2, ///< USB-PD 3.0.
    };

    /**
     * @brief USB-PD power role encoded in the message header.
     */
    enum class PowerRole : uint8_t {
        Sink = 0,   ///< Sink/UFP consumes VBUS.
        Source = 1, ///< Source/DFP supplies VBUS.
    };

    /**
     * @brief USB Type-C data role encoded in the message header.
     */
    enum class DataRole : uint8_t {
        UFP = 0, ///< Upstream Facing Port.
        DFP = 1, ///< Downstream Facing Port.
    };

    /**
     * @brief Supported USB-PD control message type values.
     */
    enum class ControlMsg : uint8_t {
        Accept       = 0x03, ///< Accept control message.
        PS_RDY       = 0x06, ///< Power Supply Ready control message.
        GetSourceCap = 0x07, ///< Get Source Capabilities control message.
        SoftReset    = 0x0D, ///< Soft Reset control message.
    };

    /**
     * @brief Local USB-PD header defaults used for transmitted messages.
     */
    struct HeaderInfoCfg {
        SpecRev spec = SpecRev::Rev20;   ///< USB-PD revision to advertise.
        PowerRole pr = PowerRole::Sink;  ///< Local power role.
        DataRole dr  = DataRole::UFP;    ///< Local data role.
    };

    /**
     * @brief Configure local message-header defaults.
     * @param cfg Header configuration to use for subsequent transmissions.
     * @retval true on success, false on register access failure.
     */
    bool configureHeaderInfo(const HeaderInfoCfg &cfg);

    /**
     * @brief Enable or disable SOP receive detection.
     * @param enable true to receive SOP messages, false to disable RX detection.
     * @retval true on success, false on register access failure.
     */
    bool setReceiveDetectSop(bool enable);

    /**
     * @brief Send a Get_Source_Cap control message.
     * @retval true on success, false on TX failure.
     */
    bool sendGetSourceCap();

    /**
     * @brief Send a USB-PD control message.
     * @param msgType Control message type to send.
     * @retval true on success, false on TX failure.
     */
    bool sendControl(ControlMsg msgType);

    /**
     * @brief Send a Hard Reset ordered set.
     *
     * This uses TCPCI transmit type 5 with no TX buffer payload and retry count 0,
     * matching the vendor Linux driver's TCPC_TX_HARD_RESET path.
     *
     * @retval true on success, false on TX failure.
     */
    bool sendHardReset();

    /**
     * @brief Send a raw USB-PD data message.
     * @param msgType USB-PD message type field.
     * @param payload Pointer to payload bytes, or nullptr when payloadLen is 0.
     * @param payloadLen Payload length in bytes. Data object payloads must be
     *                   a multiple of 4 bytes.
     * @retval true on success, false on invalid arguments or TX failure.
     */
    bool sendRaw(uint8_t msgType, const uint8_t *payload, uint8_t payloadLen);

    /**
     * @brief Try to read one pending RX FIFO message.
     * @param out Destination for the decoded TCPC RX FIFO message.
     * @retval true when a valid message was read, false otherwise.
     */
    bool tryReceive(AXP517Tcpc::RxFifoMsg &out);

    /**
     * @brief Read and clear one TCPC fault status sample.
     * @param faultStatus Receives the raw FAULT_STATUS register value.
     * @retval true on success, false on register access failure.
     */
    bool handleFaultOnce(uint8_t &faultStatus);

    /**
     * @brief Reset the local USB-PD message ID counter to zero.
     *
     * Call this after Hard Reset because the TCPC hardware resets its GoodCRC
     * counter and software message IDs must be synchronized with that state.
     */
    void resetMsgId() { _msgId = 0; }

private:
    /**
     * @brief Build a USB-PD message header from the current local configuration.
     * @param msgType USB-PD message type field.
     * @param objCnt Number of 32-bit data objects in the payload.
     * @return USB-PD header in little-endian host order.
     */
    uint16_t buildHeader(uint8_t msgType, uint8_t objCnt);

private:
    AXP517Tcpc &_tcpc;     ///< TCPC register and FIFO transport.
    HeaderInfoCfg _cfg{};  ///< Current transmit header configuration.
    uint8_t _msgId = 0;    ///< Local USB-PD transmit message ID counter.
};
