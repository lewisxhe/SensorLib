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
 * @file      AXP517Tcpc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 * @brief     Low-level TCPCI-style access for the AXP517 Type-C/PD block.
 */
#pragma once

#include <stdint.h>
#include "AXP517Core.hpp"

/**
 * @brief AXP517 Type-C Port Controller helper.
 *
 * This class provides direct access to the AXP517 TCPC registers, alert
 * handling, CC/VBUS status, and PD RX/TX FIFOs. It intentionally stays close
 * to the TCPCI register model; policy decisions such as selecting a PDO are
 * implemented in AXP517PdNegotiator.
 */
class AXP517Tcpc
{
public:
    /**
     * @brief Construct a TCPC helper using the shared AXP517 core transport.
     * @param core Initialized AXP517 core communication object.
     */
    explicit AXP517Tcpc(AXP517Core &core) : _core(core) {}

    /**
     * @brief PD_ALERTL/H bit definitions returned by readAlert().
     */
    enum Alert : uint16_t {
        AlertCcStatus        = (1u << 0),  ///< CC_STATUS register changed.
        AlertPowerStatus     = (1u << 1),  ///< POWER_STATUS register changed.
        AlertRxStatus        = (1u << 2),  ///< RX buffer contains a SOP message.
        AlertRxHardReset     = (1u << 3),  ///< Hard Reset received.
        AlertTxFailed        = (1u << 4),  ///< Message transmission failed.
        AlertTxDiscarded     = (1u << 5),  ///< Transmission discarded due to RX activity.
        AlertTxSuccess       = (1u << 6),  ///< Message transmission completed.
        AlertVbusAlarmHi     = (1u << 7),  ///< VBUS high-voltage alarm.
        AlertVbusAlarmLo     = (1u << 8),  ///< VBUS low-voltage alarm.
        AlertFault           = (1u << 9),  ///< TCPC fault status changed.
        AlertRxBufOverflow   = (1u << 10), ///< RX buffer overflow.
        AlertVbusDisconnect  = (1u << 11), ///< VBUS sink disconnect threshold crossed.
        AlertExtendedStatus  = (1u << 13), ///< EXTENDED_STATUS register changed.
        AlertExtended        = (1u << 14), ///< ALERT_EXTENDED register changed.
    };

    /**
     * @brief TCPCI TRANSMIT register packet type values.
     */
    enum class TransmitType : uint8_t {
        SOP = 0,                ///< SOP packet.
        SOP_PRIME = 1,          ///< SOP' packet.
        SOP_DPRIME = 2,         ///< SOP'' packet.
        SOP_DBG_PRIME = 3,      ///< Debug SOP' packet.
        SOP_DBG_DPRIME = 4,     ///< Debug SOP'' packet.
        HARD_RESET = 5,         ///< Hard Reset ordered set.
        CABLE_RESET = 6,        ///< Cable Reset ordered set.
        BIST_CARRIER_MODE2 = 7, ///< BIST carrier mode 2.
    };

    /**
     * @brief TCPCI transmit retry count values.
     */
    enum class RetryCount : uint8_t {
        R0 = 0, ///< No retry.
        R1 = 1, ///< One retry.
        R2 = 2, ///< Two retries.
        R3 = 3, ///< Three retries.
    };

    /**
     * @brief Decoded sink-side CC status.
     */
    enum class CcStatus : uint8_t {
        OPEN = 0, ///< No source detected on this CC pin.
        RP_DEF,   ///< Default USB current advertisement.
        RP_1_5,   ///< 1.5 A current advertisement.
        RP_3_0,   ///< 3.0 A current advertisement.
    };

    /**
     * @brief Decoded RX FIFO message.
     */
    struct RxFifoMsg {
        uint8_t byteCount = 0;   ///< TCPCI readable byte count.
        uint8_t frameType = 0;   ///< TCPCI RX frame type.
        uint16_t headerLE = 0;   ///< USB-PD message header in little-endian order.
        uint8_t payload[28] = {0}; ///< USB-PD payload bytes.
        uint8_t payloadLen = 0;  ///< Number of valid payload bytes.
    };

    /**
     * @brief Initialize the TCPC as a USB-PD sink.
     * @retval true on success, false on register access or TCPC init failure.
     */
    bool init();

    /**
     * @brief Verify the TCPC vendor ID.
     * @retval true if the vendor ID matches a supported AXP517 TCPC ID.
     */
    bool checkVendorId();

    /**
     * @brief Read PD_ALERTL/H.
     * @param out Receives the 16-bit alert bitmask.
     * @retval true on success, false on register access failure.
     */
    bool readAlert(uint16_t &out);

    /**
     * @brief Clear PD_ALERTL/H bits using write-one-to-clear semantics.
     * @param maskRw1c Alert bits to clear.
     * @retval true on success, false on register access failure.
     */
    bool clearAlert(uint16_t maskRw1c);

    /**
     * @brief Clear all PD alert bits.
     * @retval true on success, false on register access failure.
     */
    bool clearAllAlerts() { return clearAlert(0xFFFF); }

    /**
     * @brief Read the TCPC FAULT_STATUS register.
     * @param out Receives the raw fault status value.
     * @retval true on success, false on register access failure.
     */
    bool readFaultStatus(uint8_t &out);

    /**
     * @brief Clear TCPC FAULT_STATUS bits using write-one-to-clear semantics.
     * @param maskRw1c Fault bits to clear.
     * @retval true on success, false on register access failure.
     */
    bool clearFaultStatus(uint8_t maskRw1c);

    /**
     * @brief Clear the non-PD PMIC IRQ status registers.
     * @retval true on success, false on register access failure.
     */
    bool clearPmicIrqStatus();

    /**
     * @brief Write the TCPCI MESSAGE_HEADER_INFO register.
     * @param v Raw register value.
     * @retval true on success, false on register access failure.
     */
    bool writeMessageHeaderInfo(uint8_t v);

    /**
     * @brief Write the TCPCI RECEIVE_DETECT register.
     * @param v Raw register value.
     * @retval true on success, false on register access failure.
     */
    bool writeReceiveDetect(uint8_t v);

    /**
     * @brief Reset the TX byte count and transmit command register.
     * @retval true on success, false on register access failure.
     */
    bool clearTx();

    /**
     * @brief Configure sink message-header defaults and enable SOP RX.
     * @retval true on success, false on register access failure.
     */
    bool setSinkMessageHeaderAndRx();

    /**
     * @brief Set TCPC polarity from the current CC status.
     * @retval true on success, false on register access failure.
     */
    bool setPolarityFromCc();

    /**
     * @brief Read and decode CC1/CC2 status.
     * @param cc1 Receives decoded CC1 status.
     * @param cc2 Receives decoded CC2 status.
     * @retval true on success, false on register access failure.
     */
    bool getCc(CcStatus &cc1, CcStatus &cc2);

    /**
     * @brief Check whether either CC pin indicates attachment.
     * @retval true if attached, false if detached or status read fails.
     */
    bool isAttached();

    /**
     * @brief Read POWER_STATUS.
     * @param status Receives the raw POWER_STATUS register value.
     * @retval true on success, false on register access failure.
     */
    bool getPowerStatus(uint8_t &status);

    /**
     * @brief Check whether TCPC VBUS present status is set.
     * @retval true if VBUS is present, false otherwise.
     */
    bool isVbusPresent();

    /**
     * @brief Read TCPC VBUS ADC value.
     * @param mv Receives VBUS voltage in millivolts.
     * @retval true on success, false on register access failure.
     */
    bool readVbusMv(uint16_t &mv);

    /**
     * @brief Read and decode one TCPC RX FIFO message.
     * @param out Destination for the decoded message.
     * @retval true when a valid FIFO message was read, false otherwise.
     */
    bool rxReadMessage(RxFifoMsg &out);

    /**
     * @brief Send a USB-PD packet through the TCPC TX FIFO.
     * @param type TCPCI transmit packet type.
     * @param retry TCPCI retry count.
     * @param headerLE USB-PD message header in little-endian order.
     * @param payload Optional payload pointer.
     * @param payloadLen Payload length in bytes.
     * @retval true on success, false on invalid arguments or register access failure.
     */
    bool txSend(TransmitType type, RetryCount retry, uint16_t headerLE,
                const uint8_t *payload, uint8_t payloadLen);

    /**
     * @brief Read one TCPC register.
     * @param reg Register address.
     * @param out Receives the register value.
     * @retval true on success, false on register access failure.
     */
    bool readReg(uint8_t reg, uint8_t &out)
    {
        return r8(reg, out);
    }

    /**
     * @brief Write one TCPC register.
     * @param reg Register address.
     * @param v Register value to write.
     * @retval true on success, false on register access failure.
     */
    bool writeReg(uint8_t reg, uint8_t v)
    {
        return w8(reg, v);
    }

    /** @brief Return the platform millisecond counter. */
    uint32_t millis() { return _core.millis(); }

    /** @brief Delay using the active platform HAL. */
    void delayMs(uint32_t ms) { _core.delayMs(ms); }

    /** @brief Configure a platform GPIO pin through the active HAL. */
    void pinMode(uint8_t pin, uint8_t mode) { _core.pinMode(pin, mode); }

    /** @brief Read a platform GPIO pin through the active HAL. */
    uint8_t digitalRead(uint8_t pin) { return _core.digitalRead(pin); }

private:
    /**
     * @brief Read one register through the shared core transport.
     * @param reg Register address.
     * @param out Receives the register value.
     * @retval true on success, false on register access failure.
     */
    bool r8(uint8_t reg, uint8_t &out);

    /**
     * @brief Write one register through the shared core transport.
     * @param reg Register address.
     * @param v Register value.
     * @retval true on success, false on register access failure.
     */
    bool w8(uint8_t reg, uint8_t v);

    /**
     * @brief Read a no-increment register/FIFO window.
     * @param reg Register address.
     * @param buf Destination buffer.
     * @param len Number of bytes to read.
     * @retval true on success, false on register access failure.
     */
    bool readNoInc(uint8_t reg, uint8_t *buf, uint8_t len);

    /**
     * @brief Write a no-increment register/FIFO window.
     * @param reg Register address.
     * @param buf Source buffer.
     * @param len Number of bytes to write.
     * @retval true on success, false on register access failure.
     */
    bool writeNoInc(uint8_t reg, const uint8_t *buf, uint8_t len);

private:
    AXP517Core &_core; ///< Shared AXP517 register transport.
};
