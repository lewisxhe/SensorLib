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
 * @file      AXP517TcpcRegs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief     Register map and bit definitions for the AXP517 TCPC/USB-PD block.
 */

#pragma once
#include <stdint.h>

/**
 * @brief AXP517 TCPC register addresses, alert bits, and command constants.
 */
namespace axp517::tcpc
{
static constexpr uint8_t TCPC_VENDOR_ID       = 0xA0; ///< TCPC Vendor ID register (RO).
static constexpr uint16_t VENDOR_ID           = 0x1F3A; ///< Expected AXP517 TCPC vendor ID.

/** @name TCPC and USB-PD version registers */
///@{
static constexpr uint8_t USBTYPEC_REV_L      = 0xA6;
static constexpr uint8_t USBTYPEC_REV_H      = 0xA7;
static constexpr uint8_t USBPD_VER           = 0xA8;
static constexpr uint8_t USBPD_REV           = 0xA9;
static constexpr uint8_t PD_INTERFACE_VER    = 0xAA;
static constexpr uint8_t PD_INTERFACE_REV    = 0xAB;
///@}

/** @name Alert and alert-mask registers */
///@{
static constexpr uint8_t PD_ALERTL           = 0xB0;
static constexpr uint8_t PD_ALERTH           = 0xB1;
static constexpr uint8_t PD_ALERT_MASKL      = 0xB2;
static constexpr uint8_t PD_ALERT_MASKH      = 0xB3;
static constexpr uint8_t POWER_STATUS_MASK   = 0xB4;
static constexpr uint8_t FAULT_STATUS_MASK   = 0xB5;
///@}

/** @name TCPC control and status registers */
///@{
static constexpr uint8_t TCPC_CONTROL        = 0xB9;
static constexpr uint8_t ROLE_CONTROL        = 0xBA;
static constexpr uint8_t FAULT_CONTROL       = 0xBB;
static constexpr uint8_t POWER_CONTROL       = 0xBC;
static constexpr uint8_t CC_STATUS           = 0xBD;
static constexpr uint8_t POWER_STATUS        = 0xBE;
static constexpr uint8_t FAULT_STATUS        = 0xBF;
///@}

/** @name Command and capability registers */
///@{
static constexpr uint8_t COMMAND             = 0xC3;
static constexpr uint8_t DEVICE_CAPS_1L      = 0xC4;
static constexpr uint8_t DEVICE_CAPS_1H      = 0xC5;
static constexpr uint8_t DEVICE_CAPS_2L      = 0xC6;
static constexpr uint8_t DEVICE_CAPS_2H      = 0xC7;
///@}

/** @name Message header and receive-detection registers */
///@{
static constexpr uint8_t MESSAGE_HEADER_INFO = 0xCE;
static constexpr uint8_t RECEIVE_DETECT      = 0xCF;
///@}

/** @name VBUS measurement and threshold registers */
///@{
static constexpr uint8_t VBUS_VOLTAGE_L      = 0xD0;
static constexpr uint8_t VBUS_VOLTAGE_H      = 0xD1;
static constexpr uint8_t VBUS_SINK_DISCONNECT_THRESH = 0xD2;
static constexpr uint8_t VBUS_STOP_DISCHARGE_THRESH = 0xD4;
static constexpr uint8_t VBUS_VOLTAGE_ALARM_HI_CFG = 0xD6;
static constexpr uint8_t VBUS_VOLTAGE_ALARM_LO_CFG = 0xD8;
///@}

/** @name TCPC RX/TX FIFO registers */
///@{
static constexpr uint8_t RX_BUFFER_FIFO      = 0xDA; ///< RX FIFO read register.
static constexpr uint8_t TX_BUF_TRANSMIT     = 0xDB; ///< TX transmit control register.
static constexpr uint8_t TX_BUFFER_FIFO      = 0xDC; ///< TX FIFO write register.

static constexpr uint8_t RX_BYTE_CNT         = 0xDA; ///< RX byte-count register alias.
static constexpr uint8_t TX_BYTE_CNT         = 0xDC; ///< TX byte-count register alias.
static constexpr uint8_t TX_HDR              = 0x52; ///< TX header offset used by vendor FIFO access.
static constexpr uint8_t TX_DATA             = 0x54; ///< TX data offset used by vendor FIFO access.
static constexpr uint8_t RX_HDR              = 0x32; ///< RX header offset used by vendor FIFO access.
static constexpr uint8_t RX_DATA             = 0x34; ///< RX data offset used by vendor FIFO access.
///@}

/** @name Extended status registers */
///@{
static constexpr uint8_t EXTENDED_STATUS     = 0xC0; ///< Extended status register, including vSafe0V state.
static constexpr uint8_t ALERT_EXTENDED      = 0xC1; ///< Extended alert status register.
///@}

/** @brief PD state machine status register. */
static constexpr uint8_t PD_STATE             = 0xE3;

/** @brief Awake-enable control register. */
static constexpr uint8_t AWAKE_EN             = 0xE0;

/** @brief CC connection status register. */
static constexpr uint8_t CC_CONNECTION_STATUS = 0xE6;

/** @brief CC general control register. */
static constexpr uint8_t CC_GENERAL_CONTROL  = 0xE8;
static constexpr uint8_t SW_RESET = 0x20; ///< Software reset bit in CC_GENERAL_CONTROL.

/** @brief Static I2C address control register. */
static constexpr uint8_t TWI_ADDR_STATIC      = 0xEB;

/** @brief Clock-enable register for the CC module. */
static constexpr uint8_t CLK_EN                = 0x0B;

/** @brief Module-enable register for TCPC related modules. */
static constexpr uint8_t MODULE_EN            = 0x19;

/**
 * @brief PD_ALERTL/H status bit definitions.
 */
namespace alert
{
static constexpr uint16_t CC_STATUS       = (1u << 0);  ///< CC_STATUS changed.
static constexpr uint16_t POWER_STATUS    = (1u << 1);  ///< POWER_STATUS changed.
static constexpr uint16_t RX_STATUS       = (1u << 2);  ///< RX FIFO contains a message.
static constexpr uint16_t RX_HARD_RST     = (1u << 3);  ///< Hard Reset received.
static constexpr uint16_t TX_FAILED       = (1u << 4);  ///< Transmission failed.
static constexpr uint16_t TX_DISCARDED    = (1u << 5);  ///< Transmission discarded.
static constexpr uint16_t TX_SUCCESS      = (1u << 6);  ///< Transmission completed.
static constexpr uint16_t V_ALARM_HI      = (1u << 7);  ///< VBUS high-voltage alarm.
static constexpr uint16_t V_ALARM_LO      = (1u << 8);  ///< VBUS low-voltage alarm.
static constexpr uint16_t FAULT           = (1u << 9);  ///< Fault status changed.
static constexpr uint16_t RX_BUF_OVF      = (1u << 10); ///< RX buffer overflow.
static constexpr uint16_t VBUS_DISCNCT    = (1u << 11); ///< VBUS sink disconnect detected.
static constexpr uint16_t EXTENDED_STATUS = (1u << 13); ///< EXTENDED_STATUS changed.
static constexpr uint16_t EXTND           = (1u << 14); ///< ALERT_EXTENDED changed.
} // namespace alert

/**
 * @brief TCPCI command register values.
 */
namespace cmd
{
static constexpr uint8_t WAKE_I2C             = 0x11; ///< Wake TCPC I2C interface.
static constexpr uint8_t DISABLE_VBUS_DETECT  = 0x22; ///< Disable VBUS detection.
static constexpr uint8_t ENABLE_VBUS_DETECT   = 0x33; ///< Enable VBUS detection.
static constexpr uint8_t DISABLE_SINK_VBUS    = 0x44; ///< Disable sink VBUS path.
static constexpr uint8_t SINK_VBUS            = 0x55; ///< Enable sink VBUS path.
static constexpr uint8_t DISABLE_SRC_VBUS     = 0x66; ///< Disable source VBUS path.
static constexpr uint8_t SRC_VBUS_DEFAULT     = 0x77; ///< Source default VBUS.
static constexpr uint8_t SRC_VBUS_HIGH        = 0x88; ///< Source high VBUS.
static constexpr uint8_t LOOK4CONNECTION      = 0x99; ///< Start looking for a Type-C connection.
static constexpr uint8_t RXONEMORE            = 0xAA; ///< Re-enable one more RX message.
static constexpr uint8_t I2C_IDLE             = 0xFF; ///< Return TCPC command interface to idle.
} // namespace cmd

} // namespace axp517::tcpc
