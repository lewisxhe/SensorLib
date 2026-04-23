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
 */

#pragma once
#include <stdint.h>

namespace axp517::tcpc
{
///< TCPC Vendor ID (RO)
static constexpr uint8_t TCPC_VENDOR_ID       = 0xA0;
static constexpr uint16_t VENDOR_ID           = 0x1F3A;

// Versions
static constexpr uint8_t USBTYPEC_REV_L      = 0xA6;
static constexpr uint8_t USBTYPEC_REV_H      = 0xA7;
static constexpr uint8_t USBPD_VER           = 0xA8;
static constexpr uint8_t USBPD_REV           = 0xA9;
static constexpr uint8_t PD_INTERFACE_VER    = 0xAA;
static constexpr uint8_t PD_INTERFACE_REV    = 0xAB;

// Alerts + masks
static constexpr uint8_t PD_ALERTL           = 0xB0;
static constexpr uint8_t PD_ALERTH           = 0xB1;
static constexpr uint8_t PD_ALERT_MASKL      = 0xB2;
static constexpr uint8_t PD_ALERT_MASKH      = 0xB3;
static constexpr uint8_t POWER_STATUS_MASK   = 0xB4;
static constexpr uint8_t FAULT_STATUS_MASK   = 0xB5;

// TCPC controls
static constexpr uint8_t TCPC_CONTROL        = 0xB9;
static constexpr uint8_t ROLE_CONTROL        = 0xBA;
static constexpr uint8_t FAULT_CONTROL       = 0xBB;
static constexpr uint8_t POWER_CONTROL       = 0xBC;
static constexpr uint8_t CC_STATUS           = 0xBD;
static constexpr uint8_t POWER_STATUS        = 0xBE;
static constexpr uint8_t FAULT_STATUS        = 0xBF;

// Command/caps
static constexpr uint8_t COMMAND             = 0xC3;
static constexpr uint8_t DEVICE_CAPS_1L      = 0xC4;
static constexpr uint8_t DEVICE_CAPS_1H      = 0xC5;
static constexpr uint8_t DEVICE_CAPS_2L      = 0xC6;
static constexpr uint8_t DEVICE_CAPS_2H      = 0xC7;

// Message header & receive detect
static constexpr uint8_t MESSAGE_HEADER_INFO = 0xCE;
static constexpr uint8_t RECEIVE_DETECT      = 0xCF;

// VBUS voltage
static constexpr uint8_t VBUS_VOLTAGE_L      = 0xD0;
static constexpr uint8_t VBUS_VOLTAGE_H      = 0xD1;

// RX/TX FIFOs
static constexpr uint8_t RX_BUFFER_FIFO      = 0xDA; // read FIFO
static constexpr uint8_t TX_BUF_TRANSMIT     = 0xDB; // write config
static constexpr uint8_t TX_BUFFER_FIFO      = 0xDC; // write FIFO

// Extended status (for vbus_vsafe0v)
static constexpr uint8_t EXTENDED_STATUS   = 0xC0;
static constexpr uint8_t ALERT_EXTENDED    = 0xC1;

// VBUS thresholds
static constexpr uint8_t VBUS_SINK_DISCONNECT_THRESH = 0xD2;
static constexpr uint8_t VBUS_STOP_DISCHARGE_THRESH = 0xD4;
static constexpr uint8_t VBUS_VOLTAGE_ALARM_HI_CFG = 0xD6;
static constexpr uint8_t VBUS_VOLTAGE_ALARM_LO_CFG = 0xD8;

// RX byte count + frame type
static constexpr uint8_t RX_BYTE_CNT       = 0xDA;

// TX byte count + HDR + DATA
static constexpr uint8_t TX_BYTE_CNT       = 0xDC;
static constexpr uint8_t TX_HDR            = 0x52;
static constexpr uint8_t TX_DATA          = 0x54;

// RX HDR + DATA (alternate addressing)
static constexpr uint8_t RX_HDR            = 0x32;
static constexpr uint8_t RX_DATA          = 0x34;

// PD_STATE register
static constexpr uint8_t PD_STATE             = 0xE3;

// AWAKE_EN register
static constexpr uint8_t AWAKE_EN             = 0xE0;

// CC_GENERAL_CONTROL
static constexpr uint8_t CC_GENERAL_CONTROL  = 0xE8;

// CLK_EN for CC module
static constexpr uint8_t CLK_EN                = 0x0B;

// MODULE_EN
static constexpr uint8_t MODULE_EN            = 0x19;

// Alert status bit definitions
namespace alert
{
static constexpr uint16_t CC_STATUS       = (1u << 0);
static constexpr uint16_t POWER_STATUS    = (1u << 1);
static constexpr uint16_t RX_STATUS       = (1u << 2);
static constexpr uint16_t RX_HARD_RST     = (1u << 3);
static constexpr uint16_t TX_FAILED       = (1u << 4);
static constexpr uint16_t TX_DISCARDED    = (1u << 5);
static constexpr uint16_t TX_SUCCESS     = (1u << 6);
static constexpr uint16_t V_ALARM_HI     = (1u << 7);
static constexpr uint16_t V_ALARM_LO     = (1u << 8);
static constexpr uint16_t FAULT          = (1u << 9);
static constexpr uint16_t RX_BUF_OVF     = (1u << 10);
static constexpr uint16_t VBUS_DISCNCT  = (1u << 11);
static constexpr uint16_t EXTENDED_STATUS = (1u << 13);
static constexpr uint16_t EXTND         = (1u << 14);
} // namespace alert

// TCPC commands
namespace cmd
{
static constexpr uint8_t WAKE_I2C            = 0x11;
static constexpr uint8_t DISABLE_VBUS_DETECT  = 0x22;
static constexpr uint8_t ENABLE_VBUS_DETECT = 0x33;
static constexpr uint8_t DISABLE_SINK_VBUS  = 0x44;
static constexpr uint8_t SINK_VBUS         = 0x55;
static constexpr uint8_t DISABLE_SRC_VBUS  = 0x66;
static constexpr uint8_t SRC_VBUS_DEFAULT  = 0x77;
static constexpr uint8_t SRC_VBUS_HIGH    = 0x88;
static constexpr uint8_t LOOK4CONNECTION   = 0x99;
static constexpr uint8_t RXONEMORE        = 0xAA;
static constexpr uint8_t I2C_IDLE        = 0xFF;
} // namespace cmd

} // namespace axp517::tcpc
