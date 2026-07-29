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
 * @file      UsbPdDefs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
 *
 * @brief     Small USB-PD header, PDO, and RDO helpers used by AXP517.
 */
#pragma once

#include <stdint.h>

/**
 * @brief USB Power Delivery protocol helper definitions.
 */
namespace usbpd
{

/**
 * @brief Read a 16-bit little-endian integer from a byte buffer.
 * @param p Pointer to at least two bytes.
 * @return Decoded 16-bit value.
 */
inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * @brief Read a 32-bit little-endian integer from a byte buffer.
 * @param p Pointer to at least four bytes.
 * @return Decoded 32-bit value.
 */
inline uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/**
 * @brief Write a 16-bit value to a byte buffer in little-endian order.
 * @param p Pointer to at least two destination bytes.
 * @param v Value to write.
 */
inline void wr_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/**
 * @brief Write a 32-bit value to a byte buffer in little-endian order.
 * @param p Pointer to at least four destination bytes.
 * @param v Value to write.
 */
inline void wr_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/**
 * @brief USB-PD 16-bit message header wrapper.
 */
struct Header {
    /** @brief Raw 16-bit header value in little-endian host order. */
    uint16_t raw = 0;

    /** @brief Construct an empty header. */
    Header() = default;

    /**
     * @brief Construct a header from a raw value.
     * @param v Raw 16-bit header value.
     */
    explicit Header(uint16_t v) : raw(v) {}

    /** @brief Return the USB-PD message type field. */
    uint8_t  msgType()   const
    {
        return (uint8_t)(raw & 0x1F);
    }

    /** @brief Return true when the data role field is DFP. */
    bool dataRoleDfp() const
    {
        return ((raw >> 5) & 0x1) != 0;
    }

    /** @brief Return the USB-PD specification revision field. */
    uint8_t  specRev()   const
    {
        return (uint8_t)((raw >> 6) & 0x3);
    }

    /** @brief Return true when the power role field is Source. */
    bool powerRoleSource() const
    {
        return ((raw >> 8) & 0x1) != 0;
    }

    /** @brief Return the USB-PD message ID field. */
    uint8_t msgId() const
    {
        return (uint8_t)((raw >> 9) & 0x7);
    }

    /** @brief Return the number of 32-bit data objects in this message. */
    uint8_t objCount()  const
    {
        return (uint8_t)((raw >> 12) & 0x7);
    }

    /** @brief Return true when the extended message bit is set. */
    bool extended()  const
    {
        return ((raw >> 15) & 0x1) != 0;
    }

    /** @brief Set the USB-PD message type field. */
    void setMsgType(uint8_t v)
    {
        raw = (uint16_t)((raw & ~0x001F) | (v & 0x1F));
    }

    /** @brief Set the data role field. */
    void setDataRoleDfp(bool dfp)
    {
        raw = (uint16_t)((raw & ~(1u << 5)) | ((uint16_t)dfp << 5));
    }

    /** @brief Set the USB-PD specification revision field. */
    void setSpecRev(uint8_t r)
    {
        raw = (uint16_t)((raw & ~(3u << 6)) | ((uint16_t)(r & 0x3) << 6));
    }

    /** @brief Set the power role field. */
    void setPowerRoleSource(bool src)
    {
        raw = (uint16_t)((raw & ~(1u << 8)) | ((uint16_t)src << 8));
    }

    /** @brief Set the USB-PD message ID field. */
    void setMsgId(uint8_t id)
    {
        raw = (uint16_t)((raw & ~(7u << 9)) | ((uint16_t)(id & 0x7) << 9));
    }

    /** @brief Set the number of 32-bit data objects. */
    void setObjCount(uint8_t n)
    {
        raw = (uint16_t)((raw & ~(7u << 12)) | ((uint16_t)(n & 0x7) << 12));
    }

    /** @brief Set or clear the extended message bit. */
    void setExtended(bool ex)
    {
        raw = (uint16_t)((raw & ~(1u << 15)) | ((uint16_t)ex << 15));
    }
};

/**
 * @brief USB-PD specification revision values used in message headers.
 */
enum class SpecRev : uint8_t {
    Rev10 = 0, ///< USB-PD 1.0.
    Rev20 = 1, ///< USB-PD 2.0.
    Rev30 = 2, ///< USB-PD 3.0.
};

/**
 * @brief USB-PD control message type field values.
 */
enum class ControlMsg : uint8_t {
    GoodCRC      = 0x01, ///< GoodCRC.
    GotoMin      = 0x02, ///< GotoMin.
    Accept       = 0x03, ///< Accept.
    Reject       = 0x04, ///< Reject.
    Ping         = 0x05, ///< Ping.
    PS_RDY       = 0x06, ///< Power Supply Ready.
    GetSourceCap = 0x07, ///< Get Source Capabilities.
    GetSinkCap   = 0x08, ///< Get Sink Capabilities.
    DR_Swap      = 0x09, ///< Data Role Swap.
    PR_Swap      = 0x0A, ///< Power Role Swap.
    VCONN_Swap   = 0x0B, ///< VCONN Swap.
    Wait         = 0x0C, ///< Wait.
    SoftReset    = 0x0D, ///< Soft Reset.
};

/**
 * @brief USB-PD data message type field values.
 */
enum class DataMsg : uint8_t {
    SourceCapabilities = 0x01, ///< Source_Capabilities.
    Request            = 0x02, ///< Request.
    BIST               = 0x03, ///< Built-In Self Test.
    SinkCapabilities   = 0x04, ///< Sink_Capabilities.
    VendorDefined      = 0x0F, ///< Vendor Defined Message.
};

/**
 * @brief Build a USB-PD control message header.
 * @param type Control message type.
 * @param msgId Three-bit USB-PD message ID.
 * @param powerRoleSource true for Source, false for Sink.
 * @param dataRoleDfp true for DFP, false for UFP.
 * @param specRev USB-PD specification revision.
 * @return Encoded 16-bit USB-PD header.
 */
inline uint16_t buildControlHeader(
    ControlMsg type,
    uint8_t msgId,
    bool powerRoleSource,
    bool dataRoleDfp,
    SpecRev specRev
)
{
    Header h{};
    h.setExtended(false);
    h.setObjCount(0);
    h.setMsgId(msgId);
    h.setPowerRoleSource(powerRoleSource);
    h.setSpecRev((uint8_t)specRev);
    h.setDataRoleDfp(dataRoleDfp);
    h.setMsgType((uint8_t)type);
    return h.raw;
}

/**
 * @brief Build a USB-PD data message header.
 * @param type Data message type.
 * @param objCount Number of 32-bit data objects.
 * @param msgId Three-bit USB-PD message ID.
 * @param powerRoleSource true for Source, false for Sink.
 * @param dataRoleDfp true for DFP, false for UFP.
 * @param specRev USB-PD specification revision.
 * @return Encoded 16-bit USB-PD header.
 */
inline uint16_t buildDataHeader(
    DataMsg type,
    uint8_t objCount,
    uint8_t msgId,
    bool powerRoleSource,
    bool dataRoleDfp,
    SpecRev specRev
)
{
    Header h{};
    h.setExtended(false);
    h.setObjCount(objCount);
    h.setMsgId(msgId);
    h.setPowerRoleSource(powerRoleSource);
    h.setSpecRev((uint8_t)specRev);
    h.setDataRoleDfp(dataRoleDfp);
    h.setMsgType((uint8_t)type);
    return h.raw;
}

/**
 * @brief USB-PD Power Data Object type.
 */
enum class PdoType : uint8_t {
    Fixed     = 0b00, ///< Fixed supply PDO.
    Battery   = 0b01, ///< Battery supply PDO.
    Variable  = 0b10, ///< Variable supply PDO.
    Augmented = 0b11, ///< Augmented PDO.
};

/**
 * @brief Parsed fixed source PDO fields.
 */
struct FixedSourcePdo {
    uint16_t mv = 0; ///< Voltage in millivolts.
    uint16_t ma = 0; ///< Maximum current in milliamps.
    bool usbCommCapable = false; ///< USB communications capable flag.
    bool dualRolePower = false;  ///< Dual-role power flag.
};

/**
 * @brief Return the type field of a raw PDO.
 * @param pdo Raw 32-bit PDO value.
 * @return PDO type.
 */
inline PdoType pdoType(uint32_t pdo)
{
    return (PdoType)((pdo >> 30) & 0x03);
}

/**
 * @brief Parse a fixed source PDO.
 * @param pdo Raw 32-bit PDO value.
 * @param out Destination for decoded fixed-source fields.
 * @retval true if the PDO is fixed-source, false for other PDO types.
 */
inline bool parseFixedSourcePdo(uint32_t pdo, FixedSourcePdo &out)
{
    if (pdoType(pdo) != PdoType::Fixed) return false;

    uint16_t v50 = (uint16_t)((pdo >> 10) & 0x03FF);
    uint16_t i10 = (uint16_t)(pdo & 0x03FF);

    out.mv = (uint16_t)(v50 * 50);
    out.ma = (uint16_t)(i10 * 10);

    out.usbCommCapable = ((pdo >> 26) & 0x1) != 0;
    out.dualRolePower  = ((pdo >> 29) & 0x1) != 0;
    return true;
}

/**
 * @brief Build a fixed-current Request Data Object.
 *
 * The generated RDO uses the common PD2.0 fixed/variable current layout:
 * object position, capability mismatch, USB communications capable, no USB
 * suspend, operating current, and maximum operating current.
 *
 * @param objectPos1based Source PDO object position, 1 through 7.
 * @param opMa Operating current in milliamps.
 * @param maxMa Maximum operating current in milliamps.
 * @param capMismatch Set the Capability Mismatch bit.
 * @param usbCommCapable Set the USB Communications Capable bit.
 * @param noUsbSuspend Set the No USB Suspend bit.
 * @return Encoded 32-bit RDO value.
 */
inline uint32_t buildRdoFixedCurrent(
    uint8_t objectPos1based,
    uint16_t opMa,
    uint16_t maxMa,
    bool capMismatch = false,
    bool usbCommCapable = true,
    bool noUsbSuspend = true
)
{
    if (objectPos1based < 1) objectPos1based = 1;
    if (objectPos1based > 7) objectPos1based = 7;

    uint16_t opI10  = (uint16_t)(opMa / 10);
    uint16_t maxI10 = (uint16_t)(maxMa / 10);
    if (opI10 > 0x3FF) opI10 = 0x3FF;
    if (maxI10 > 0x3FF) maxI10 = 0x3FF;

    uint32_t rdo = 0;
    rdo |= ((uint32_t)(objectPos1based & 0x0F) << 28);
    rdo |= ((uint32_t)(capMismatch ? 1u : 0u) << 26);
    rdo |= ((uint32_t)(usbCommCapable ? 1u : 0u) << 25);
    rdo |= ((uint32_t)(noUsbSuspend ? 1u : 0u) << 24);
    rdo |= ((uint32_t)(opI10 & 0x3FF) << 10);
    rdo |= ((uint32_t)(maxI10 & 0x3FF) << 0);
    return rdo;
}

} // namespace usbpd
