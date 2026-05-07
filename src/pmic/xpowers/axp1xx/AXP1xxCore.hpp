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
 * @file      AXP1xxCore.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 */
#pragma once
#include <stdint.h>

namespace axp1xx {

/**
 * @brief Register-bit pair identifying a power module.
 */
struct ModuleEntry {
    uint8_t reg; /**< Register address. */
    uint8_t bit; /**< Bit position within the register. */
};

/**
 * @brief Chip configuration metadata.
 */
struct CoreConfig {
    uint8_t chipIdReg;      /**< Register address for chip ID. */
    uint8_t chipIdValue;    /**< Expected chip ID value. */
    uint8_t i2cAddress;     /**< I2C slave address. */
    struct {
        const ModuleEntry *table; /**< Pointer to module entry table. */
        uint8_t count;            /**< Number of entries in the table. */
    } modules;
};

/**
 * @brief Look up a module entry by index.
 * @param table Pointer to the module entry table.
 * @param count Number of entries in the table.
 * @param index Zero-based index of the entry to retrieve.
 * @return Pointer to the ModuleEntry, or nullptr if index is out of range.
 */
inline const ModuleEntry *findEntry(const ModuleEntry *table, uint8_t count, uint8_t index)
{
    if (index >= count) return nullptr;
    return &table[index];
}

} // namespace axp1xx
