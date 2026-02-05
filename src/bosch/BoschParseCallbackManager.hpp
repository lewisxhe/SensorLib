/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      BoschParseCallbackManager.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-31
 */
#pragma once

#include <stdint.h>
#include <vector>

using SensorDataParseCallback = void (*)(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data);

class BoschParseCallbackManager
{
private:
    struct Entry {
        uint8_t sensor_id;
        SensorDataParseCallback callback;
        void *user_data;
        Entry() : sensor_id(0), callback(nullptr), user_data(nullptr) {}
    };

    std::vector<Entry> entries;

public:
    BoschParseCallbackManager() = default;
    ~BoschParseCallbackManager() = default;
    BoschParseCallbackManager(const BoschParseCallbackManager &) = delete;
    BoschParseCallbackManager &operator=(const BoschParseCallbackManager &) = delete;
    BoschParseCallbackManager(BoschParseCallbackManager &&) = default;
    BoschParseCallbackManager &operator=(BoschParseCallbackManager &&) = default;

    bool add(uint8_t sensor_id, SensorDataParseCallback callback, void *user_data)
    {
        if (!callback) {
            return false;
        }
        Entry newEntry;
        newEntry.sensor_id = sensor_id;
        newEntry.callback = callback;
        newEntry.user_data = user_data;
        entries.push_back(newEntry);
        return true;
    }

    bool remove(uint8_t sensor_id, SensorDataParseCallback callback)
    {
        if (!callback) {
            return false;
        }
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (it->callback == callback && it->sensor_id == sensor_id) {
                entries.erase(it);
                break;
            }
        }
        return true;
    }

    void call(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp)
    {
        for (uint32_t i = 0; i < entries.size(); i++) {
            if (entries[i].callback) {
                if (entries[i].sensor_id == sensor_id) {
                    entries[i].callback(sensor_id, data, size, timestamp, entries[i].user_data);
                }
            }
        }
    }

    void removeAll()
    {
        entries.clear();
    }
};
