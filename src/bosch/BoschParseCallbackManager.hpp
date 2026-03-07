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
 * @file      BoschParseCallbackManager.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-07
 * @brief     Template-based sensor callback manager supporting registration, removal,
 *            and invocation of callbacks keyed by sensor ID.
 *
 * This class uses std::unordered_map for efficient lookup. It is not thread-safe;
 * external synchronization is required if used in a multithreaded environment.
 */

#pragma once

#include <stdint.h>
#include <unordered_map>
#include <vector>

/**
 * @brief Callback type for sensor data parsing.
 * @param sensor_id  Sensor identifier.
 * @param data       Pointer to the data buffer.
 * @param size       Size of the data buffer in bytes.
 * @param timestamp  Pointer to a timestamp value (may be used as input or output).
 * @param user_data  Opaque user pointer provided during registration.
 */
using SensorDataParseCallback = void (*)(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data);

/**
 * @brief Callback type for sensor meta‑events.
 * @param sensor_id  Sensor identifier.
 * @param event_id   Event identifier.
 * @param event_data Event‑specific data byte.
 * @param user_data  Opaque user pointer provided during registration.
 */
using SensorMetaEventCallback = void (*)(uint8_t sensor_id, uint8_t event_id, uint8_t event_data, void *user_data);

/**
 * @brief Template class that manages callbacks for sensors.
 * @tparam CallbackType The function pointer type of the callback, e.g.
 *                      SensorDataParseCallback or SensorMetaEventCallback.
 *
 * Multiple callbacks can be registered for the same sensor ID.
 * When adding a callback, a user_data pointer is stored and passed back
 * to the callback during invocation.
 */
template<typename CallbackType>
class BoschSensorCallbackTemplate
{
private:
    /**
     * @brief Internal entry holding a callback and its associated user data.
     */
    struct CallbackEntry {
        CallbackType callback;   ///< Function pointer to the callback.
        void *user_data;         ///< User data to be passed to the callback.
    };

    std::unordered_map<uint8_t, std::vector<CallbackEntry>> callback_map; ///< Maps sensor ID to a list of callback entries.

public:
    /// @name Constructors / Destructor / Copy / Move control
    BoschSensorCallbackTemplate() = default;
    ~BoschSensorCallbackTemplate() = default;

    // Non-copyable
    BoschSensorCallbackTemplate(const BoschSensorCallbackTemplate &) = delete;
    BoschSensorCallbackTemplate& operator = (const BoschSensorCallbackTemplate &) = delete;

    // Movable (efficient)
    BoschSensorCallbackTemplate(BoschSensorCallbackTemplate &&) noexcept = default;
    BoschSensorCallbackTemplate& operator = (BoschSensorCallbackTemplate &&) noexcept = default;


    /**
     * @brief Add a callback for a specific sensor.
     * @param sensor_id  Sensor identifier.
     * @param callback   Function pointer to the callback. Must not be null.
     * @param user_data  Opaque pointer that will be passed to the callback.
     * @return true if the callback was added, false if callback was null.
     */
    bool add(uint8_t sensor_id, CallbackType callback, void *user_data)
    {
        if (!callback) return false;
        CallbackEntry entry{callback, user_data};
        callback_map[sensor_id].push_back(entry);
        return true;
    }

    /**
     * @brief Remove all callbacks associated with a specific sensor.
     * @param sensor_id  Sensor identifier.
     * @return true if any callbacks were removed, false otherwise.
     */
    bool remove(uint8_t sensor_id)
    {
        return callback_map.erase(sensor_id) > 0;
    }

    /**
     * @brief Remove a specific callback for a given sensor.
     * @param sensor_id  Sensor identifier.
     * @param callback   The exact callback function pointer to remove.
     * @return true if the callback was found and removed, false otherwise.
     *
     * @note If multiple identical callbacks are registered for the same sensor,
     *       only the first one encountered is removed.
     */
    bool remove(uint8_t sensor_id, CallbackType callback)
    {
        auto it = callback_map.find(sensor_id);
        if (it == callback_map.end()) return false;
        auto &vec = it->second;
        for (auto vec_it = vec.begin(); vec_it != vec.end(); ++vec_it) {
            if (vec_it->callback == callback) {
                vec.erase(vec_it);
                if (vec.empty()) {
                    callback_map.erase(it);
                }
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Remove all registered callbacks for all sensors.
     */
    void clear()
    {
        callback_map.clear();
    }

    /**
     * @brief Invoke all callbacks registered for a sensor with data‑parse signature.
     * @param sensor_id  Sensor identifier.
     * @param data       Pointer to the data buffer.
     * @param size       Size of the data buffer.
     * @param timestamp  Pointer to a timestamp value (may be used as input/output).
     *
     * @note This overload participates in overload resolution only if
     *       CallbackType is SensorDataParseCallback. Otherwise it may still
     *       be present but will cause compilation errors when used.
     */
    void call(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp) const
    {
        auto it = callback_map.find(sensor_id);
        if (it != callback_map.end()) {
            for (const auto &entry : it->second) {
                entry.callback(sensor_id, data, size, timestamp, entry.user_data);
            }
        }
    }

    /**
     * @brief Invoke all callbacks registered for a sensor with meta‑event signature.
     * @param sensor_id  Sensor identifier.
     * @param event_id   Event identifier.
     * @param event_data Event‑specific data byte.
     *
     * @note This overload participates in overload resolution only if
     *       CallbackType is SensorMetaEventCallback.
     */
    void call(uint8_t sensor_id, uint8_t event_id, uint8_t event_data) const
    {
        auto it = callback_map.find(sensor_id);
        if (it != callback_map.end()) {
            for (const auto &entry : it->second) {
                entry.callback(sensor_id, event_id, event_data, entry.user_data);
            }
        }
    }

    /**
     * @brief Check whether any callbacks are registered for a given sensor.
     * @param sensor_id  Sensor identifier.
     * @return true if at least one callback exists for the sensor, false otherwise.
     */
    bool contains(uint8_t sensor_id) const
    {
        return callback_map.find(sensor_id) != callback_map.end();
    }

    /**
     * @brief Get the number of sensors that have at least one callback registered.
     * @return Number of distinct sensor IDs present in the map.
     */
    size_t size() const
    {
        return callback_map.size();
    }

    /**
     * @brief Check whether no callbacks are registered for any sensor.
     * @return true if the manager is empty, false otherwise.
     */
    bool empty() const
    {
        return callback_map.empty();
    }
};

/**
 * @brief Concrete manager for SensorDataParseCallback callbacks.
 */
using BoschParseCallbackManager = BoschSensorCallbackTemplate<SensorDataParseCallback>;

/**
 * @brief Concrete manager for SensorMetaEventCallback callbacks.
 */
using BoschMetaEventCallbackManager = BoschSensorCallbackTemplate<SensorMetaEventCallback>;
