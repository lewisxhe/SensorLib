/**
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
 * This class automatically selects an appropriate container based on the platform:
 * - On ArduinoIDE nRF52 (or if BOSCH_FORCE_VECTOR_FALLBACK is defined), a std::vector-based
 *   implementation is used for maximum compatibility.
 * - On platformio platform uses unordered map by default.
 * - Otherwise, std::unordered_map (or std::map if forced) is used for efficient lookup.
 *
 * Note: The vector-based implementation is not thread-safe and provides O(n) lookup.
 *       The size() method returns the total number of registered callbacks, not the
 *       number of distinct sensor IDs.
 */

#pragma once

#include <stdint.h>
#include <vector>

// -----------------------------------------------------------------------------
// Platform detection and container selection
// -----------------------------------------------------------------------------

// If we are on nRF52 and not explicitly forcing map usage, fall back to vector.
#if (defined(ARDUINO_ARCH_NRF52) || defined(NRF52) || defined(__NRF52__)) && !defined(BOSCH_FORCE_USE_MAP) && !defined(PLATFORMIO)
#define BOSCH_USE_VECTOR_FALLBACK
#endif

// User can force vector fallback regardless of platform.
#ifdef BOSCH_FORCE_VECTOR_FALLBACK
#define BOSCH_USE_VECTOR_FALLBACK
#endif

// If not using vector, decide between unordered_map and map.
#ifndef BOSCH_USE_VECTOR_FALLBACK
#if (defined(ARDUINO_ARCH_NRF52) || defined(NRF52) || defined(__NRF52__) || defined(BOSCH_FORCE_USE_MAP)) && !defined(PLATFORMIO)
#include <map>
template<typename Key, typename Value>
using BoschCallbackMap = std::map<Key, Value>;
#else
#include <unordered_map>
template<typename Key, typename Value>
using BoschCallbackMap = std::unordered_map<Key, Value>;
#endif
#endif

// -----------------------------------------------------------------------------
// Callback type definitions
// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// Main template class
// -----------------------------------------------------------------------------

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
    // -------------------------------------------------------------------------
    // Internal storage (selected by platform)
    // -------------------------------------------------------------------------
#ifdef BOSCH_USE_VECTOR_FALLBACK
    /**
     * @brief Entry for vector-based storage.
     */
    struct Entry {
        uint8_t sensor_id;      ///< Sensor identifier.
        CallbackType callback;   ///< Function pointer to the callback.
        void *user_data;         ///< User data to be passed to the callback.
    };

    std::vector<Entry> entries; ///< Linear list of all registered callbacks.

public:
    // -------------------------------------------------------------------------
    // Vector-based implementation
    // -------------------------------------------------------------------------
    bool add(uint8_t sensor_id, CallbackType callback, void *user_data)
    {
        if (!callback) return false;
        entries.push_back({sensor_id, callback, user_data});
        return true;
    }

    bool remove(uint8_t sensor_id)
    {
        bool removed = false;
        for (auto it = entries.begin(); it != entries.end(); ) {
            if (it->sensor_id == sensor_id) {
                it = entries.erase(it);
                removed = true;
            } else {
                ++it;
            }
        }
        return removed;
    }

    bool remove(uint8_t sensor_id, CallbackType callback)
    {
        for (auto it = entries.begin(); it != entries.end(); ++it) {
            if (it->sensor_id == sensor_id && it->callback == callback) {
                entries.erase(it);
                return true;
            }
        }
        return false;
    }

    void clear()
    {
        entries.clear();
    }

    void call(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp) const
    {
        for (const auto &entry : entries) {
            if (entry.sensor_id == sensor_id) {
                entry.callback(sensor_id, data, size, timestamp, entry.user_data);
            }
        }
    }

    void call(uint8_t sensor_id, uint8_t event_id, uint8_t event_data) const
    {
        for (const auto &entry : entries) {
            if (entry.sensor_id == sensor_id) {
                entry.callback(sensor_id, event_id, event_data, entry.user_data);
            }
        }
    }

    bool contains(uint8_t sensor_id) const
    {
        for (const auto &entry : entries) {
            if (entry.sensor_id == sensor_id) return true;
        }
        return false;
    }

    /**
     * @brief Returns the total number of registered callbacks (not distinct sensor IDs).
     */
    size_t size() const
    {
        return entries.size();
    }

    bool empty() const
    {
        return entries.empty();
    }

#else  // !BOSCH_USE_VECTOR_FALLBACK
    // -------------------------------------------------------------------------
    // Map-based implementation (unordered_map or map)
    // -------------------------------------------------------------------------
    /**
     * @brief Internal entry holding a callback and its associated user data.
     */
    struct CallbackEntry {
        CallbackType callback;   ///< Function pointer to the callback.
        void *user_data;         ///< User data to be passed to the callback.
    };

    BoschCallbackMap<uint8_t, std::vector<CallbackEntry>> callback_map; ///< Maps sensor ID to a list of callback entries.

public:
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
        callback_map[sensor_id].push_back({callback, user_data});
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
     * @brief Returns the number of distinct sensor IDs that have at least one callback registered.
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
#endif // BOSCH_USE_VECTOR_FALLBACK

    // -------------------------------------------------------------------------
    // Common constructors / destructor / copy / move control
    // -------------------------------------------------------------------------
    BoschSensorCallbackTemplate() = default;
    ~BoschSensorCallbackTemplate() = default;

    // Non-copyable
    BoschSensorCallbackTemplate(const BoschSensorCallbackTemplate &) = delete;
    BoschSensorCallbackTemplate& operator = (const BoschSensorCallbackTemplate &) = delete;

    // Movable (efficient)
    BoschSensorCallbackTemplate(BoschSensorCallbackTemplate &&) noexcept = default;
    BoschSensorCallbackTemplate& operator = (BoschSensorCallbackTemplate &&) noexcept = default;
};

// -----------------------------------------------------------------------------
// Concrete type aliases
// -----------------------------------------------------------------------------

/**
 * @brief Concrete manager for SensorDataParseCallback callbacks.
 */
using BoschParseCallbackManager = BoschSensorCallbackTemplate<SensorDataParseCallback>;

/**
 * @brief Concrete manager for SensorMetaEventCallback callbacks.
 */
using BoschMetaEventCallbackManager = BoschSensorCallbackTemplate<SensorMetaEventCallback>;
