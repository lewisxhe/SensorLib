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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      BMA_Universal_Compatibility.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-25
 *
 * @brief Universal BMA4XX Compatibility Example
 *
 * This example demonstrates how to write a single firmware that works with
 * BMA422, BMA423, or BMA456H sensors without knowing which chip is on the board.
 *
 * Design pattern:
 *  1. Auto-detect the sensor by trying each driver
 *  2. Operate through the SensorBMA4XX base class pointer for common APIs
 *  3. Query capabilities via getCapabilities() to enable features polymorphically
 *  4. For typed operations (callbacks), dispatch via DetectedModel enum
 */
#include <AccelerometerDrv.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  21
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  22
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  39
#endif

// Board compatibility layer: instantiate all possible sensors.
// Only the detected one will be initialized; the others remain idle.
SensorBMA422 bma422;
SensorBMA423 bma423;
SensorBMA456H bma456h;

// Base class pointer for polymorphic operation (features, config, poll)
SensorBMA4XX *sensor = nullptr;

enum class DetectedModel {
    NONE,
    BMA422,
    BMA423,
    BMA456H
};

DetectedModel detectedModel = DetectedModel::NONE;

volatile bool isInterruptTriggered = false;


// ============================================================
// Probe & init: try each sensor in order, keep the first hit
// ============================================================
bool initSensor()
{
    if (bma422.begin(Wire, BMA4XX_I2C_ADDR_SDO_HIGH, SENSOR_SDA, SENSOR_SCL)) {
        sensor = &bma422;
        detectedModel = DetectedModel::BMA422;
        Serial.println("[probe] Detected BMA422");
        return true;
    }

    if (bma423.begin(Wire, BMA4XX_I2C_ADDR_SDO_LOW, SENSOR_SDA, SENSOR_SCL)) {
        sensor = &bma423;
        detectedModel = DetectedModel::BMA423;
        Serial.println("[probe] Detected BMA423");
        return true;
    }

    if (bma456h.begin(Wire, BMA4XX_I2C_ADDR_SDO_HIGH, SENSOR_SDA, SENSOR_SCL)) {
        sensor = &bma456h;
        detectedModel = DetectedModel::BMA456H;
        Serial.println("[probe] Detected BMA456H");
        return true;
    }

    return false;
}


// ============================================================
// Callback dispatcher
// uses the already-known DetectedModel enum.
// ============================================================
void setupTapCallback()
{
    switch (detectedModel) {
    case DetectedModel::BMA423:
        bma423.setOnTapCallback([](TapType t) {
            switch (t) {
            case TapType::SINGLE_TAP:
                Serial.println("  [callback] SINGLE TAP"); break;
            case TapType::DOUBLE_TAP:
                Serial.println("  [callback] DOUBLE TAP"); break;
            default: break;
            }
        });
        break;
    case DetectedModel::BMA456H:
        bma456h.setOnTapCallback([](TapType t) {
            switch (t) {
            case TapType::SINGLE_TAP:
                Serial.println("  [callback] SINGLE TAP"); break;
            case TapType::DOUBLE_TAP:
                Serial.println("  [callback] DOUBLE TAP"); break;
            case TapType::TRIPLE_TAP:
                Serial.println("  [callback] TRIPLE TAP"); break;
            default: break;
            }
        });
        break;
    default:
        break;
    }
}

void setupStepCallbacks()
{
    switch (detectedModel) {
    case DetectedModel::BMA423:
        bma423.setOnStepDetectedCallback([]() {
            Serial.println("  [callback] STEP DETECTED");
        });
        bma423.setOnStepCountCallback([](uint32_t count) {
            Serial.print("  [callback] STEP COUNT: ");
            Serial.println(count);
        });
        break;
    case DetectedModel::BMA456H:
        bma456h.setOnStepDetectedCallback([]() {
            Serial.println("  [callback] STEP DETECTED");
        });
        bma456h.setOnStepCountCallback([](uint32_t count) {
            Serial.print("  [callback] STEP COUNT: ");
            Serial.println(count);
        });
        break;
    default:
        break;
    }
}

void setupActivityCallback()
{
    auto fn = [](ActivityType a) {
        Serial.print("  [callback] ACTIVITY: ");
        switch (a) {
        case ActivityType::STATIONARY: Serial.println("Stationary"); break;
        case ActivityType::WALKING:    Serial.println("Walking");    break;
        case ActivityType::RUNNING:    Serial.println("Running");    break;
        default:                       Serial.println("Unknown");   break;
        }
    };

    switch (detectedModel) {
    case DetectedModel::BMA423:
        bma423.setOnActivityCallback(fn);
        break;
    case DetectedModel::BMA456H:
        bma456h.setOnActivityCallback(fn);
        break;
    default:
        break;
    }
}

void setupTiltCallback()
{
    if (detectedModel == DetectedModel::BMA423) {
        bma423.setOnTiltDetectedCallback([]() {
            Serial.println("  [callback] TILT DETECTED");
        });
    }
}

void setupDataReadyCallback()
{
    auto fn = [](AccelerometerData data) {
        Serial.print("  [callback] DataReady: ");
        Serial.print(data.mps2.x, 2); Serial.print(", ");
        Serial.print(data.mps2.y, 2); Serial.print(", ");
        Serial.println(data.mps2.z, 2);
    };

    switch (detectedModel) {
    case DetectedModel::BMA422:
        bma422.setOnDataReadyCallback(fn);
        break;
    case DetectedModel::BMA423:
        bma423.setOnDataReadyCallback(fn);
        break;
    case DetectedModel::BMA456H:
        bma456h.setOnDataReadyCallback(fn);
        break;
    default:
        break;
    }
}


// ============================================================
// Capability-based feature setup
// ============================================================
void setupFeatures()
{
    auto caps = sensor->getCapabilities();

    Serial.print("Chip: ");
    Serial.print(sensor->getModelName());
    Serial.print(", Capabilities: 0x");
    Serial.println(static_cast<uint32_t>(caps), HEX);

    if (caps & BMA4XXCapability::Capability::SupportDataReady) {
        Serial.println("  + enableDataReady");
        sensor->enableDataReady(true);
    }

    if (caps & BMA4XXCapability::Capability::SupportAnyMotion) {
        Serial.println("  + enableAnyMotion");
        sensor->enableAnyMotionDetection(
            SensorBMA4XX::MotionAxesConfig(1, 1, 1), true
        );
    }

    if (caps & BMA4XXCapability::Capability::SupportAxisRemap) {
        Serial.println("  + setRemapAxes");
        sensor->setRemapAxes(SensorRemap::TOP_LAYER_RIGHT_CORNER);
    }

    if (caps & BMA4XXCapability::Capability::SupportTap) {
        Serial.println("  + enableTapDetector");
        sensor->enableTapDetector(true);
        setupTapCallback();
    }

    if (caps & BMA4XXCapability::Capability::SupportStepDetector) {
        Serial.println("  + enableStepDetector");
        sensor->enableStepCounter(true, 1, true);
        sensor->enableStepDetector(true, true);
        setupStepCallbacks();
    }

    if (caps & BMA4XXCapability::Capability::SupportActivity) {
        Serial.println("  + enableActivityRecognition");
        sensor->enableActivityRecognition(true);
        setupActivityCallback();
    }

    if (caps & BMA4XXCapability::Capability::SupportTilt) {
        Serial.println("  + enableTiltDetector");
        sensor->enableTiltDetector(true);
        setupTiltCallback();
    }
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(SENSOR_IRQ, INPUT);
    attachInterrupt(SENSOR_IRQ, []() {
        isInterruptTriggered = true;
    }, RISING);

    // ---- Step 1: Auto-detect sensor ----
    if (!initSensor()) {
        while (1) {
            Serial.println("No BMA4XX sensor detected!");
            delay(1000);
        }
    }

    Serial.print("Using: ");
    Serial.println(sensor->getModelName());

    // ---- Step 2: Configure accelerometer (base class API) ----
    if (!sensor->configAccelerometer(
                OperationMode::NORMAL,
                AccelFullScaleRange::FS_2G,
                100.0f,
                AccelBandwidth::OSR2_AVG2,
                AccelPerfMode::CIC_AVG_MODE)) {
        Serial.println("Failed to configure accelerometer");
        while (1);
    }

    // ---- Step 3: Configure interrupt pin (base class API) ----
    sensor->setInterruptPinConfig(
        InterruptPinMap::PIN1,
        false,   // level trigger
        false,   // active high
        true,    // output enable
        false);  // input disable

    // ---- Step 4: Enable features based on capabilities ----
    setupFeatures();

    // ---- Step 5: Set data ready callback ----
    setupDataReadyCallback();

    delay(1000);
    Serial.println();
    Serial.println("=== System ready. Waiting for events... ===");
    Serial.println();
}

void loop()
{
    if (isInterruptTriggered) {
        isInterruptTriggered = false;
        sensor->update();
    }

    static unsigned long lastPoll = 0;
    if (millis() - lastPoll > 5000) {
        lastPoll = millis();
        Serial.print("[poll] Temperature: ");
        Serial.print(sensor->getTemperature());
        Serial.println(" C");
    }
    delay(10);
}
