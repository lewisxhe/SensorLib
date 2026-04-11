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
 * @file      BQ25896_Charger_WebMonitor.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-10
 *
 * @brief BQ25896 PMIC Web Monitor - BMU (Battery Management Unit) Web Dashboard
 *
 * This example demonstrates how to monitor and control the BQ25896 charger IC
 * via a web browser. The ESP32 creates a web server that displays:
 * - Real-time charging status (voltage, current, temperature)
 * - Charger state (charging, done, fault)
 * - Boost mode control (OTG power output)
 * - Ship mode control (battery disconnect for storage)
 *
 * Access the web interface by connecting to the ESP32's WiFi IP address.
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#include <WebServer.h>
#include <PmicDrv.hpp>

#ifndef WIFI_SSID
#define WIFI_SSID "YourSSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YourPassword"
#endif

#ifndef PMIC_SDA
#define PMIC_SDA  2
#endif

#ifndef PMIC_SCL
#define PMIC_SCL  3
#endif

#ifndef PMIC_IRQ
#define PMIC_IRQ  -1
#endif

volatile bool isFaultTrigger = false;

WebServer server(80);

PmicBQ25896 bmu;

#define CHECK_FOR_ERRORS(condition) if (condition) {log_e("BQ25896 error"); while(1)delay(1000);}

void printBanner()
{
    Serial.println("");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     BQ25896 Charger Web Monitor Test     ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println("");
}

void setup()
{
    bool rlst;

    Serial.begin(115200);

    printBanner();

    rlst = bmu.begin(Wire, BQ25896_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL);
    if (!rlst) {
        Serial.println("BQ25896 begin() failed. Check wiring.");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("BQ25896 initialized successfully");

    // bmu.led().setMode(PmicLedBase::Mode::AUTO); // STAT pin indicates charging status
    bmu.led().setMode(PmicLedBase::Mode::DISABLE); // DISABLE LED to save power (if STAT pin is not used for indication)

    // Pre-charge current: 64-1024mA, 64mA steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setPreChargeCurrent(256);
    CHECK_FOR_ERRORS(!rlst);
    // Fast charge current: 0-3008mA, 64mA steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setFastChargeCurrent(1024);
    CHECK_FOR_ERRORS(!rlst);
    // Termination current: 64-1024mA, 64mA steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setTerminationCurrent(64);
    CHECK_FOR_ERRORS(!rlst);
    // Charge voltage: 3840-4608mV, 16mV steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setChargeVoltage(4288);
    CHECK_FOR_ERRORS(!rlst);

    Serial.print("Pre Charge Current: ");
    Serial.println(bmu.charger().getPreChargeCurrent());
    Serial.print("Fast Charge Current: ");
    Serial.println(bmu.charger().getFastChargeCurrent());
    Serial.print("Termination Current: ");
    Serial.println(bmu.charger().getTerminationCurrent());
    Serial.print("Charge Voltage: ");
    Serial.println(bmu.charger().getChargeVoltage());

    // Input current limit: 100-3008mA, 100mA steps (fuzzy: auto-adjusts to nearest valid value)
    bmu.power().setInputCurrentLimit(1000);
    // Input voltage limit: 3900-14000mV, 100mV steps (fuzzy: auto-adjusts to nearest valid value)
    bmu.power().setInputVoltageLimit(5000);
    // System minimum voltage: 3000-3700mV, 100mV steps (fuzzy: auto-adjusts to nearest valid value)
    bmu.power().setMinimumSystemVoltage(3300);

    Serial.print("Input Current Limit: ");
    Serial.println(bmu.power().getInputCurrentLimit());
    Serial.print("Input Voltage Limit: ");
    Serial.println(bmu.power().getInputVoltageLimit());
    Serial.print("Minimum System Voltage: ");
    Serial.println(bmu.power().getMinimumSystemVoltage());

    // Enable ADC channels (VBUS voltage, Battery voltage, VSYS voltage, Charging current, NTC temperature)
    bmu.adc().enableChannels(BQ25896Adc::ADC_CONV_START);

    // Disabling ADC can reduce quiescent current.
    // If call `read` directly after disabling `adc`, it will trigger a single read operation.
    // bmu.adc().disableChannels(0); // No specific channels to disable, but call for compatibility

    const char *ssid = WIFI_SSID;
    const char *password = WIFI_PASSWORD;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());

    server.on("/boost/on", []() {
        if (bmu.core().isVbusPresent()) {
            server.send(400, "text/plain", "ERROR: Remove USB first before enabling boost");
            return;
        }
        bool ret = bmu.power().enableBoost(true);
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/boost/off", []() {
        bool ret = bmu.power().enableBoost(false);
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/ship/on", []() {
        if (bmu.core().isVbusPresent()) {
            server.send(400, "text/plain", "ERROR: Remove USB power first to enable ship mode");
            return;
        }
        bool ret = bmu.power().enableShipMode(true);
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/", []() {
        char timeStr[32];
        time_t now = millis() / 1000;
        int hrs = (now / 3600) % 24;
        int mins = (now / 60) % 60;
        int secs = now % 60;
        sprintf(timeStr, "%02d:%02d:%02d", hrs, mins, secs);

        bool boostEnabled = bmu.power().isBoostEnabled();
        bool vbusPresent = bmu.core().isVbusPresent();

        String html = "<html><head><meta charset='utf-8'><title>BQ25896 Charger Monitor</title>";
        html += "<script>";
        html += "function toggleBoost(enable) {";
        html += "  var xhr = new XMLHttpRequest();";
        html += "  xhr.open('GET', enable ? '/boost/on' : '/boost/off', true);";
        html += "  xhr.onload = function() { if(xhr.status==200) location.reload(); else alert(xhr.responseText); };";
        html += "  xhr.send();";
        html += "}";
        html += "function enableShipMode() {";
        html += "  if(!confirm('WARNING: Ship mode will disconnect the battery and power off the device!\\n\\nYou must connect USB power  or Press PWR button to keep the device running after enabling ship mode.\\n\\nAre you sure you want to continue?')) return;";
        html += "  var xhr = new XMLHttpRequest();";
        html += "  xhr.open('GET', '/ship/on', true);";
        html += "  xhr.onload = function() { if(xhr.status==200) { alert('Ship mode enabled! Device will power off.'); location.reload(); } else { alert(xhr.responseText); } };";
        html += "  xhr.send();";
        html += "}";
        html += "</script>";
        html += "</head><body>";
        html += "<h1>BQ25896 Charger Status</h1>";
        html += "<p><strong>Last Update: " + String(timeStr) + "</strong></p>";

        html += "<h2>Boost Control</h2>";
        html += "<p>USB Status: <strong>" + String(vbusPresent ? "Connected" : "Not Connected") + "</strong></p>";
        html += "<p>Boost Status: <strong>" + String(boostEnabled ? "ON" : "OFF") + "</strong></p>";
        if (boostEnabled) {
            html += "<button onclick='toggleBoost(false)' style='background:#f44336;color:white;padding:10px 20px;border:none;cursor:pointer;'>Turn OFF Boost</button>";
        } else if (vbusPresent) {
            html += "<button disabled style='background:#ccc;color:#666;padding:10px 20px;border:none;cursor:not-allowed;'>Turn ON Boost (USB Connected)</button>";
        } else {
            html += "<button onclick='toggleBoost(true)' style='background:#4CAF50;color:white;padding:10px 20px;border:none;cursor:pointer;'>Turn ON Boost</button>";
        }

        html += "<h2>Ship Mode (Battery Off)</h2>";
        html += "<p>Ship mode disconnects the battery and powers off the device.</p>";
        html += "<p>To exit ship mode, connect USB power.</p>";
        html += "<button onclick='enableShipMode()' style='background:#FF9800;color:white;padding:10px 20px;border:none;cursor:pointer;'>Enable Ship Mode</button>";

        html += "<h2>Charger Status</h2>";
        html += "<table border='1' cellpadding='10'>";

        float val;
        bool ret;

        ret = bmu.adc().read(PmicAdcBase::Channel::VBUS_VOLTAGE, val);
        html += "<tr><td>VBUS Voltage</td><td>" + String(ret ? val : -1) + " mV</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_VOLTAGE, val);
        html += "<tr><td>Battery Voltage</td><td>" + String(ret ? val : -1) + " mV</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::VSYS_VOLTAGE, val);
        html += "<tr><td>VSYS Voltage</td><td>" + String(ret ? val : -1) + " mV</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_CURRENT, val);
        html += "<tr><td>Battery Current</td><td>" + String(ret ? val : -1) + " mA</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_TEMPERATURE, val);
        html += "<tr><td>NTC Temperature</td><td>" + String(ret ? val : -1) + " %</td></tr>";

        html += "<tr><td>Charging</td><td>" + String(bmu.charger().isCharging() ? "Yes" : "No") + "</td></tr>";

        PmicChargerBase::Status status = bmu.charger().getStatus();
        html += "<tr><td>Online</td><td>" + String(status.online ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>VBUS Present</td><td>" + String(status.vbusPresent ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>Charge Done</td><td>" + String(status.chargeDone ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>Fault</td><td>" + String(status.fault ? "Yes" : "No") + "</td></tr>";
        String chargingStatusStr;
        switch (status.chargingStatus) {
        case PmicChargerBase::ChargingStatus::NO_CHARGING: chargingStatusStr = "No charging";
            break;
        case PmicChargerBase::ChargingStatus::PRE_CHARGE: chargingStatusStr = "Pre-charge";
            break;
        case PmicChargerBase::ChargingStatus::FAST_CHARGE: chargingStatusStr = "Fast Charging";
            break;
        case PmicChargerBase::ChargingStatus::TERMINATION: chargingStatusStr = "Charge Termination Done";
            break;
        default:
            break;
        }
        html += "<tr><td>Charging Status</td><td>" + chargingStatusStr + "</td></tr>";

        html += "<tr><td>Boost Enabled</td><td>" + String(boostEnabled ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>Boost Voltage</td><td>" + String(bmu.power().getBoostVoltage()) + " mV</td></tr>";

        html += "<tr><td>ICO Optimized</td><td>" + String(bmu.core().isInputCurrentOptimizerOptimized() ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>VINDPM Active</td><td>" + String(bmu.core().isVindpmActive() ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>IINDPM Active</td><td>" + String(bmu.core().isIindpmActive() ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>VSYS in Regulation</td><td>" + String(bmu.core().isVsysInRegulation() ? "Yes" : "No") + "</td></tr>";

        html += "</table>";
        html += "<meta http-equiv='refresh' content='1'>";
        html += "<p>Auto-refresh every 1 second</p>";
        html += "</body></html>";

        server.send(200, "text/html", html);
    });

    server.begin();
    Serial.println("Web server started");

    Serial.println("Setup complete!");

#if PMIC_IRQ != -1
    pinMode(PMIC_IRQ, INPUT_PULLUP);
    attachInterrupt(PMIC_IRQ, []() {
        isFaultTrigger = true;
    }, FALLING);
#endif

}

void loop()
{

    server.handleClient();
    delay(1);

#if PMIC_IRQ != -1
    if (isFaultTrigger) {
        isFaultTrigger = false;
        PmicChargerBase::Status status = bmu.charger().getStatus();
        if (status.fault) {
            Serial.print("Fault Code: ");
            Serial.println(status.faultCode);
            using namespace BQ25896Faults;
            if (isWatchdogTimeout(status.faultCode)) {
                Serial.println("\t Watchdog Fault");
            }
            if (isBoostFault(status.faultCode)) {
                Serial.println("\t Boost Mode Fault");
            }
            if (isChargeFault(status.faultCode)) {
                Serial.println("\t Charge Mode Fault");
                uint8_t type = getChargeFaultType(status.faultCode);
                if (isChargeInputFault(status.faultCode)) {
                    Serial.println("\t   - Input Fault (BUS OVP or VBAT<BUS<3.8V)");
                }
                if (isChargeThermalFault(status.faultCode)) {
                    Serial.println("\t   - Thermal Shutdown");
                }
                if (isChargeTimerFault(status.faultCode)) {
                    Serial.println("\t   - Safety Timer Expiration");
                }
            }
            if (isBatteryFault(status.faultCode)) {
                Serial.println("\t Battery Fault (BATOVP)");
            }
            uint8_t ntcFault = getNtcFault(status.faultCode);
            if (ntcFault) {
                Serial.println("\t NTC Fault");
            }
        }
    }
#endif
}
#else
void setup()
{
    Serial.begin(115200);
    Serial.println("This example is only compatible with ESP32. Please run on ESP32 platform.");
}
void loop()
{
    delay(1000);
}
#endif
