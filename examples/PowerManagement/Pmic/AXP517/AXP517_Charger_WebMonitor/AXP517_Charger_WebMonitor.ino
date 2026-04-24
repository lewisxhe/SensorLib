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
 * IMPLIFF, BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      AXP517_Charger_WebMonitor.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-23
 *
 * @brief AXP517 PMIC Web Monitor - Battery Management Web Dashboard
 *
 * This example demonstrates how to monitor and control the AXP517 PMIC
 * via a web browser. The ESP32 creates a web server that displays:
 * - Real-time charging status (voltage, current, die temperature)
 * - Charger state (charging, done, battery present)
 * - Boost mode control (OTG power output)
 * - Ship mode control (battery disconnect for storage)
 * - BC1.2 charger type detection
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
#define PMIC_SDA  3
#endif

#ifndef PMIC_SCL
#define PMIC_SCL  2
#endif

#ifndef PMIC_IRQ
#define PMIC_IRQ  -1
#endif

volatile bool irqTriggered = false;

WebServer server(80);

PmicAXP517 bmu;

String ledModeStr(PmicLedBase::Mode mode)
{
    switch (mode) {
    case PmicLedBase::Mode::AUTO:    return "Auto";
    case PmicLedBase::Mode::MANUAL:  return "Manual";
    case PmicLedBase::Mode::BREATH:  return "Breath";
    case PmicLedBase::Mode::DISABLE: return "Disabled";
    default:                         return "Unknown";
    }
}

String ledStateStr(PmicLedBase::ManualState state)
{
    switch (state) {
    case PmicLedBase::ManualState::HiZ:        return "Off (Hi-Z)";
    case PmicLedBase::ManualState::LEVEL_LOW:  return "Low";
    case PmicLedBase::ManualState::LEVEL_HIGH: return "High";
    case PmicLedBase::ManualState::BLINK_1HZ:  return "Blink 1Hz";
    case PmicLedBase::ManualState::BLINK_4HZ:  return "Blink 4Hz";
    default:                                    return "Unknown";
    }
}

void printBanner()
{
    Serial.println("");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║      AXP517 Charger Web Monitor Test     ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println("");
}

void setup()
{
    bool rlst;

    Serial.begin(115200);

    printBanner();

    rlst = bmu.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL);
    if (!rlst) {
        Serial.println("AXP517 begin() failed. Check wiring.");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("AXP517 initialized successfully");

    // Enable charger module
    bmu.enableModule(PmicAXP517::Module::CHARGE, true);

    // Configure LED mode
    bmu.led().setMode(PmicLedBase::Mode::DISABLE);

    // Pre-charge current: 0-960mA, 64mA steps
    rlst = bmu.charger().setPreChargeCurrent(256);
    if (!rlst) Serial.println("setPreChargeCurrent failed");
    // Fast charge current: 0-5120mA, 64mA steps
    rlst = bmu.charger().setFastChargeCurrent(1024);
    if (!rlst) Serial.println("setFastChargeCurrent failed");
    // Termination current: 0-960mA, 64mA steps
    rlst = bmu.charger().setTerminationCurrent(64);
    if (!rlst) Serial.println("setTerminationCurrent failed");
    // Charge voltage: 3600/3800/4000/4100/4200/4350/4400/5000 mV
    rlst = bmu.charger().setChargeVoltage(4200);
    if (!rlst) Serial.println("setChargeVoltage failed");

    Serial.print("Pre Charge Current: ");
    Serial.println(bmu.charger().getPreChargeCurrent());
    Serial.print("Fast Charge Current: ");
    Serial.println(bmu.charger().getFastChargeCurrent());
    Serial.print("Termination Current: ");
    Serial.println(bmu.charger().getTerminationCurrent());
    Serial.print("Charge Voltage: ");
    Serial.println(bmu.charger().getChargeVoltage());

    // Power path configuration
    // Input current limit: 100-3250mA, 50mA steps
    bmu.power().setInputCurrentLimit(1000);
    // Input voltage limit: 3600-16200mV, 100mV steps
    bmu.power().setInputVoltageLimit(4700);
    // Minimum system voltage: 1000-3800mV, 100mV steps
    bmu.power().setMinimumSystemVoltage(3300);

    Serial.print("Input Current Limit: ");
    Serial.println(bmu.power().getInputCurrentLimit());
    Serial.print("Input Voltage Limit: ");
    Serial.println(bmu.power().getInputVoltageLimit());
    Serial.print("Minimum System Voltage: ");
    Serial.println(bmu.power().getMinimumSystemVoltage());

    // Enable ADC channels for monitoring
    bmu.adc().enableChannels(
           AXP517Adc::ADC_VBUS_VOLTAGE |
           AXP517Adc::ADC_VBUS_CURRENT |
           AXP517Adc::ADC_BAT_VOLTAGE |
           AXP517Adc::ADC_BAT_CHARGE |
           AXP517Adc::ADC_BAT_DISCHARGE |
           AXP517Adc::ADC_SYSTEM_VOLTAGE |
           AXP517Adc::ADC_DIE_TEMP |
           AXP517Adc::ADC_TS_PIN
       );

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
        bool ret = bmu.power().enableBoost(true);
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/boost/off", []() {
        bool ret = bmu.power().enableBoost(false);
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/ship/on", []() {
        PmicChargerBase::Status status = bmu.charger().getStatus();
        if (status.vbusPresent) {
            server.send(400, "text/plain", "ERROR: Remove USB power first to enable ship mode");
            return;
        }
        bool ret = bmu.power().enableShipMode(true);
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/led/mode", []() {
        if (!server.hasArg("mode")) {
            server.send(400, "text/plain", "Missing mode argument");
            return;
        }
        String mode = server.arg("mode");
        bool ret = false;
        if (mode == "auto") {
            ret = bmu.led().setMode(PmicLedBase::Mode::AUTO);
        } else if (mode == "manual") {
            ret = bmu.led().setMode(PmicLedBase::Mode::MANUAL);
        } else if (mode == "breath") {
            ret = bmu.led().setMode(PmicLedBase::Mode::BREATH);
        } else if (mode == "disable") {
            ret = bmu.led().setMode(PmicLedBase::Mode::DISABLE);
        } else {
            server.send(400, "text/plain", "Invalid mode");
            return;
        }
        server.send(200, "text/plain", ret ? "OK" : "FAILED");
    });

    server.on("/led/state", []() {
        if (!server.hasArg("state")) {
            server.send(400, "text/plain", "Missing state argument");
            return;
        }
        String state = server.arg("state");
        bool ret = false;
        if (state == "off") {
            ret = bmu.led().setManualState(PmicLedBase::ManualState::HiZ);
        } else if (state == "low") {
            ret = bmu.led().setManualState(PmicLedBase::ManualState::LEVEL_LOW);
        } else if (state == "high") {
            ret = bmu.led().setManualState(PmicLedBase::ManualState::LEVEL_HIGH);
        } else if (state == "blink1") {
            ret = bmu.led().setManualState(PmicLedBase::ManualState::BLINK_1HZ);
        } else if (state == "blink4") {
            ret = bmu.led().setManualState(PmicLedBase::ManualState::BLINK_4HZ);
        } else {
            server.send(400, "text/plain", "Invalid state");
            return;
        }
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
        bool shipEnabled = bmu.power().isShipModeEnabled();
        PmicChargerBase::Status status = bmu.charger().getStatus();

        String html = "<html><head><meta charset='utf-8'><title>AXP517 Charger Monitor</title>";
        html += "<script>";
        html += "function toggleBoost(enable) {";
        html += "  var xhr = new XMLHttpRequest();";
        html += "  xhr.open('GET', enable ? '/boost/on' : '/boost/off', true);";
        html += "  xhr.onload = function() { if(xhr.status==200) location.reload(); else alert(xhr.responseText); };";
        html += "  xhr.send();";
        html += "}";
        html += "function enableShipMode() {";
        html += "  if(!confirm('WARNING: Ship mode will disconnect the battery and power off the device!\\n\\nYou must connect USB power or press PWR button to keep the device running after enabling ship mode.\\n\\nAre you sure you want to continue?')) return;";
        html += "  var xhr = new XMLHttpRequest();";
        html += "  xhr.open('GET', '/ship/on', true);";
        html += "  xhr.onload = function() { if(xhr.status==200) { alert('Ship mode enabled! Device will power off.'); location.reload(); } else { alert(xhr.responseText); } };";
        html += "  xhr.send();";
        html += "}";
        html += "function setLedMode(mode) {";
        html += "  var xhr = new XMLHttpRequest();";
        html += "  xhr.open('GET', '/led/mode?mode=' + mode, true);";
        html += "  xhr.onload = function() { if(xhr.status==200) location.reload(); else alert(xhr.responseText); };";
        html += "  xhr.send();";
        html += "}";
        html += "function setLedState(state) {";
        html += "  var xhr = new XMLHttpRequest();";
        html += "  xhr.open('GET', '/led/state?state=' + state, true);";
        html += "  xhr.onload = function() { if(xhr.status==200) location.reload(); else alert(xhr.responseText); };";
        html += "  xhr.send();";
        html += "}";
        html += "</script>";
        html += "</head><body>";
        html += "<h1>AXP517 Charger Status</h1>";
        html += "<p><strong>Last Update: " + String(timeStr) + "</strong></p>";

        html += "<h2>Boost Control</h2>";
        html += "<p>Boost Status: <strong>" + String(boostEnabled ? "ON" : "OFF") + "</strong></p>";
        if (boostEnabled) {
            html += "<button onclick='toggleBoost(false)' style='background:#f44336;color:white;padding:10px 20px;border:none;cursor:pointer;'>Turn OFF Boost</button>";
        } else {
            html += "<button onclick='toggleBoost(true)' style='background:#4CAF50;color:white;padding:10px 20px;border:none;cursor:pointer;'>Turn ON Boost</button>";
        }

        html += "<h2>Ship Mode (Battery Off)</h2>";
        html += "<p>Ship mode is <strong>" + String(shipEnabled ? "Enabled" : "Disabled") + "</strong></p>";
        html += "<p>Ship mode disconnects the battery and powers off the device.</p>";
        html += "<p>To exit ship mode, connect USB power.</p>";
        html += "<button onclick='enableShipMode()' style='background:#FF9800;color:white;padding:10px 20px;border:none;cursor:pointer;'>Enable Ship Mode</button>";

        html += "<h2>LED Control</h2>";
        html += "<p>LED Mode: <strong>" + String(ledModeStr(bmu.led().getMode())) + "</strong></p>";
        html += "<div>";
        html += "<button onclick='setLedMode(\"auto\")' style='background:#2196F3;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Auto</button>";
        html += "<button onclick='setLedMode(\"manual\")' style='background:#2196F3;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Manual</button>";
        html += "<button onclick='setLedMode(\"breath\")' style='background:#2196F3;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Breath</button>";
        html += "<button onclick='setLedMode(\"disable\")' style='background:#f44336;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Disable</button>";
        html += "</div>";
        html += "<p>Manual State: <strong>" + String(ledStateStr(bmu.led().getManualState())) + "</strong></p>";
        html += "<div>";
        html += "<button onclick='setLedState(\"off\")' style='background:#607D8B;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Off (Hi-Z)</button>";
        html += "<button onclick='setLedState(\"low\")' style='background:#607D8B;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Low</button>";
        html += "<button onclick='setLedState(\"high\")' style='background:#4CAF50;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>High</button>";
        html += "<button onclick='setLedState(\"blink1\")' style='background:#FF9800;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Blink 1Hz</button>";
        html += "<button onclick='setLedState(\"blink4\")' style='background:#FF9800;color:white;padding:8px 16px;border:none;cursor:pointer;margin:2px;'>Blink 4Hz</button>";
        html += "</div>";

        html += "<h2>Charger Status</h2>";
        html += "<table border='1' cellpadding='10'>";

        float val;
        bool ret;

        html += "<tr><td>Chip</td><td>AXP517</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::VBUS_VOLTAGE, val);
        html += "<tr><td>VBUS Voltage</td><td>" + String(ret ? val : -1) + " mV</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::VBUS_CURRENT, val);
        html += "<tr><td>VBUS Current</td><td>" + String(ret ? val : -1) + " mA</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_VOLTAGE, val);
        html += "<tr><td>Battery Voltage</td><td>" + String(ret ? val : -1) + " mV</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_CURRENT, val);
        html += "<tr><td>Battery Current</td><td>" + String(ret ? val : -1) + " mA</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::VSYS_VOLTAGE, val);
        html += "<tr><td>VSYS Voltage</td><td>" + String(ret ? val : -1) + " mV</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::DIE_TEMPERATURE, val);
        html += "<tr><td>Die Temperature</td><td>" + String(ret ? val : -1) + " C</td></tr>";

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_TEMPERATURE, val);
        html += "<tr><td>NTC Temperature</td><td>" + String(ret ? val : -1) + " %</td></tr>";

        html += "<tr><td>Charging</td><td>" + String(status.charging ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>Online</td><td>" + String(status.online ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>VBUS Present</td><td>" + String(status.vbusPresent ? "Yes" : "No") + "</td></tr>";
        html += "<tr><td>Battery Present</td><td>" + String(status.batteryPresent ? "Yes" : "No") + "</td></tr>";
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

        html += "<tr><td>Pre-Charge Current</td><td>" + String(bmu.charger().getPreChargeCurrent()) + " mA</td></tr>";
        html += "<tr><td>Fast Charge Current</td><td>" + String(bmu.charger().getFastChargeCurrent()) + " mA</td></tr>";
        html += "<tr><td>Termination Current</td><td>" + String(bmu.charger().getTerminationCurrent()) + " mA</td></tr>";
        html += "<tr><td>Charge Voltage</td><td>" + String(bmu.charger().getChargeVoltage()) + " mV</td></tr>";

        html += "<tr><td>Input Current Limit</td><td>" + String(bmu.power().getInputCurrentLimit()) + " mA</td></tr>";
        html += "<tr><td>Input Voltage Limit</td><td>" + String(bmu.power().getInputVoltageLimit()) + " mV</td></tr>";
        html += "<tr><td>Min System Voltage</td><td>" + String(bmu.power().getMinimumSystemVoltage()) + " mV</td></tr>";

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
        irqTriggered = true;
    }, FALLING);
#endif

}

void loop()
{

    server.handleClient();
    delay(1);

#if PMIC_IRQ != -1
    if (irqTriggered) {
        irqTriggered = false;
        uint32_t irqStatus = bmu.irq().readStatus(true);
        if (irqStatus) {
            Serial.print("IRQ Status: 0x");
            Serial.println(irqStatus, HEX);
            if (bmu.irq().isVbusInsert(irqStatus)) {
                Serial.println("VBUS Inserted");
            }
            if (bmu.irq().isVbusRemove(irqStatus)) {
                Serial.println("VBUS Removed");
            }
            if (bmu.irq().isBatteryInsert(irqStatus)) {
                Serial.println("Battery Inserted");
            }
            if (bmu.irq().isBatteryRemove(irqStatus)) {
                Serial.println("Battery Removed");
            }
            if (bmu.irq().isChargeDone(irqStatus)) {
                Serial.println("Charge Done");
            }
            if (bmu.irq().isChargeStart(irqStatus)) {
                Serial.println("Charge Started");
            }
            if (bmu.irq().isVbusOverVoltage(irqStatus)) {
                Serial.println("VBUS Over Voltage");
            }
            if (bmu.irq().isBoostOverVoltage(irqStatus)) {
                Serial.println("Boost Over Voltage");
            }
            if (bmu.irq().isDieOverTempLevel1(irqStatus)) {
                Serial.println("Die Over Temperature");
            }
            if (bmu.irq().isChgSafetyTimer(irqStatus)) {
                Serial.println("Safety Timer Expired");
            }
            if (bmu.irq().isBatOverVoltage(irqStatus)) {
                Serial.println("Battery Over Voltage");
            }
            if (bmu.irq().isBatFetOcp(irqStatus)) {
                Serial.println("Battery FET Over Current");
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
