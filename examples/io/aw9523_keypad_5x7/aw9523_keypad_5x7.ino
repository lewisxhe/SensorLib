/**
 * @file      aw9523_keypad_5x7.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @date      2026-09-23
 *
 * AW9523 5x7 interrupt-driven keyboard matrix example.
 *
 * Rows:    P0_0-P0_4 (open-drain outputs)
 * Columns: P0_5-P0_7 and P1_0-P1_3 (inputs with external pull-ups)
 * INTN:    Active-low open-drain output; connect it to SENSOR_IRQ.
 */
#include <IoExpanderDrv.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  10
#endif

static constexpr uint8_t ROW_COUNT = 5;
static constexpr uint8_t COLUMN_COUNT = 7;
static constexpr uint8_t ROW_PINS[ROW_COUNT] = {0, 1, 2, 3, 4};
static constexpr uint8_t COLUMN_PINS[COLUMN_COUNT] = {5, 6, 7, 8, 9, 10, 11};
static constexpr uint16_t ROW_MASK = 0x001F;
static constexpr uint16_t COLUMN_MASK = 0x0FE0;

// T-Deck V2 5x7 maps. The first index is the column and the second is the row.
static constexpr char KEY_MAP[COLUMN_COUNT][ROW_COUNT] = {
    {'q', 'e', 'r', 'u', 'o'},
    {'w', 's', 'g', 'h', 'l'},
    {'\0', 'd', 't', 'y', 'i'},
    {'a', 'p', '\0', '\n', '\0'},
    {'\0', 'x', 'v', 'b', '$'},
    {' ', 'z', 'c', 'n', 'm'},
    {'\0', '\0', 'f', 'j', 'k'},
};

static constexpr char SYMBOL_MAP[COLUMN_COUNT][ROW_COUNT] = {
    {'#', '2', '3', '_', '+'},
    {'1', '4', '/', ':', '"'},
    {'\0', '5', '(', ')', '-'},
    {'*', '@', '\0', '\0', '\0'},
    {'\0', '8', '?', '!', '\0'},
    {'\0', '7', '9', ',', '.'},
    {'0', '\0', '6', ';', '\''},
};

static constexpr uint8_t SYMBOL_ROW = 0;
static constexpr uint8_t SYMBOL_COLUMN = 2;

IoExpanderAW9523 expander;
volatile bool keyboardInterrupt = false;
bool keyState[ROW_COUNT][COLUMN_COUNT] = {};

void onKeyboardInterrupt()
{
    keyboardInterrupt = true;
}

void scanKeyboard(bool state[ROW_COUNT][COLUMN_COUNT])
{
    // In open-drain mode HIGH releases a row and LOW selects it.
    expander.digitalWritePort(ROW_MASK, ROW_MASK);

    for (uint8_t row = 0; row < ROW_COUNT; ++row) {
        expander.digitalWrite(ROW_PINS[row], LOW);
        delayMicroseconds(50);

        const uint16_t levels = expander.digitalReadPort();
        for (uint8_t column = 0; column < COLUMN_COUNT; ++column) {
            state[row][column] = (levels & (1UL << COLUMN_PINS[column])) == 0;
        }

        expander.digitalWrite(ROW_PINS[row], HIGH);
    }

    // All rows low allows any key press or release to change a column level.
    expander.digitalWritePort(ROW_MASK, 0);
    delayMicroseconds(20); // Longer than the AW9523 input debounce time.
    expander.digitalReadPort(); // Clear interrupts generated while scanning.
}

const char *specialKeyName(uint8_t row, uint8_t column)
{
    if (row == 0 && column == 2) return "Symbol";
    if (row == 2 && column == 3) return "Right Shift";
    if (row == 4 && column == 3) return "Backspace";
    if (row == 0 && column == 4) return "Alt";
    if (row == 0 && column == 6) return "Mic";
    if (row == 1 && column == 6) return "Left Shift";
    return nullptr;
}

void printKeyName(uint8_t row, uint8_t column, bool symbolLayer)
{
    const char *name = specialKeyName(row, column);
    if (name) {
        Serial.print(name);
        return;
    }

    const char key = symbolLayer ? SYMBOL_MAP[column][row] : KEY_MAP[column][row];
    if (key == '\n') {
        Serial.print("Enter");
    } else if (key == ' ') {
        Serial.print("Space");
    } else if (key == '\0') {
        Serial.print("Unmapped");
    } else {
        Serial.print(key);
    }
}

void reportChanges(const bool current[ROW_COUNT][COLUMN_COUNT])
{
    const bool symbolLayer = current[SYMBOL_ROW][SYMBOL_COLUMN];
    for (uint8_t row = 0; row < ROW_COUNT; ++row) {
        for (uint8_t column = 0; column < COLUMN_COUNT; ++column) {
            if (current[row][column] == keyState[row][column]) {
                continue;
            }

            keyState[row][column] = current[row][column];
            printKeyName(row, column, symbolLayer);
            Serial.println(current[row][column] ? " pressed" : " released");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(SENSOR_IRQ, INPUT_PULLUP);
    if (!expander.begin(Wire, AW9523_DEFAULT_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find AW9523 - check your wiring and address pins!");
            delay(1000);
        }
    }

    // Open-drain rows avoid driving one row high while another row is low.
    expander.setPort0PushPull(false);
    expander.configPins(ROW_MASK, OUTPUT);
    expander.digitalWritePort(ROW_MASK, 0);
    expander.configPins(COLUMN_MASK, INPUT);

    // The chip enables every GPIO interrupt after reset. Only watch columns.
    expander.enableInterrupts(IoExpanderAW9523::PORT_ALL, false);
    expander.enableInterrupts(COLUMN_MASK, true);

    scanKeyboard(keyState);
    keyboardInterrupt = false;
    attachInterrupt(digitalPinToInterrupt(SENSOR_IRQ), onKeyboardInterrupt, FALLING);
}

void loop()
{
    if (!keyboardInterrupt) {
        return;
    }

    keyboardInterrupt = false;
    delay(10); // Keyboard switch debounce.

    // Matrix scanning changes column levels, so temporarily ignore MCU IRQ edges.
    detachInterrupt(digitalPinToInterrupt(SENSOR_IRQ));
    bool current[ROW_COUNT][COLUMN_COUNT] = {};
    scanKeyboard(current);
    reportChanges(current);
    attachInterrupt(digitalPinToInterrupt(SENSOR_IRQ), onKeyboardInterrupt, FALLING);

    // Catch a state change that arrived between the final clear and re-attach.
    if (digitalRead(SENSOR_IRQ) == LOW) {
        keyboardInterrupt = true;
    }
}
