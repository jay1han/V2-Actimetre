#include <Wire.h>
#include <HardwareSerial.h>
#include <esp_cpu.h>
#include <Esp.h>
#include "Actimetre.h"

typedef enum {
    _PIN_BUTTON = 0,
    _PIN_LED,

    _PIN_UART_GND,
    _PIN_UART_TX,
    _PIN_UART_RX,
 
    _PIN_I2C0_SDA,
    _PIN_I2C0_SCL,
    _PIN_I2C0_GND,
    _PIN_I2C0_VCC,

    _PIN_I2C1_SDA,
    _PIN_I2C1_SCL,
    _PIN_I2C1_GND,
    _PIN_I2C1_VCC,

    _PIN_I2C0_SDA_MUX,
    _PIN_I2C0_SCL_MUX,
    _PIN_I2C0_VCC_MUX,
    _PIN_I2C0_GND_MUX,

    PIN_MAX
} PinName;

// BOARD DEFINITIONS (See readme.txt)

#define POWERED_PIN  0x80
#define DISABLE_I2C  0x80
const uint8_t PINS[BOARD_TYPES][PIN_MAX] = {
    // Board Type 0 (pin 35 is GND)
    {0, 15,
     1, 2, 4,
     39, 37, 35, 33,
     8, 10, 13, 14,
     3, 5, 7, 9},
    // Board Type 1 (pin 35 is HIGH`)
    {0, 15,
     1, 2, 4,
     21, 34, 36, 38,
     8, 10, 13, 14,
     3, 5, 7, 9},
    // Board Type 2 (S2 without UART, 35 and 1 are HIGH)
    {0, 0xFF,
     0xFF, 0xFF, 0xFF,
     21, 17, 0xFF, 15,
     8, 10, 13, 14,
     3, 5, 7, 9},
    // Board Type 3 (S3 mini with I2C)
    {0, 0xFF,
     0xFF, 0xFF, 0xFF,   // UART is unused
     21, 17, 0xFF, 15,   // Less MUX
     7, 8, 9, 14,
     12, 13, 11, 10},
    // Board Type 4 (S3 mini with new box)
    {0, 0xFF,
     0xFF, 0xFF, 0xFF,   // UART is unused
     13, 11, POWERED_PIN | 10, 0xFF,   // I2C0 on left side
     0xFF, 0xFF, 0xFF, 0xFF,    // No I2C1
     0xFF, 0xFF, 0xFF, 0xFF},
};
#define PIN_DETECT_01 35 // HIGH for type 1
#define PIN_DETECT_12 1  // if also HIGH then type 2
#define PIN_DETECT_34 14 // if pulled HIGH then type 3 else type 4

// GLOBALS

uint8_t PIN_BUTTON, PIN_LED,
    PIN_UART_GND, PIN_UART_TX, PIN_UART_RX,
    PIN_I2C0_SDA, PIN_I2C0_SCL, PIN_I2C0_GND, PIN_I2C0_VCC,
    PIN_I2C1_SDA, PIN_I2C1_SCL, PIN_I2C1_GND, PIN_I2C1_VCC,
    PIN_I2C0_SDA_MUX, PIN_I2C0_SCL_MUX, PIN_I2C0_VCC_MUX, PIN_I2C0_GND_MUX;

int cycleFrequency;
unsigned long cycleMicroseconds;
#ifdef _V3
typedef enum {FREQ_BASE = 0, FREQ_FAST, FREQ_MAX, FREQ_OVERMAX, FREQ_COUNT} FreqCode;
#else
typedef enum {FREQ_BASE = 0, FREQ_SLOW, FREQ_DRIP, FREQ_TURBO, FREQ_COUNT} FreqCode;
#endif

FreqCode freqCode = FREQ_BASE;
static int Frequencies[BOARD_TYPES][FREQ_COUNT]   = {
    {50,  30, 10, 100},
    {50,  30, 10, 100},
    {50,  30, 10, 100},
    {100, 50, 10, 200},
#ifdef _V3
    {100, 500, 1000, 2000},
#else
    {100, 50, 10, 200},
#endif
};
static int FrequencyCode[BOARD_TYPES][FREQ_COUNT] = {
    {0, 4, 5, 1},
    {0, 4, 5, 1},
    {0, 4, 5, 1},
    {1, 0, 5, 3},
#ifdef _V3
    {0, 1, 2, 3},
#else
    {1, 0, 5, 3},
#endif
};
// 0=50, 1=100, 2=1, 3=200, 4=30, 5=10
// V3: 0=100, 1=500, 2=1000, 3=2000, 4=4000, 5=8000
static char BoardName[BOARD_TYPES][4] = {".S2", "S2x", "S2u", "S3i", "S3n"};

void setupBoard() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    if (chip_info.model == CHIP_ESP32S3) {
        my.dualCore = 1;
        pinMode(PIN_DETECT_34, INPUT_PULLDOWN);
        if (digitalRead(PIN_DETECT_34) == 1) my.boardType = BOARD_S3_I2C;
        else my.boardType = BOARD_S3_NEWBOX;
    } else {
        my.dualCore = 0;
        pinMode(PIN_DETECT_01, INPUT);
        if (digitalRead(PIN_DETECT_01) == 0) my.boardType = BOARD_S2;
        else if (digitalRead(PIN_DETECT_12) == 0) my.boardType = BOARD_S2_CONNECTORS;
        else my.boardType = BOARD_S2_NO_UART;
    }
    strcpy(my.boardName, BoardName[my.boardType]);

    pinMode(PIN_BUTTON    = PINS[my.boardType][_PIN_BUTTON],   INPUT_PULLUP);
    pinMode(PIN_LED       = PINS[my.boardType][_PIN_LED],      OUTPUT);
    if (my.boardType >= BOARD_S3_I2C)
        blinkLed(-2);
    else blinkLed(1);

#if ARDUINO_USB_CDC_ON_BOOT
#define HWSerial  Serial0
#define USBSerial Serial
    Serial.begin(2000000);
#else
#define HWSerial  Serial
    USBCDC USBSerial;
    pinMode(PIN_UART_GND  = PINS[my.boardType][_PIN_UART_GND], INPUT);
    PIN_UART_RX           = PINS[my.boardType][_PIN_UART_RX];
    PIN_UART_TX           = PINS[my.boardType][_PIN_UART_TX];
    Serial.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
#endif

    delay(1000);
    Serial.printf("\nHELLO! Board Type %d. Main thread on Core %d. Cycle frequency %dHz.\n",
                  my.boardType, xPortGetCoreID(), Frequencies[my.boardType][0]);

    if (PINS[my.boardType][_PIN_I2C0_SDA] & DISABLE_I2C) {
        my.hasI2C[0] = false;
    } else {
        my.hasI2C[0] = true;
        PIN_I2C0_SDA                = PINS[my.boardType][_PIN_I2C0_SDA];
        PIN_I2C0_SCL                = PINS[my.boardType][_PIN_I2C0_SCL];
        PIN_I2C0_GND                = PINS[my.boardType][_PIN_I2C0_GND];
        PIN_I2C0_VCC                = PINS[my.boardType][_PIN_I2C0_VCC];
        if (PIN_I2C0_GND & POWERED_PIN == 0) pinMode(PIN_I2C0_GND, INPUT);
        else { PIN_I2C0_GND &= 0x7F;  pinMode(PIN_I2C0_GND, OUTPUT); digitalWrite(PIN_I2C0_GND, 0); }
        if (PIN_I2C0_VCC & POWERED_PIN == 0) pinMode(PIN_I2C0_VCC, INPUT);
        else { PIN_I2C0_VCC &= 0x7F;  pinMode(PIN_I2C0_VCC, OUTPUT); digitalWrite(PIN_I2C0_VCC, 1); }
    }
    
    if (PINS[my.boardType][_PIN_I2C1_SDA] & DISABLE_I2C) {
        my.hasI2C[1] = false;
    } else {
        my.hasI2C[1] = true;
        PIN_I2C1_SDA                = PINS[my.boardType][_PIN_I2C1_SDA];
        PIN_I2C1_SCL                = PINS[my.boardType][_PIN_I2C1_SCL];
        PIN_I2C1_GND                = PINS[my.boardType][_PIN_I2C1_GND];
        PIN_I2C1_VCC                = PINS[my.boardType][_PIN_I2C1_VCC];
        if (PIN_I2C1_GND & POWERED_PIN == 0) pinMode(PIN_I2C1_GND, INPUT);
        else { PIN_I2C1_GND &= 0x7F;  pinMode(PIN_I2C1_GND, OUTPUT); digitalWrite(PIN_I2C1_GND, 0); }
        if (PIN_I2C1_VCC & POWERED_PIN == 0) pinMode(PIN_I2C1_VCC, INPUT);
        else { PIN_I2C1_VCC &= 0x7F;  pinMode(PIN_I2C1_VCC, OUTPUT); digitalWrite(PIN_I2C1_VCC, 1); }
    }
    
    pinMode(PIN_I2C0_GND_MUX = PINS[my.boardType][_PIN_I2C0_GND_MUX], INPUT);
    pinMode(PIN_I2C0_VCC_MUX = PINS[my.boardType][_PIN_I2C0_VCC_MUX], INPUT);
    pinMode(PIN_I2C0_SCL_MUX = PINS[my.boardType][_PIN_I2C0_SCL_MUX], INPUT);
    pinMode(PIN_I2C0_SDA_MUX = PINS[my.boardType][_PIN_I2C0_SDA_MUX], INPUT);

    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_BAUDRATE);
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C_BAUDRATE);

    cycleFrequency = Frequencies[my.boardType][FREQ_BASE];
    cycleMicroseconds = 1000000L / cycleFrequency;
    my.frequencyCode = FrequencyCode[my.boardType][FREQ_BASE];
}

TaskHandle_t core0Task;
void setupCore0(void (*core0Loop)(void*)) {
    if (xTaskCreatePinnedToCore(core0Loop, "Core0", 16384, NULL, 2, &core0Task, 0) != pdPASS) {
        Serial.println("Error starting Core 0");
        ESP.restart();
    }
}

static void switchFrequency() {
    freqCode = (FreqCode) (((int)freqCode + 1) % FREQ_COUNT);
    cycleFrequency = Frequencies[my.boardType][freqCode];
    cycleMicroseconds = 10000000L / cycleFrequency;
    my.frequencyCode = FrequencyCode[my.boardType][freqCode];
    Serial.printf("Sampling at %dHz = %dus per reading\n", cycleFrequency, cycleMicroseconds);
    setSensorsFrequency(cycleFrequency);
    displaySensors();
    clearCycleTime();
}

// LED AND BUTTON

void lightNeoPixel(byte r, byte g, byte b) {
}

void blinkLed(int color) {
    static int set = 0, save = 0;
    static int colored = 0;
    if (color == -2) {               // setting up
        colored = 1; 
        set = COLOR_WHITE;
    } else if (color == -1) {        // blink
        if (colored) {               // in color
            if (set == 0) set = save;
            else {                   // save the color for blinking
                save = color;
                set = 0;
            }
        }
        else set = 1 - set;          // B/W blinking
    } else {
        set = color;                 // straight set color
    }

    if (colored) {                   // output the color...
        
    } else
        digitalWrite(PIN_LED, set);
}

int buttonPressed() {
    return 1 - digitalRead(PIN_BUTTON);
}

void longPress() {
    Serial.println("Button long-press");
    // Do nothing
}

void shortPress() {
    Serial.println("Button press");
    switchFrequency();
}

void manageButton() {
    static unsigned long buttonDown;
    static int prevButton = 0;
    static int nowButton  = 0;
    static int actioned   = 0;

    nowButton = 1 - digitalRead(PIN_BUTTON);
    if (nowButton) { // pressed
        if (prevButton) { // still pressed
            if (millis_diff_10(millis(), buttonDown) > LONGPRESS_MILLIS) { // it's a long press
                if (!actioned){  // only action once
                    longPress();
                    actioned = 1;
                }
            } else { // wait until timeout or release
            }
        } else { // was not pressed, start counter
            buttonDown = millis();
            actioned = 0;
        }
    } else { // now released
        if (prevButton) { // was pressed
            if (!actioned) { // short press
                shortPress();
            } else { // already actioned, so do nothing
                actioned = 0;
            }
        } else {
            // nothing...
        }
    }

    prevButton = nowButton;
}
