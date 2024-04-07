#include <Wire.h>
#include <HardwareSerial.h>
#include <esp_cpu.h>
#include <Esp.h>
#include <esp32-hal.h>
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
    // Board Type 0 (S3 mini with I2C)
    {0, 47,
     0xFF, 0xFF, 0xFF,   // UART is unused
     21, 17, 0xFF, 15 | POWERED_PIN,   // Less MUX
     7, 8, 9 | POWERED_PIN, 14,   // PIN 14 is connected to 3V
     12, 13, 11, 10},
    // Board Type 1 (S3 mini with new box)
    {0, 47,
     0xFF, 0xFF, 0xFF,   // UART is unused
     13, 11, POWERED_PIN | 10, 0xFF,   // I2C0 on left side
     44, 36, 35 | POWERED_PIN, 18 | POWERED_PIN, 
     0xFF, 0xFF, 0xFF, 0xFF},
    // Board Type 2 (S3 super mini)
    {0, 21,
     0xFF, 0xFF, 0xFF,   // UART is unused
     3, 4, POWERED_PIN | 5, POWERED_PIN | 6,   // I2C0
     10, 9, 8 | POWERED_PIN, 7 | POWERED_PIN,
     0xFF, 0xFF, 0xFF, 0xFF},
    // Board Type 3 (S3 super mini single with display)
    {0, 21,
     0xFF, 0xFF, 0xFF,   // UART is unused
     3, 4, POWERED_PIN | 5, POWERED_PIN | 6,   // I2C0
     10, 9, 8 | POWERED_PIN, 7 | POWERED_PIN,
     0xFF, 0xFF, 0xFF, 0xFF},
    // Unsupported
    {0xFF, 0xFF,
     0xFF, 0xFF, 0xFF,   // UART is unused
     0xFF, 0xFF, 0xFF, 0xFF,
     0xFF, 0xFF, 0xFF, 0xFF,
     0xFF, 0xFF, 0xFF, 0xFF},
};
#define PIN_DETECT_34 14 // if pulled HIGH then type 0 else type 1
#define PIN_DETECT_45 1  // if pulled HIGH then type 2 else type 1

// GLOBALS

static char BoardName[BOARD_TYPES][4] = {"S3i", "S3n", "S3s", "S3d", "BAD"};

uint8_t PIN_BUTTON, PIN_LED,
    PIN_UART_GND, PIN_UART_TX, PIN_UART_RX,
    PIN_I2C0_SDA, PIN_I2C0_SCL, PIN_I2C0_GND, PIN_I2C0_VCC,
    PIN_I2C1_SDA, PIN_I2C1_SCL, PIN_I2C1_GND, PIN_I2C1_VCC,
    PIN_I2C0_SDA_MUX, PIN_I2C0_SCL_MUX, PIN_I2C0_VCC_MUX, PIN_I2C0_GND_MUX;

#define FREQ_COUNT   4
int freqCode =  0;
static int Frequencies[8] = {100, 500, 1000, 2000, 4000, 8000};

static int FrequencyCode[BOARD_TYPES][FREQ_COUNT] = {
    {0, 2, 4, 5},
    {0, 2, 4, 5},
    {0, 2, 4, 5},
    {0, 2, 4, 5},
    {0, 0, 0, 0}
};

static void switchFrequency() {
    freqCode = (freqCode + 1) % FREQ_COUNT;
    my.frequencyCode = FrequencyCode[my.boardType][freqCode];
    my.cycleFrequency = Frequencies[my.frequencyCode];
    setSamplingMode();
    
    setSensorsFrequency();
    displaySensors();
    clearCycleTime();
    clearNextCycle();
    int kHz = my.cycleFrequency / 1000;
    int rank = 0;
    while (kHz > 0) {
        rank ++;
        kHz >>= 1;
    }
    blinkLed(COLOR_FREQ | rank);
}

// LED AND BUTTON

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

void manageButton(int set) {
    static unsigned long buttonDown;
    static int prevButton = 0;
    int nowButton  = 0;
    static int actioned   = 0;

    if (set == 1) { // Force a short press
        prevButton = 1;
        buttonDown = millis() - LONGPRESS_MILLIS;
        return;
    }
    
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

#define RMT_SIZE (8 * 3)
static rmt_obj_t *RmtObject;
static rmt_data_t RmtBuffer[RMT_SIZE];
static void setupRGB() {
    Serial.printf("RGB pin %d init ", PIN_LED);
    RmtObject = rmtInit(PIN_LED, RMT_TX_MODE, RMT_MEM_64);
    if (RmtObject == NULL) {
        Serial.println("FAILED");
        return;
    } else {
        float tick = rmtSetTick(RmtObject, 50.0);
        if (abs(tick - 50.0) > 1.0) {
            Serial.printf("tick set %.0fns!\n", tick);
            return;
        }
    }
    Serial.println("OK");
}
    
static rmt_data_t *stuffBits(rmt_data_t *data, int level) {
    for (int bit = 7; bit >= 0; bit--) {
        if (level & (1 << bit)) {
            data->level0    = 1;
            data->duration0 = 12;
            data->level1    = 0;
            data->duration1 = 5;
        } else {
            data->level0    = 1;
            data->duration0 = 5;
            data->level1    = 0;
            data->duration1 = 12;
        }
        data++;
    }
    return data;
}

// magenta, yellow, cyan, green, blue
static int COLORS[] = {0x0F0007, 0x00070F, 0x0F0700, 0x000F00, 0x170000};

void blinkLed(int command) {
    static int saved = COLOR_WHITE;
    static bool state = false;
    int color;
    
    if (command == COLOR_SWAP) {
        if (state) color = COLOR_BLACK;
        else color = saved;
        state = !state;
    } else if (command & COLOR_FREQ) {
        color = saved = COLORS[command & 0x07];
        state = true;
    } else {
        color = saved = command;
        state = true;
    }
    
    if (my.ledRGB) {
        rmt_data_t *data = RmtBuffer;
        data = stuffBits(data, color & 0xFF);
        data = stuffBits(data, (color >> 8) & 0xFF);
        data = stuffBits(data, color >> 16);
        rmtWrite(RmtObject, RmtBuffer, RMT_SIZE);
    } else {
        if (color == 0) digitalWrite(PIN_LED, 0);
        else digitalWrite(PIN_LED, 1);
    }
}

void setupBoard() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    if (chip_info.model == CHIP_ESP32S3) {
        my.dualCore = 1;
        my.ledRGB = true;
        pinMode(PIN_DETECT_34, INPUT_PULLDOWN);
        pinMode(PIN_DETECT_45, INPUT_PULLDOWN);
        if (digitalRead(PIN_DETECT_34) == 1) my.boardType = BOARD_S3_I2C;
        else if (digitalRead(PIN_DETECT_45) == 1) my.boardType = BOARD_S3_SUPER;
        else my.boardType = BOARD_S3_NEWBOX;
    } else {
        my.boardType = BOARD_BAD;
    }
    strcpy(my.boardName, BoardName[my.boardType]);

    pinMode(PIN_BUTTON    = PINS[my.boardType][_PIN_BUTTON],   INPUT_PULLUP);
    pinMode(PIN_LED       = PINS[my.boardType][_PIN_LED],      OUTPUT);

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
    Serial.printf("\nSoftware v%s Board Type %s(%d)\n",
                  VERSION_STR, my.boardName, my.boardType);
    if (my.ledRGB) setupRGB();

    if (PINS[my.boardType][_PIN_I2C0_SDA] & DISABLE_I2C) {
        my.hasI2C[0] = false;
    } else {
        my.hasI2C[0] = true;
        PIN_I2C0_SDA                = PINS[my.boardType][_PIN_I2C0_SDA];
        PIN_I2C0_SCL                = PINS[my.boardType][_PIN_I2C0_SCL];
        PIN_I2C0_GND                = PINS[my.boardType][_PIN_I2C0_GND];
        PIN_I2C0_VCC                = PINS[my.boardType][_PIN_I2C0_VCC];
        if (PIN_I2C0_GND == 0xFF) {}
        else if ((PIN_I2C0_GND & POWERED_PIN) == 0) pinMode(PIN_I2C0_GND, INPUT);
        else { PIN_I2C0_GND &= 0x7F;  pinMode(PIN_I2C0_GND, OUTPUT); digitalWrite(PIN_I2C0_GND, 0); }
        if (PIN_I2C0_VCC == 0xFF) {}
        else if ((PIN_I2C0_VCC & POWERED_PIN) == 0) pinMode(PIN_I2C0_VCC, INPUT);
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
        if ((PIN_I2C1_GND & POWERED_PIN) == 0) pinMode(PIN_I2C1_GND, INPUT);
        else { PIN_I2C1_GND &= 0x7F;  pinMode(PIN_I2C1_GND, OUTPUT); digitalWrite(PIN_I2C1_GND, 0); }
        if ((PIN_I2C1_VCC & POWERED_PIN) == 0) pinMode(PIN_I2C1_VCC, INPUT);
        else { PIN_I2C1_VCC &= 0x7F;  pinMode(PIN_I2C1_VCC, OUTPUT); digitalWrite(PIN_I2C1_VCC, 1); }
    }
    
    if ((PINS[my.boardType][_PIN_I2C0_GND_MUX] & 0x80) == 0)
        pinMode(PIN_I2C0_GND_MUX = PINS[my.boardType][_PIN_I2C0_GND_MUX], INPUT);
    if ((PINS[my.boardType][_PIN_I2C0_VCC_MUX] & 0x80) == 0)
        pinMode(PIN_I2C0_VCC_MUX = PINS[my.boardType][_PIN_I2C0_VCC_MUX], INPUT);
    if ((PINS[my.boardType][_PIN_I2C0_SCL_MUX] & 0x80) == 0)
        pinMode(PIN_I2C0_SCL_MUX = PINS[my.boardType][_PIN_I2C0_SCL_MUX], INPUT);
    if ((PINS[my.boardType][_PIN_I2C0_SDA_MUX] & 0x80) == 0)
        pinMode(PIN_I2C0_SDA_MUX = PINS[my.boardType][_PIN_I2C0_SDA_MUX], INPUT);

    if (my.hasI2C[0])
        Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_BAUDRATE);
    if (my.hasI2C[1])
        Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C_BAUDRATE);

    blinkLed(COLOR_WHITE);

    my.frequencyCode = FrequencyCode[my.boardType][0];
    my.cycleFrequency = Frequencies[my.frequencyCode];
    if (my.boardType == BOARD_BAD) {
        my.cycleMicroseconds = 100000;
    } else {
        setSamplingMode();
    }
}

TaskHandle_t core0Task;
void setupCore0(void (*core0Loop)(void*)) {
    if (xTaskCreatePinnedToCore(core0Loop, "Core0", 16384, NULL, 2, &core0Task, 0) != pdPASS) {
        Serial.println("Error starting Core 0");
        ESP.restart();
    }
}
