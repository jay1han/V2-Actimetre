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
    // Board Type 0 (pin 35 is GND)
    {0, 15,
     1, 2, 4,
     39, 37, 35, 33,
     8, 10, 13, 14,
     3, 5, 7, 9},
    // Board Type 1 (pin 35 is HIGH)
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
#ifdef _V3    
    // Board Type 4 (S3 mini with new box)
    {0, 47,
     0xFF, 0xFF, 0xFF,   // UART is unused
     13, 11, POWERED_PIN | 10, 0xFF,   // I2C0 on left side
     0xFF, 0xFF, 0xFF, 0xFF,    // No I2C1
     0xFF, 0xFF, 0xFF, 0xFF},
    // Board Type 5 (S3 mini with MPU-6500)
    {0, 47,
     0xFF, 0xFF, 0xFF,   // UART is unused
     13, 11, POWERED_PIN | 10, 0xFF,   // I2C0 on left side
     0xFF, 0xFF, 0xFF, 0xFF,    // No I2C1
     0xFF, 0xFF, 0xFF, 0xFF},
#endif    
};
#define PIN_DETECT_01 35 // HIGH for type 1
#define PIN_DETECT_12 1  // if also HIGH then type 2
#define PIN_DETECT_34 14 // if pulled HIGH then type 3 else type 4
#define PIN_DETECT_45 16 // if pulled LOW then type 5 else type 4

// GLOBALS

static char BoardName[BOARD_TYPES][4] = {".S2", "S2x", "S2u", "S3i"
#ifdef _V3    
    , "S3n", "S3+"
#endif    
};

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
    {100, 1000, 2000, 4000},
    {100, 1000, 4000, 8000},
#endif
};
static int FrequencyCode[BOARD_TYPES][FREQ_COUNT] = {
    {0, 4, 5, 1},
    {0, 4, 5, 1},
    {0, 4, 5, 1},
    {1, 0, 5, 3},
#ifdef _V3
    {0, 2, 3, 4},
    {0, 2, 4 | (SAMPLE_ACCEL << 3), 5 | (SAMPLE_GYRO << 3)},
#endif
};
// 0=50, 1=100, 2=1, 3=200, 4=30, 5=10
// V3: 0=100, 1=500, 2=1000, 3=2000, 4=4000, 5=8000

static void switchFrequency() {
    freqCode = (FreqCode) (((int)freqCode + 1) % FREQ_COUNT);
    my.frequencyCode = FrequencyCode[my.boardType][freqCode];
    cycleFrequency = Frequencies[my.boardType][freqCode];
#ifdef _V3    
    my.samplingMode = my.frequencyCode >> 3;
    if (my.sensorType == WAI_6500) {
        switch (my.frequencyCode >> 3) {
        case SAMPLE_ACCEL:
            my.maxMeasures = 40;
            my.perCycle    = 30;
            my.dataLength  = 6;
            break;

        case SAMPLE_GYRO:
            my.maxMeasures = 60;
            my.perCycle    = 40;
            my.dataLength  = 4;
            break;

        default:
            my.maxMeasures = 25;
            my.perCycle    = 20;
            my.dataLength  = 10;
            break;
        }
    }
#endif        
    cycleMicroseconds = READING_BASE / cycleFrequency;
    
    Serial.printf("Sampling at %dHz (code %X) = %dus per reading\n",
                  cycleFrequency, my.frequencyCode, cycleMicroseconds);
    setSensorsFrequency(cycleFrequency);
    displaySensors();
    clearCycleTime();
    int kHz = cycleFrequency / 1000;
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

static int COLORS[] = {0x070007, 0x00070F, 0x000F00, 0x070700, 0x0F0000};

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
        pinMode(PIN_DETECT_45, INPUT_PULLUP);
#ifdef _V3        
        if (digitalRead(PIN_DETECT_34) == 1) my.boardType = BOARD_S3_I2C;
        else if (digitalRead(PIN_DETECT_45) == 0) my.boardType = BOARD_S3_6500;
        else my.boardType = BOARD_S3_NEWBOX;
#else
        my.boardType = BOARD_S3_I2C;
#endif        
    } else {
        my.dualCore = 0;
        my.ledRGB = false;
        pinMode(PIN_DETECT_01, INPUT);
        if (digitalRead(PIN_DETECT_01) == 0) my.boardType = BOARD_S2;
        else if (digitalRead(PIN_DETECT_12) == 0) my.boardType = BOARD_S2_CONNECTORS;
        else my.boardType = BOARD_S2_NO_UART;
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
    Serial.printf("\nSoftware v%s! Board Type %s(%d). Main thread on Core %d. Cycle frequency %dHz.\n",
                  VERSION_STR, my.boardName, my.boardType, xPortGetCoreID(), Frequencies[my.boardType][0]);
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

#ifdef _V3    
    my.samplingMode = SAMPLE_ALL;
    my.dataLength  = 10;
    my.maxMeasures = 25;
    my.perCycle    = 20;
#endif    
    cycleFrequency = Frequencies[my.boardType][FREQ_BASE];
    cycleMicroseconds = READING_BASE / cycleFrequency;
    my.frequencyCode = FrequencyCode[my.boardType][FREQ_BASE];
    Serial.printf("Sampling at %dHz = %dus per reading\n", cycleFrequency, cycleMicroseconds);
}

TaskHandle_t core0Task;
void setupCore0(void (*core0Loop)(void*)) {
    if (xTaskCreatePinnedToCore(core0Loop, "Core0", 16384, NULL, 2, &core0Task, 0) != pdPASS) {
        Serial.println("Error starting Core 0");
        ESP.restart();
    }
}
