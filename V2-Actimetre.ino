#include <Wire.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "Actimetre.h"

// GLOBALS

MyInfo my;

// FILE-WIDE GLOBAL

byte msgQueueStore[QUEUE_SIZE][BUFFER_LENGTH];
static int nextIndex() {
    static int msgIndex = 1;
    int index = msgIndex++;
    if (msgIndex >= QUEUE_SIZE) msgIndex = 1;
    return index;
}

// MAIN SETUP

void setup() {
    memset(&my, 0x00, sizeof(MyInfo));
    
    setupBoard();

    if (my.boardType == BOARD_BAD) {
        Serial.println("Unsupported board type");
        writeLine("Unsupported");
        RESTART(30);
    }
    delay(100);
    deviceScanInit();

    char title[16];
    sprintf(title, "v%s", VERSION_STR);
    displayTitle(title);
    displaySensors();
    
    netInit();
    clearSensors();
    blinkLed(COLOR_FREQ | 0);
}

// MAIN LOOP

int64_t formatHeader(int port, int address, byte *message, int count, int timeOffset) {
    time_t msgBootEpoch;
    int msgMicros;
    getTimeSinceBoot(&msgBootEpoch, &msgMicros);
    msgMicros -= timeOffset;
    if (msgMicros < 0) {
        msgMicros += 1000000;
        msgBootEpoch -= 1;
    }
    message[0] = (msgBootEpoch >> 16) & 0xFF;
    message[1] = (msgBootEpoch >> 8) & 0xFF;
    message[2] = msgBootEpoch & 0xFF;

    if (count > 63) {
        ERROR_FATAL("Fifo count > 63");
    }
    message[3] = count | (port << 7) | (address << 6);
    message[4] = ((byte)my.rssi << 5) | ((byte)my.sensor[port][address].samplingMode << 3) | (byte)my.frequencyCode;
    message[5] = (msgMicros >> 16) & 0xFF;
    message[6] = (msgMicros >> 8) & 0xFF;
    message[7] = msgMicros & 0xFF;
    return (int64_t)msgBootEpoch * 1000000 + msgMicros;
}

void loop() {
    unsigned long cycle_time;

    while (FATAL_ERROR);
    if (!isConnected()) RESTART(2);
    manageButton(0);
    
    waitNextCycle();
    
    cycle_time = micros();
    int fifoState;
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensor[port][address].type) {
                do {
                    int index = nextIndex();
                    fifoState = readFifo(port, address, msgQueueStore[index]);
                    if (fifoState > 0) {
                        queueMessage(&index);
                    }
                } while (fifoState > 1);
            }
        }
    }
    
    logCycleTime(Core1I2C, micros_diff(micros(), cycle_time));
}

// UTILITY FUNCTION

void RESTART(int seconds) {
    FATAL_ERROR = true;
    Serial.printf("RESTART in %d\n", seconds);
    blinkLed(COLOR_RED);
    delay(1000 * seconds);
    blinkLed(COLOR_BLACK);
    ESP.restart();
}

bool FATAL_ERROR = false;

void ERROR_REPORT(char *what) {
    Serial.printf("\nREPORT:%s\n", what);

    int index = nextIndex();
    byte *message = msgQueueStore[index];
    message[0] = 0xFF;
    message[3] = strlen(what);
    strcpy((char*)message + HEADER_LENGTH, what);
    queueMessage(&index);
}

void ERROR_FATAL(char *where) {
    while (FATAL_ERROR);
    FATAL_ERROR = true;
    Serial.printf("\nFATAL:%s\n", where);
    ERROR_REPORT(where);
    RESTART(5);
}

void dump(void *pointer, int size) {
    byte *address = (byte*)pointer;
    for(int line = 0; line < size; line += 16) {
        for (int i = 0; (i < 16) && (line + i < size); i++) {
            Serial.printf("%02X ", address[line + i]);
        }
        Serial.print("    ");
        for (int i = 0; (i < 16) && (line + i < size); i++) {
            byte c = address[line + i];
            if (c < 0x20) c = '.';
            Serial.printf("%c", c);
        }
        Serial.println();
    }
}
