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

#ifdef _V3
byte msgQueueStore[QUEUE_SIZE][BUFFER_LENGTH];
int msgIndex = 0;
byte *message;
#else
static unsigned char dataPoint[12];
static unsigned char message[BUFFER_LENGTH];
#endif

// STATISTICS

int nError              = 0;

// MAIN SETUP

void setup() {
    memset(&my, 0x00, sizeof(MyInfo));
    
    setupBoard();
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

static time_t msgBootEpoch;
static int msgMicros;

#ifdef _V3
void formatHeader(int port, int address, byte *message, int count, int timeOffset) {
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
}
#else    
void formatHeader(unsigned char *message) {
    getTimeSinceBoot(&msgBootEpoch, &msgMicros);
    message[0] = (msgBootEpoch >> 16) & 0xFF;
    message[1] = (msgBootEpoch >> 8) & 0xFF;
    message[2] = msgBootEpoch & 0xFF;
    
    int millis = msgMicros / 1000L;
    // 76543210
    // rrrfffmm
    message[3] = (unsigned char)(millis / 256) | ((unsigned char)my.frequencyCode << 2) | ((unsigned char)my.rssi << 5) ;
    message[4] = millis % 256;
}
#endif

#ifndef _V3
void formatData(unsigned char *message) {
    int offsetMillis = getRelMicroseconds(msgBootEpoch, msgMicros) / 1000;
    message[0] = offsetMillis / 256;
    message[1] = offsetMillis % 256;
    memcpy(message + 2, dataPoint, 6);
    memcpy(message + 8, dataPoint + 8, 4);
}
#endif

void loop() {
    unsigned long cycle_time;

    while (FATAL_ERROR);
    if (!isConnected()) RESTART(2);
    manageButton(0);
    
#ifdef _V3
    waitNextCycle();
    
    cycle_time = micros();
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensor[port][address].type) {
                message = msgQueueStore[msgIndex];
                int fifoCount = readFifo(port, address, message);
                if (fifoCount > 0) {
                    queueMessage(&msgIndex);
                    if (++msgIndex >= QUEUE_SIZE) msgIndex = 0;
                }
            }
        }
    }
#else                
    if (timeRemaining() == 0) {
        nMissed[Core1I2C]++;
        catchUpCycle();
    } else {
        waitNextCycle();
    }

    cycle_time = micros();
    formatHeader(message);
    int offset = HEADER_LENGTH;
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensor[port][address].type) {
                if (!readSensor(port, address, dataPoint)) nError ++;
                formatData(message + offset);
                offset += DATA_LENGTH;
            }
        }
    }
    queueMessage(message);
#endif                
    logCycleTime(Core1I2C, micros_diff(micros(), cycle_time));
}

// UTILITY FUNCTION

void RESTART(int seconds) {
    Serial.printf("RESTART in %d\n", seconds);
    blinkLed(COLOR_RED);
    delay(1000 * seconds);
    blinkLed(COLOR_BLACK);
    ESP.restart();
}

bool FATAL_ERROR = false;

void ERROR_FATAL(char *where) {
    while (FATAL_ERROR);
    FATAL_ERROR = true;
    Serial.printf("\nFATAL ERROR:%s\n", where);
#ifdef _V3
    byte *message = msgQueueStore[msgIndex];
    message[0] = 0xFF;
    message[3] = strlen(where);
    strcpy((char*)message + 8, where);
    queueMessage(&msgIndex);
    if (++msgIndex >= QUEUE_SIZE) msgIndex = 0;
#endif
    RESTART(5);
}
