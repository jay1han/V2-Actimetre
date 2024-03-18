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

static unsigned char dataPoint[12];
static unsigned char message[BUFFER_LENGTH];

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
    Serial.println(title);

    netInit();
}

// MAIN LOOP

static time_t msgBootEpoch;
static int msgMicros;

void formatHeader(unsigned char *message) {
    getTimeSinceBoot(&msgBootEpoch, &msgMicros);
    int millis = msgMicros / 1000L;
    message[0] = (msgBootEpoch >> 16) % 256;
    message[1] = (msgBootEpoch >> 8) % 256;
    message[2] = msgBootEpoch % 256;
    // 76543210
    // rrrfffmm
    message[3] = (unsigned char)(millis / 256) | ((unsigned char)my.frequencyCode << 2) | ((unsigned char)my.rssi << 5) ;
    message[4] = millis % 256;
}

void formatData(unsigned char *message) {
#ifndef _OVERCLOCK
    int offsetMillis = getRelMicroseconds(msgBootEpoch, msgMicros) / 1000;
    message[0] = offsetMillis / 256;
    message[1] = offsetMillis % 256;
    memcpy(message + 2, dataPoint, 6);
    memcpy(message + 8, dataPoint + 8, 4);
#else
    memcpy(message, dataPoint, 6);
    memcpy(message + 6, dataPoint + 8, 4);
#endif
}

void loop() {
    static int firstLoop = 1;
    static unsigned long cycle_time;
    
    esp_task_wdt_reset();
    manageButton();

    if (!isConnected()) ESP.restart();

    if (timeRemaining() == 0) {
        nMissed[Core1I2C]++;
        catchUpCycle();
    } else {
        waitNextCycle();
    }
    cycle_time = micros();

    int port, address = 0;
    formatHeader(message);
    int offset = HEADER_LENGTH;

    for (port = 0; port <= 1; port++) {
        for (address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
                if (!readSensor(port, address, dataPoint)) nError ++;
                formatData(message + offset);
                offset += DATA_LENGTH;
            }
        }

        logCycleTime(Core1I2C, micros_diff(micros(), cycle_time));
    }
    
    queueMessage(message);
}

// UTILITY FUNCTION

void ERROR_FATAL(char *where) {
    Serial.printf("\nFATAL ERROR %s\n", where);
    ESP.restart();
}

