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

static DataPoint data;
static unsigned char message[BUFFER_LENGTH];

// STATISTICS

int nError              = 0;

// MAIN SETUP

void setup() {
    memset(&my, 0x00, sizeof(MyInfo));
    
    setupBoard();

    deviceScanInit();

    displayTitle(VERSION_STR);
    displaySensors();

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
    message[3] = millis / 256;
    message[4] = millis % 256;
}

void formatData(unsigned char *message) {
    int offsetMillis = getRelMicroseconds(msgBootEpoch, msgMicros) / 1000;
    message[0] = offsetMillis / 256;
    message[1] = offsetMillis % 256;
    memcpy(message + 2, data.readBuffer, 6);
    memcpy(message + 8, data.readBuffer + 8, 4);
}

void loop() {
    static int firstLoop = 1;
    static unsigned long cycle_time;
    
    esp_task_wdt_reset();
    manageButton();

    if (!isConnected()) ESP.restart();

    if (firstLoop == 1) firstLoop = 0;    // ignore first loop()
    else logCycleTime(Core1I2C, micros_diff(micros(), cycle_time));
    
    waitNextCycle();
    cycle_time = micros();

    int port, address = 0;
    formatHeader(message);
    int offset = HEADER_LENGTH;

    for (port = 0; port <= 1; port++) {
        for (address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
                if (readSensor(port, address, &data)) {
                    formatData(message + offset);
                    offset += DATA_LENGTH;
                } else {
                    nError ++;
                }
            }
        }
    }
    
    queueMessage(message);
}

// UTILITY FUNCTION

void ERROR_FATAL(char *where) {
    Serial.printf("\nFATAL ERROR %s\n", where);
    ESP.restart();
}

// SHORT PRESS TO TURN ON SCREEN

void shortPress() {
    Serial.println("\nScreen on");
    displayLoop(2);
}
