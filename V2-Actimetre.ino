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

static long msgEpoch;
static long msgMicros;

void formatHeader(unsigned char *message) {
  getTime(&msgEpoch, &msgMicros);
  msgEpoch -= my.bootTime;
  unsigned long millis = msgMicros / 1000L;
  message[0] = (msgEpoch << 16L) % 256L;
  message[1] = (msgEpoch << 8L) % 256L;
  message[2] = msgEpoch % 256L;
  message[3] = millis / 256L;
  message[4] = millis % 256L;
}

void formatData(unsigned char *message) {
    long millis, micros;
    getTime(NULL, &micros);
    if (micros >= msgMicros) millis = (micros - msgMicros) / 1000L;
    else millis = (micros + 1000000L - msgMicros) / 1000L;
    message[0] = millis / 256;
    message[1] = millis % 256;
    memcpy(message + 2, data.readBuffer, 6);
    memcpy(message + 8, data.readBuffer + 8, 4);
}

void loop() {
    static int firstloop = 1;
    static unsigned long cycle_time = 0L;
    
    unsigned long time_spent;
    esp_task_wdt_reset();

    displayLoop(0);
    manageButton();

    if (firstloop == 1) { // ignore first loop()
        firstloop = 0;
    }
    else {
        time_spent = micros_diff(micros(), cycle_time);
        logCycleTime(Core1I2C, time_spent);
        if (time_spent > cycleMicroseconds) {
            Serial.println("Missed Cycle");
            nMissed[Core1I2C]++;
        } else {
            waitNextCycle(cycle_time);
        }
    }
    
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

    sendMessageProcess(message);
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
