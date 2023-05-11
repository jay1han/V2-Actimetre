#include <Wire.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "Actimetre.h"

// GLOBALS

MyInfo my;
unsigned long nSamples = 0;

// FILE-WIDE GLOBAL

static DataPoint data;
static unsigned char message[PAYLOAD_PER_SENSOR * 4 + PAYLOAD_NUM_SENSORS];
// 75 bytes, also defined as MQTT_MSG_LENGTH

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

void formatData(int port, int address, unsigned char *message) {
    message[0] = port << 4 | address;
    message[1] = (data.time / (1L << 24)) % 256; // we have MSB first
    message[2] = (data.time / (1L << 16)) % 256;
    message[3] = (data.time / (1L << 8)) % 256;
    message[4] = data.time % 256;
    message[5] = (data.micros / (1L << 16)) % 256;
    message[6] = (data.micros / (1L << 8)) % 256;
    message[7] = data.micros % 256;
    memcpy(message + 8, data.readBuffer, 6);
    memcpy(message + 14, data.readBuffer + 8, 4);
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
    
    sendCast();
    
    cycle_time = micros();

    int length = PAYLOAD_NUM_SENSORS;
    int port, address, nSensors = 0;
    for (port = 0; port <= 1; port++) {
        for (address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
                if (readSensor(port, address, &data)) {
                    formatData(port, address, message + length);
                    length += PAYLOAD_PER_SENSOR;
                    nSamples += 1;
                    nSensors ++;
                } else {
                    nError ++;
                }
            }
        }
    }

    message[0] = nSensors;

    if (nSensors > 0)
        sendMessageProcess(message, length);
}

// UTILITY FUNCTION

void ERROR_FATAL(char *where) {
    Serial.printf("\nFATAL ERROR %s\n", where);
    ESP.restart();
}

// SHORT PRESS TO TURN ON SCREEN

void shortPress() {
    Serial.print("\nButton pressed, turn screen on");
    displayLoop(1);
}
