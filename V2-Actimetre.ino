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
    Serial.println(title);

    netInit();
    clearSensors();
}

// MAIN LOOP

static time_t msgBootEpoch;
static int msgMicros;

#ifdef _V3
void formatHeader(unsigned char *message, int count)
#else    
void formatHeader(unsigned char *message)
#endif    
{
    getTimeSinceBoot(&msgBootEpoch, &msgMicros);
    message[0] = (msgBootEpoch >> 16) & 0xFF;
    message[1] = (msgBootEpoch >> 8) & 0xFF;
    message[2] = msgBootEpoch & 0xFF;
#ifdef _V3
    message[3] = count;
    message[4] = ((byte)my.rssi << 4)  | (byte)my.frequencyCode;
    message[5] = (msgMicros >> 16) & 0xFF;
    message[6] = (msgMicros >> 8) & 0xFF;
    message[7] = msgMicros & 0xFF;
#else    
    int millis = msgMicros / 1000L;
    // 76543210
    // rrrfffmm
    message[3] = (unsigned char)(millis / 256) | ((unsigned char)my.frequencyCode << 2) | ((unsigned char)my.rssi << 5) ;
    message[4] = millis % 256;
#endif    
}

void formatData(unsigned char *message) {
#ifdef _V3
    // TODO
#else
    int offsetMillis = getRelMicroseconds(msgBootEpoch, msgMicros) / 1000;
    message[0] = offsetMillis / 256;
    message[1] = offsetMillis % 256;
    memcpy(message + 2, dataPoint, 6);
    memcpy(message + 8, dataPoint + 8, 4);
#endif
}

void loop() {
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

#ifdef _V3
    message = msgQueueStore[msgIndex];
    int fifoCount = readFifo(0, 0, message + HEADER_LENGTH);
    if (fifoCount > 0) {
        formatHeader(message, fifoCount);
        queueMessage(&msgIndex);
        if (++msgIndex > QUEUE_SIZE) msgIndex = 0;
    }
#else                
    formatHeader(message);
    int offset = HEADER_LENGTH;
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
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

void ERROR_FATAL(char *where) {
    Serial.printf("\nFATAL ERROR %s\n", where);
    ESP.restart();
}

