#include <WiFi.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <esp_wifi.h>
#include "Actimetre.h"

#define ACTI_PORT 2883

static WiFiClient wifiClient;
static QueueHandle_t msgQueue;

#ifdef STATIC_QUEUE
static StaticQueue_t msgQueueStatic;
static byte msgQueueItems[QUEUE_SIZE * sizeof(int)];
#endif

#define INIT_LENGTH      13   // boardName = 3, MAC = 6, Sensors = 1, version = 3 : Total 13
#define RESPONSE_LENGTH  6    // actimId = 2, time = 4 : Total 6

static time_t getActimIdAndTime() {
    byte initMessage[INIT_LENGTH];
    Serial.print("Getting name and time ");

    memcpy(initMessage, my.boardName, 3);
    memcpy(initMessage + 3, my.mac, 6);
    initMessage[9] = my.sensorBits;
    memcpy(initMessage + 10, VERSION_STR, 3);

    int err = 0;
    err = wifiClient.write(initMessage, INIT_LENGTH);
    if (err < INIT_LENGTH) {
	Serial.printf("\nSent %d bytes != %d\n", err, INIT_LENGTH);
	writeLine("Init failed");
        RESTART(2);
    }
    byte response[RESPONSE_LENGTH];

    err = 0;
    time_t timeout = time(NULL);
    while (err < 6) {
        err += wifiClient.read(response + err, RESPONSE_LENGTH - err);
        if ((time(NULL) - timeout) > 10) {
            Serial.println("No response from Actiserver");
	    writeLine("No response");
            RESTART(2);
        }
    }

    Serial.printf("read: %d bytes, response=%02X%02X, %02X %02X %02X %02X\n",
                  err, response[0], response[1],
                  response[2], response[3], response[4], response[5]);
    
    my.clientId = response[0] * 256 + response[1];
    time_t bootTime = (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
    
    sprintf(my.clientName, "Actim%04d", my.clientId);
    Serial.println(my.clientName);

    char message[16];
    sprintf(message, "v%s>%04d  %03d", VERSION_STR, my.clientId, my.serverId);
    displayTitle(message);

    return bootTime;
}

// Messaging functions

static int DATA_LENGTH[] = {10, 6, 4, 10};

static void sendMessage(byte *message) {
    int timeout = micros();
    int epochSec = message[0] << 16 | message[1] << 8 | message[2];
    int count = message[3] & 0x3F;
    int msgLength;
    if (message[0] == 0xFF) {
        msgLength = HEADER_LENGTH + count;
        Serial.printf("REPORT message length %d\n", msgLength);
    } else {
        int dataLength = DATA_LENGTH[(message[4] >> 3) & 0x03];
        msgLength = HEADER_LENGTH + dataLength * count;
    }
    
    int sent = 0;
    while (sent < msgLength && micros_diff(micros(), timeout) < 1000000L) {
        sent += wifiClient.write(message + sent, msgLength - sent);
    }

    if (message[0] == 0xFF) {
        if (sent != msgLength)
            Serial.printf("Timeout sending message\n");
        return;
    }

    if (sent != msgLength) {
        Serial.printf("Timeout sending data\n");
        writeLine("Timeout");
        RESTART(2);
    }

#ifdef PROFILE_NETWORK
    int port = (message[3] >> 7) & 1;
    int address = (message[3] >> 6) & 1;
    static int inSec[2][2];
    static int thisSec[2][2];
    static int nMessages[2][2];
    nMessages[port][address] ++;
    inSec[port][address] += count;
    if (epochSec != thisSec[port][address]) {
        Serial.printf("%d%c: %d samples in %d packets (mode %d, %d bytes), avg. %.1f/s\n",
                      port + 1, address + 'A', inSec[port][address], nMessages[port][address],
                      my.sensor[port][address].samplingMode, my.sensor[port][address].dataLength,
                      (float)inSec[port][address] / nMessages[port][address]);
        inSec[port][address] = 0;
        nMessages[port][address] = 0;
        thisSec[port][address] = epochSec;
    }
#endif    

    logCycleTime(Core0Net, micros_diff(micros(), timeout));
}

void queueMessage(void *message) {
    int index = *(int*)message;
    if (index <= 0 || index >= QUEUE_SIZE) {
        char error[16];
        sprintf(error, "QUEUE %d", index);
        Serial.println(error);
        ERROR_FATAL0(error);
    }
    if (xQueueSend(msgQueue, message, 0) != pdTRUE) {
        Serial.println("Error queueing. Queue full?");
        RESTART(2);
    }
}

void readRssi() {
    int rssi = WiFi.RSSI();
    if (rssi != 0) {
        if (rssi > -28) my.rssi = 7;
        else if (rssi <= -91) my.rssi = 0;
        else my.rssi = (rssi + 91) / 9;
    } else my.rssi = 0;
}

// Check connection still working and process message queue

int isConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nNetwork disconnected. Rebooting");
        writeLine("WiFi lost");
        RESTART(2);
    }

    return 1;
}

static void Core0Loop(void *dummy_to_match_argument_signature) {
    Serial.printf("Core %d started\n", xPortGetCoreID());

    unsigned long startWork;
    for (;;) {
//        TEST_LOCAL(1);
        int index;
        while (xQueueReceive(msgQueue, &index, 1) != pdTRUE) {
        }
        startWork = micros();

        if (index <= 0 || index >= QUEUE_SIZE) {
            char error[16];
            sprintf(error, "0LOOP %d", index);
            Serial.println(error);
#ifdef STATIC_QUEUE            
            dump(msgQueueItems, QUEUE_SIZE * sizeof(int));
#endif            
            ERROR_FATAL0(error);
        }
        sendMessage(msgQueueStore[index]);

#ifndef TIGHT_QUEUE        
        int availableSpaces = uxQueueSpacesAvailable(msgQueue);
        if (availableSpaces < QUEUE_SIZE / 5) {
            my.nMissed[Core0Net] ++;
            xQueueReset(msgQueue);
            Serial.println("Queue more than 80%, cleared");
            my.queueFill = 0.0;
        } else {
            my.queueFill = 100.0 * (QUEUE_SIZE - availableSpaces) / QUEUE_SIZE;
        }
#endif        

#ifdef LOG_QUEUE
        static time_t timer = 0;
        if (time(NULL) != timer) {
            timer = time(NULL);
            Serial.printf("===== Dump at %d\n", timer);
            dump(msgQueueItems, 32 * sizeof(int));
        }
#endif        

        int command = wifiClient.read();
        if (command >= 0) {
            Serial.printf("Remote command 0x%02X\n", command);
            switch(command & REMOTE_COMMAND) {
            case REMOTE_BUTTON:
                Serial.println("Simulated button press");
                manageButton(1);
                break;
                
            case REMOTE_RESTART:
                Serial.println("Remote restart");
                RESTART(5);
            }
        }
        
        readRssi();
        logCycleTime(Core0Net, micros_diff(micros(), startWork));
    }
}

// Network initializations

static void storeMacAddress() {
    unsigned char mac[6];
    WiFi.macAddress(mac);
    memcpy(my.mac, mac, 6);
    sprintf(my.macString, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("MAC address ");
    Serial.println(my.macString);
    writeLine(my.macString);
}

static int scanNetworks() {
    int nScan;
    Serial.print("Scan... ");
    writeLine("Scanning");
    nScan = WiFi.scanNetworks();
    Serial.printf("Done. Found %d APs\n", nScan);

    if (nScan <= 0) {
        Serial.println("\nCan't find AP. Rebooting");
	writeLine("No AP");
        RESTART(2);
    }
    return nScan;
}

static char ssidList[5][10];
static int nActis = 0;
static void findSsid(int nScan) {
    int i;
    char ssid[10];
    for (i = 0; i < nScan; i++) {
        strncpy(ssid, WiFi.SSID(i).c_str(), 9);
        ssid[9] = 0;  // always careful
        if (strncmp(ACTISERVER, ssid, 5) == 0) {
            strcpy(ssidList[nActis], ssid);
            nActis ++;
            if (nActis == 5) break;
        }
    }
    if (nActis == 0) {
        Serial.println("\nCan't find server, rebooting");
	writeLine("No Actis");
        RESTART(2);
    }
}

static int waitConnected() {
    int wait = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.printf("%d ", WiFi.status());
        blinkLed(COLOR_SWAP);
        wait++;
        if (wait > 10) {
            Serial.println("Failed!");
            return 0;
        }
        delay(1000);
    }
    return 1;
}

static int printAndSaveNetwork() {
    Serial.print(" Connected! IP=");
    Serial.print(WiFi.localIP());
    strcpy(my.serverIP, WiFi.gatewayIP().toString().c_str());
    Serial.print(" Server=");
    Serial.println(my.serverIP);

    String ipString = WiFi.localIP().toString();
    char myIPstring[20];
    int trail = 0;
    trail = ipString.indexOf('.');
    trail = ipString.indexOf('.', trail + 1);
    
    strcpy(myIPstring, ipString.substring(trail + 1).c_str());
    writeLine(myIPstring);
    readRssi();

    Serial.printf("Socket to %s:%d\n", my.serverIP, ACTI_PORT);
    int err = wifiClient.connect(my.serverIP, ACTI_PORT);
    Serial.printf("connect() returned %d\n", err);
    if (err == 0) {
        Serial.println("Connection refused");
        return 0;
    }
    wifiClient.setNoDelay(false);
    return 1;
}

// Huge and ugly but that's the way it is.

void netInit() {
    blinkLed(COLOR_SWAP);
    WiFi.disconnect(true, true);
    delay(100);
    esp_wifi_set_max_tx_power(84);  // Max power 20dB
    WiFi.mode(WIFI_STA);

    storeMacAddress();

    blinkLed(COLOR_SWAP);
    int nScan;
    nScan = scanNetworks();
    
    findSsid(nScan);
    WiFi.scanDelete();
    WiFi.disconnect(true, true);
    delay(100);

    int i;
    for (i = 0; i < nActis; i++) {
        blinkLed(COLOR_SWAP);
        Serial.print(ssidList[i]);
        writeLine(ssidList[i]);
        strcpy(my.ssid, ssidList[i]);
        sscanf(my.ssid + 5, "%d", &my.serverId);

        char pass[] = "000animalerie";
        memcpy(pass, my.ssid + 5, 3);
        WiFi.begin(my.ssid, pass);
        Serial.print(" Connecting ");
        writeLine("Connecting");

        if (!waitConnected()) continue;
        if (printAndSaveNetwork()) break;
    }
    if (i == nActis) {
        Serial.println("\nCan't connect to any server, rebooting");
	writeLine("No server");
        esp_wifi_stop();
        RESTART(2);
    }

    WiFi.setAutoReconnect(true);
    
    time_t bootEpoch = getActimIdAndTime();
    initClock(bootEpoch);

#ifdef STATIC_QUEUE    
    msgQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(int), msgQueueItems, &msgQueueStatic);
#else    
    msgQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
#endif    
    if (msgQueue == 0) {
        Serial.println("Error creating queue, rebooting");
	writeLine("OS error");
        RESTART(1);
    }
    my.queueFill = 0.0;

    setupCore0(Core0Loop);
}

static void _test(int type) {
    switch (type) {
    case 1:
        if (getAbsMicros() > 10000000) {
            ERROR_FATAL0("Test FATAL0");
        }
        break;
    }
}
