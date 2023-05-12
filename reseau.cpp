#include <WiFi.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "Actimetre.h"

#define MQTT_PORT 2883

static WiFiClient wifiClient;
static int testMode = 0;

QueueHandle_t mqttQueue;
#define MQTT_QUEUE_SIZE 100
int nUnqueue = 0;

static unsigned char mqttBuffer[MQTT_MSG_LENGTH];
static unsigned char mqttCastMessage[MQTT_CAST_LENGTH];
// marker = 1(0x40), boardName = 3, MAC = 6, bootTime = 4 : Total 14

// Register by HTTP to Actiserver

static void getActimId() {
    Serial.print("Getting name ");

    mqttCastMessage[0] = 0x40;
    memcpy(mqttCastMessage + 1, my.boardName, 3);
    memcpy(mqttCastMessage + 4, my.mac, 6);
    mqttCastMessage[10] = my.bootTime / (1 << 24);
    mqttCastMessage[11] = (my.bootTime / (1 << 16)) % 256;
    mqttCastMessage[12] = (my.bootTime / (1 << 8)) % 256;
    mqttCastMessage[13] = my.bootTime % 256;

    int i;
    for (i = 0; i < 14; i++) {
        Serial.printf("%02x ", mqttCastMessage[i]);
    }
    Serial.println();
    wifiClient.write(mqttCastMessage, MQTT_CAST_LENGTH);
    unsigned char response[2];

    delay(1000);
    int err = 0;
    while (err == 0)
        err = wifiClient.read(response, 2);

    Serial.printf("Return from read(): %d, response=%02X%02X\n", err, response[0], response[1]);
    my.clientId = response[0] * 256 + response[1];

    sprintf(my.clientName, "Actim%04d", my.clientId);
    Serial.println(my.clientName);

    mqttCastMessage[0] = 0x20;

    char message[16];
    sprintf(message, "%s%04d  %d", VERSION_STR, my.clientId, my.serverId);
    displayTitle(message);
}

// MQTT functions

static void sendMessage(unsigned char *message, int length) {
    int sent = wifiClient.write(message, length);
    if (sent != length) {
        Serial.printf("Sent only %d bytes out of %d\n", sent, length);
        ESP.restart();
    }
}

static void queueMessage(unsigned char *message, int length) {
    if (xQueueSend(mqttQueue, message, 1) != pdPASS) {
        nUnqueue++;
    }
}

void sendMessageProcess(unsigned char *message, int length) {
    if (testMode) return;
    if (my.boardType >= BOARD_S3) {
        queueMessage(message, length);
    } else {
        sendMessage(message, length);
    }
}

void sendCast() {
    if (testMode) return;
    if (my.boardType >= BOARD_S3) return; // Done in Core0
    if (isCastTime())
        sendMessage(mqttCastMessage, MQTT_CAST_LENGTH);
}

static void Core0Loop(void *dummy_to_match_argument_signatue) {
    if (testMode) return;

    Serial.printf("Core %d started\n", xPortGetCoreID());
    mqttQueue = xQueueCreate(MQTT_QUEUE_SIZE, MQTT_MSG_LENGTH);
    if (mqttQueue == 0) {
        Serial.println("Error creating queue, rebooting");
        ESP.restart();
    }
    
    int availableSpaces;
    unsigned long startWork;
    for (;;) {
        while (xQueueReceive(mqttQueue, mqttBuffer, 1) != pdTRUE) {
            esp_task_wdt_reset();
        }
        startWork = micros();
        esp_task_wdt_reset();
        
        if (!isConnected()) ESP.restart();

        blinkLed(-1);

        if (isCastTime())
            sendMessage(mqttCastMessage, MQTT_CAST_LENGTH);

        int messageLength = PAYLOAD_NUM_SENSORS + mqttBuffer[0] * PAYLOAD_PER_SENSOR;
        sendMessage(mqttBuffer, messageLength);
        if ((availableSpaces = uxQueueSpacesAvailable(mqttQueue)) < MQTT_QUEUE_SIZE / 5) {
            nMissed[Core0Net] += MQTT_QUEUE_SIZE - availableSpaces;
            xQueueReset(mqttQueue);
            Serial.print("MQTT Queue less than 20% remaining, cleared");
        }
        
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
    blinkLed(0);
    nScan = WiFi.scanNetworks();
    Serial.printf("Done. Found %d APs\n", nScan);

    if (nScan <= 0) {
        Serial.println("\nCan't find AP. Rebooting");
        ESP.restart();
    }

    return nScan;
}

static void findSsid(int nScan, char *ssid) {
    int i;
    for (i = 0; i < nScan; i++) {
        strncpy(ssid, WiFi.SSID(i).c_str(), 9);
        ssid[9] = 0;  // always careful
        if (strncmp(ACTISERVER, ssid, 5) == 0) break;
    }
    if (i >= nScan) {
        Serial.println("\nCan't find server, rebooting");
        ESP.restart();
    }

    Serial.print(ssid);
    writeLine(ssid);
    strcpy(my.ssid, ssid);
    sscanf(ssid + 5, "%d", &my.serverId);
    if (my.serverId == 997) {
	      my.serverId = 117;
        my.serverPort = 60000;
    }
    else {
        my.serverPort = 0;
    }
}

static void waitConnected() {
    int wait = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.printf("%d ", WiFi.status());
        blinkLed(-1);
        wait++;
        if (wait > 10) {
            Serial.println("Failed!\nRebooting");
            ESP.restart();
        }
        delay(1000);
    }
}

static void printAndSaveNetwork() {
    Serial.print(" Connected! IP=");
    Serial.print(WiFi.localIP());
    if (my.serverPort > 0)
        strcpy(my.serverIP, "home.jayhan.name");
    else
        strcpy(my.serverIP, WiFi.gatewayIP().toString().c_str());
    Serial.print(" AP IP=");
    Serial.println(my.serverIP);

    String ipString = WiFi.localIP().toString();
    char myIPstring[20];
    int trail = 0;
    trail = ipString.indexOf('.');
    trail = ipString.indexOf('.', trail + 1);
    
    strcpy(myIPstring, ipString.substring(trail + 1).c_str());
    writeLine(myIPstring);
    my.rssi = WiFi.RSSI();

    wifiClient.setNoDelay(true);
    int err = wifiClient.connect(my.serverIP, MQTT_PORT + my.serverPort);
    Serial.printf("connect() returned %d\n", err);
}

// Huge and ugly but that's the way it is.

void netInit() {
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);

    storeMacAddress();

    int nScan;
    nScan = scanNetworks();
    
    if (buttonPressed()) {
        testMode = 1;
        displayTitle("TEST MODE");
        initClockNoNTP();
    } else {
        char ssid[10] = "";
        findSsid(nScan, ssid);

        WiFi.scanDelete();
        WiFi.disconnect(true, true);
        delay(100);

        char pass[] = "000animalerie-eops";
        memcpy(pass, ssid + 5, 3);
        WiFi.begin(ssid, pass);
        Serial.print(" Connecting ");
        writeLine("Connecting");
        blinkLed(0);

        waitConnected();

        printAndSaveNetwork();

        WiFi.setAutoReconnect(true);
        blinkLed(1);
    
        if (my.serverPort > 0) {
            Serial.print("Skipping NTP");
            initClockNoNTP();
        } else {
            Serial.print("Getting time ");
            configTime(0, 0, my.serverIP);
            initClock();
        }
        
        getActimId();
    }    
    displayLoop(1);
    displayLoop(1);

    if (my.boardType >= 3)
        setupCore0(Core0Loop);
}

// Check connection still working, otherwise reboot

int isConnected() {
    if (testMode) return 1;
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nNetwork disconnected. Rebooting");
        writeLine("WiFi lost");
        ESP.restart();
    }

    my.rssi = WiFi.RSSI();
    return 1;
}
