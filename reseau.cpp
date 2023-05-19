#include <WiFi.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "Actimetre.h"

#define ACTI_PORT 2883

static WiFiClient wifiClient;
static int testMode = 0;

QueueHandle_t msgQueue;
#define QUEUE_SIZE 100
unsigned char msgBuffer[BUFFER_LENGTH];
int nUnqueue = 0;

#define INIT_LENGTH      10   // boardName = 3, MAC = 6, Sensors = 1 : Total 10
#define RESPONSE_LENGTH  6    // actimId = 2, time = 4 : Total 6
static void getActimId() {
    static unsigned char initMessage[INIT_LENGTH];
    Serial.print("Getting name and time ");

    memcpy(initMessage, my.boardName, 3);
    memcpy(initMessage + 3, my.mac, 6);
    initMessage[9] = my.sensorBits;

    int err = 0;
    err = wifiClient.write(initMessage, INIT_LENGTH);
    if (err < INIT_LENGTH) {
	Serial.printf("\nSent %d bytes != %d\n", err, INIT_LENGTH);
	ESP.restart();
    }
    unsigned char response[RESPONSE_LENGTH];

    err = 0;
    time_t timeout = time(NULL);
    while (err < 6) {
        err += wifiClient.read(response + err, RESPONSE_LENGTH - err);
        if ((time(NULL) - timeout) > 10) {
            Serial.println("No response from Actiserver");
            ESP.restart();
        }
    }

    Serial.printf("read: %d bytes, response=%02X%02X, %02X %02X %02X %02X\n",
                  err, response[0], response[1],
                  response[2], response[3], response[4], response[5]);
    
    my.clientId = response[0] * 256 + response[1];
    int bootTime = (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
    struct timeval timeofday = {bootTime, 500000} ;
    settimeofday(&timeofday, 0);
    
    sprintf(my.clientName, "Actim%04d", my.clientId);
    Serial.printf("%s, boot time = %d\n", my.clientName, bootTime);

    char message[16];
    sprintf(message, "%s%04d  %03d", VERSION_STR, my.clientId, my.serverId);
    displayTitle(message);
}

// Messaging functions

static void sendMessage(unsigned char *message) {
    int timeout = micros();
    int sent = 0;
    while (sent < my.msgLength && micros_diff(micros(), timeout) < 1000) {
        sent += wifiClient.write(message + sent, my.msgLength - sent);
    }
    if (sent != my.msgLength) {
        Serial.printf("Sent only %d bytes out of %d\n", sent, my.msgLength);
        wifiClient.stop();
        ESP.restart();
    }
}

static void queueMessage(unsigned char *message) {
    if (xQueueSend(msgQueue, message, 0) != pdPASS) {
        nUnqueue++;
    }
}

void sendMessageProcess(unsigned char *message) {
    if (testMode) return;
    queueMessage(message);
}

// Check connection still working and process message queue

int isConnected(unsigned long startMicros) {
    if (testMode) return 1;
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nNetwork disconnected. Rebooting");
        writeLine("WiFi lost");
        ESP.restart();
    }

    int availableSpaces;
    if ((availableSpaces = uxQueueSpacesAvailable(msgQueue)) < QUEUE_SIZE / 5) {
        nMissed[Core0Net] += QUEUE_SIZE - availableSpaces;
        xQueueReset(msgQueue);
        Serial.print("Queue more than 80%, cleared");
    } else {
        while ((cycleMicroseconds - micros_diff(micros(), startMicros) > 1200L)
               && xQueueReceive(msgQueue, msgBuffer, 0) == pdTRUE) {
            sendMessage(msgBuffer);
        }
    }    

    my.rssi = WiFi.RSSI();
    return 1;
}

static void Core0Loop(void *dummy_to_match_argument_signatue) {
    if (testMode) return;

    Serial.printf("Core %d started\n", xPortGetCoreID());
    
    int availableSpaces;
    unsigned long startWork;
    for (;;) {
        while (xQueueReceive(msgQueue, msgBuffer, 1) != pdTRUE) {
            esp_task_wdt_reset();
        }
        startWork = micros();
        esp_task_wdt_reset();
        
        blinkLed(-1);

        sendMessage(msgBuffer);
        if ((availableSpaces = uxQueueSpacesAvailable(msgQueue)) < QUEUE_SIZE / 5) {
            nMissed[Core0Net] += QUEUE_SIZE - availableSpaces;
            xQueueReset(msgQueue);
            Serial.print("Queue more than 80%, cleared");
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
        my.serverId = 44;
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
    Serial.print(" Server=");
    Serial.println(my.serverIP);

    String ipString = WiFi.localIP().toString();
    char myIPstring[20];
    int trail = 0;
    trail = ipString.indexOf('.');
    trail = ipString.indexOf('.', trail + 1);
    
    strcpy(myIPstring, ipString.substring(trail + 1).c_str());
    writeLine(myIPstring);
    my.rssi = WiFi.RSSI();

    Serial.printf("Socket to %s:%d\n", my.serverIP, ACTI_PORT + my.serverPort);
    int err = wifiClient.connect(my.serverIP, ACTI_PORT + my.serverPort);
    Serial.printf("connect() returned %d\n", err);
    wifiClient.setNoDelay(false);
    wifiClient.setTimeout(1);
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
    
        getActimId();
        initClock();
    }    
    displayLoop(2);

    msgQueue = xQueueCreate(QUEUE_SIZE, BUFFER_LENGTH);
    if (msgQueue == 0) {
        Serial.println("Error creating queue, rebooting");
        ESP.restart();
    }

    if (my.dualCore)
        setupCore0(Core0Loop);
}
