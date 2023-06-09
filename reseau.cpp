#include <WiFi.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <esp_wifi.h>
#include "Actimetre.h"

#define ACTI_PORT 2883

static WiFiClient wifiClient;

QueueHandle_t msgQueue;
#define QUEUE_SIZE 20
unsigned char msgBuffer[BUFFER_LENGTH];
float queueFill = 0.0;

#define INIT_LENGTH      13   // boardName = 3, MAC = 6, Sensors = 1, version = 3 : Total 13
#define RESPONSE_LENGTH  6    // actimId = 2, time = 4 : Total 6
static time_t getActimIdAndTime() {
    static unsigned char initMessage[INIT_LENGTH];
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
	delay(2000);
	ESP.restart();
    }
    unsigned char response[RESPONSE_LENGTH];

    err = 0;
    time_t timeout = time(NULL);
    while (err < 6) {
        err += wifiClient.read(response + err, RESPONSE_LENGTH - err);
        if ((time(NULL) - timeout) > 10) {
            Serial.println("No response from Actiserver");
	    writeLine("No response");
	    delay(2000);
            ESP.restart();
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

static void sendMessage(unsigned char *message) {
    int timeout = micros();
    int sent = 0;
    while (sent < my.msgLength && micros_diff(micros(), timeout) < 1000000L) {
        sent += wifiClient.write(message + sent, my.msgLength - sent);
    }
    if (sent != my.msgLength) {
        Serial.printf("Timeout sending data\n");
        writeLine("Timeout");
        delay(2000);
        ESP.restart();
    }
    logCycleTime(Core0Net, micros_diff(micros(), timeout));
}

void queueMessage(unsigned char *message) {
    xQueueSend(msgQueue, message, 0);
}

// Check connection still working and process message queue

int isConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nNetwork disconnected. Rebooting");
        writeLine("WiFi lost");
	delay(2000);
        ESP.restart();
    }

    if (!my.dualCore) {
        int availableSpaces = uxQueueSpacesAvailable(msgQueue);
        if (availableSpaces == 0) {
            nMissed[Core0Net] ++;
            xQueueReset(msgQueue);
            Serial.print("Queue full, cleared");
            queueFill = 0.0;
        } else {
            while (timeRemaining() > 0
                   && xQueueReceive(msgQueue, msgBuffer, 0) == pdTRUE) {
                sendMessage(msgBuffer);
            }
            availableSpaces = uxQueueSpacesAvailable(msgQueue);
            queueFill = 100.0 * (QUEUE_SIZE - availableSpaces) / QUEUE_SIZE;
        }    

        my.rssi = WiFi.RSSI();
    }
    return 1;
}

static void Core0Loop(void *dummy_to_match_argument_signatue) {
    Serial.printf("Core %d started\n", xPortGetCoreID());
    
    unsigned long startWork;
    for (;;) {
        while (xQueueReceive(msgQueue, msgBuffer, 1) != pdTRUE);
        startWork = micros();
        esp_task_wdt_reset();
        
        blinkLed(-1);
        sendMessage(msgBuffer);

        int availableSpaces = uxQueueSpacesAvailable(msgQueue);
        if (availableSpaces < QUEUE_SIZE / 5) {
            nMissed[Core0Net] += QUEUE_SIZE - availableSpaces;
            xQueueReset(msgQueue);
            Serial.print("Queue more than 80%, cleared");
            queueFill = 0.0;
        } else {
            queueFill = 100.0 * (QUEUE_SIZE - availableSpaces) / QUEUE_SIZE;
        }
        
        my.rssi = WiFi.RSSI();
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
	writeLine("No AP");
	delay(2000);
        ESP.restart();
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
	delay(2000);
        ESP.restart();
    }
}

static int waitConnected() {
    int wait = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.printf("%d ", WiFi.status());
        blinkLed(-1);
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
    my.rssi = WiFi.RSSI();

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
    WiFi.disconnect(true, true);
    delay(100);
    esp_wifi_set_max_tx_power(84);  // Max power 20dB
    WiFi.mode(WIFI_STA);

    storeMacAddress();

    int nScan;
    nScan = scanNetworks();
    
    findSsid(nScan);
    WiFi.scanDelete();
    WiFi.disconnect(true, true);
    delay(100);

    int i;
    for (i = 0; i < nActis; i++) {
        Serial.print(ssidList[i]);
        writeLine(ssidList[i]);
        strcpy(my.ssid, ssidList[i]);
        sscanf(my.ssid + 5, "%d", &my.serverId);

        char pass[] = "000animalerie";
        memcpy(pass, my.ssid + 5, 3);
        WiFi.begin(my.ssid, pass);
        Serial.print(" Connecting ");
        writeLine("Connecting");
        blinkLed(0);

        if (!waitConnected()) continue;
        if (printAndSaveNetwork()) break;
    }
    if (i == nActis) {
        Serial.println("\nCan't connect to any server, rebooting");
	writeLine("No server");
  esp_wifi_stop();
  delay(random(3000,6000));
        ESP.restart();
    }

    WiFi.setAutoReconnect(true);
    blinkLed(1);
    
    time_t bootEpoch = getActimIdAndTime();
    initClock(bootEpoch);

    msgQueue = xQueueCreate(QUEUE_SIZE, BUFFER_LENGTH);
    if (msgQueue == 0) {
        Serial.println("Error creating queue, rebooting");
	writeLine("OS error");
	delay(2000);
        ESP.restart();
    }

    if (my.dualCore)
        setupCore0(Core0Loop);
}
