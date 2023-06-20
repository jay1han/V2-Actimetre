#include <Wire.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include "Actimetre.h"

// GENERAL

void writeByte(int port, int address, int memory, unsigned char cmd) {
    //Logger.printf("writeByte(port=%d, address=0x%02X, register=0x%02X, data=0x%02X)\n", port, address, memory, cmd);
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    wire.beginTransmission(address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("writeByte() -> write(memory)");
    if (wire.write(cmd) != 1) ERROR_FATAL("writeByte() -> write(cmd)");
    if (wire.endTransmission() != 0) ERROR_FATAL("writeByte() -> endTransmission");
}

unsigned char readByte(int port, int address, int memory) {
    //Logger.printf("readByte(port=%d, address=0x%02X, register=0x%02X) ", port, address, memory);
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    unsigned char data;
    wire.beginTransmission(address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("readByte() -> write");
    if (wire.endTransmission() != 0) ERROR_FATAL("readByte() -> endTransmission");
    if (wire.requestFrom(address, 1) != 1) ERROR_FATAL("readByte() -> requestFrom");
    data = wire.read();
    wire.endTransmission(true);

    //Logger.printf("=0x%02X\n", data);
    return data;
}

// SENSORS (MPU-6050)

static void initSensor(int port, int address) {
    Serial.printf("Initializing %d%c", port + 1, 'A' + address);
    writeByte(port, MPU6050_ADDR + address, 0x6B, 0x80); // Power on
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x6B, 0x08); // Disable Temp
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x1C, 0x08); // Range +/-4g
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x68, 0x07); // Signal path reset
    delay(100);
//    writeByte(port, MPU6050_ADDR + address, 0x1A, 0x01); // enable DLPF
//    delay(100);

    int res = readByte(port, MPU6050_ADDR + address, 0x75);
    if (res != 0x68) {
        Serial.printf(" received WAI=0x%02X, BAD. Rebooting\n", res);
        ESP.restart();
    }

    Serial.println(" OK!");
}

static int detectSensor(int port, int address) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.endTransmission() == 0) {
        Serial.printf("Found sensor %d%c ", port + 1, 'A' + address);
        my.sensorPresent[port][address] = 1;
        initSensor(port, address);
        return 1;
    } else {
        return 0;
    }
}

int readSensor(int port, int address, unsigned char *dataPoint) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    int n;

    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(0x3B) != 1) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> write", port + 1, 'A' + address);
        return 0;
    }
    if (wire.endTransmission() != 0) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> endTransmission", port + 1, 'A' + address);
        return 0;
    }
    if (wire.requestFrom(MPU6050_ADDR + address, 12) != 12) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> requestFrom", port + 1, 'A' + address);
        return 0;
    }
    if ((n = wire.readBytes(dataPoint, 12)) != 12) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> readBytes", port + 1, 'A' + address);
        memset(dataPoint, 0, 12);
        return 0;
    }
    wire.endTransmission();
    return 1;
}

// GLOBAL

void deviceScanInit() {
    Serial.println("Checking I2C devices");

    Wire.beginTransmission(SSD1306_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.println("Display found on port 0");
        my.displayPort = 0;
        initDisplay();
    } else {
        Wire1.beginTransmission(SSD1306_ADDR);
        if (Wire1.endTransmission() == 0) {
            Serial.println("Display found on port 1");
            my.displayPort = 1;
            initDisplay();
        } else {
            Serial.println("No display found");
            my.displayPort = -1;
        }
    }

    int port, address;
    my.sensorBits = 0;
    my.nSensors = 0;
    for (port = 0; port <= 1; port++)
        for (address = 0; address <= 1; address++)
            if (detectSensor(port, address)) {
                my.sensorBits |= 1 << (port * 4 + address);
                my.nSensors++;
            }

    if (my.nSensors == 0) {
        Serial.println("No sensors found, rebooting");
        displayTitle("No sensors");
        sleep(5);
        ESP.restart();
    }
        
    my.msgLength = HEADER_LENGTH + DATA_LENGTH * my.nSensors;
    
    my.sensorList[0] = 0;
    for (port = 0; port < 2; port++) {
        if (my.sensorPresent[port][0] || my.sensorPresent[port][1])
            sprintf(my.sensorList + strlen(my.sensorList), "%d", port + 1);
        for (address = 0; address < 2; address++)
            if (my.sensorPresent[port][address])
                sprintf(my.sensorList + strlen(my.sensorList), "%c", 'A' + address);
    }
}

