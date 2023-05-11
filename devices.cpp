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
    writeByte(port, MPU6050_ADDR + address, 0x6B, 0x80);
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x6B, 0x08);
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x1C, 0x08);
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x68, 0x07);

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

int readSensor(int port, int address, DataPoint *data) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    int i, n = 14;
    memset(data, 0, sizeof(DataPoint));

    getTime(&data->time, &data->micros);
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(0x3B) != 1) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> write", port + 1, 'A' + address);
        return 0;
    }
    if (wire.endTransmission() != 0) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> endTransmission", port + 1, 'A' + address);
        return 0;
    }
    if (wire.requestFrom(MPU6050_ADDR + address, 14) != 14) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> requestFrom", port + 1, 'A' + address);
        return 0;
    }
    if ((n = wire.readBytes(data->readBuffer, 14)) != 14) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> readBytes", port + 1, 'A' + address);
        memset(data, 0, sizeof(DataPoint));
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
        my.displayPresent = 1;
        initDisplay();
    }

    int port, address;
    for (port = 0; port <= 1; port++)
        for (address = 0; address <= 1; address++)
            if (detectSensor(port, address)) my.nSensors++;

    my.sensorList[0] = 0;
    
    for (port = 0; port < 2; port++) {
        if (my.sensorPresent[port][0] || my.sensorPresent[port][1])
            sprintf(my.sensorList + strlen(my.sensorList), "%d", port + 1);
        for (address = 0; address < 2; address++)
            if (my.sensorPresent[port][address])
                sprintf(my.sensorList + strlen(my.sensorList), "%c", 'A' + address);
    }
}

