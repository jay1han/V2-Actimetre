#include <Wire.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include "Actimetre.h"

#define MPU6050_FIFO_CNT_H  0x72
#define MPU6050_FIFO_CNT_L  0x73
#define MPU6050_FIFO_DATA   0x74
#define MPU6050_INT_STAT    0x3A

// GENERAL

void writeByte(int port, int address, int memory, unsigned char cmd) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    wire.beginTransmission(address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("writeByte() -> write(memory)");
    if (wire.write(cmd) != 1) ERROR_FATAL("writeByte() -> write(cmd)");
    if (wire.endTransmission() != 0) ERROR_FATAL("writeByte() -> endTransmission");
}

unsigned char readByte(int port, int address, int memory) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    unsigned char data;
    wire.beginTransmission(address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("readByte() -> write");
    if (wire.endTransmission() != 0) ERROR_FATAL("readByte() -> endTransmission0");
    if (wire.requestFrom(address, 1) != 1) ERROR_FATAL("readByte() -> requestFrom");
    data = wire.read();
    if (wire.endTransmission() != 0) ERROR_FATAL("readByte() -> endTransmission1");

    return data;
}

// SENSORS (MPU-6050)

static void initSensor(int port, int address) {
    Serial.printf("Initializing %d%c", port + 1, 'A' + address);
    writeByte(port, MPU6050_ADDR + address, 0x6B, 0x00); // Power on
    delay(100);
    writeByte(port, MPU6050_ADDR + address, 0x1C, 0x08); // Range +/-4g
    writeByte(port, MPU6050_ADDR + address, 0x19, 79); // Sampling rate divider = 79 (100Hz)
    writeByte(port, MPU6050_ADDR + address, 0x6A, 0x04); // reset FIFO
    writeByte(port, MPU6050_ADDR + address, 0x38, 0x10); // enable FIFO overflow interrupt
    writeByte(port, MPU6050_ADDR + address, 0x6A, 0x40); // enable FIFO
    writeByte(port, MPU6050_ADDR + address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)

    int res = readByte(port, MPU6050_ADDR + address, 0x75);
    if (res != 0x68) {
        Serial.printf(" received WAI=0x%02X, BAD. Rebooting\n", res);
        ESP.restart();
    }

    Serial.println(" OK!");
}

#ifdef _FIFO
byte fifoBuffer[1024];

static void clear1Sensor(int port, int address) {
    byte lsb, msb, overflow;
    int fifoCount;
    
    msb = readByte(port, MPU6050_ADDR + address, MPU6050_FIFO_CNT_H);
    lsb = readByte(port, MPU6050_ADDR + address, MPU6050_FIFO_CNT_L);
    fifoCount = msb << 8 | lsb;
    if (fifoCount <= 0 || fifoCount > 1024) return;
    for (int count = 0; count < fifoCount; count++) {
        fifoBuffer[count] = readByte(port, MPU6050_ADDR + address, MPU6050_FIFO_DATA);
    }
}
#endif

void setSensorsFrequency(int frequency) {
#ifdef _FIFO
    int divider = 8000 / frequency - 1;
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
                clear1Sensor(port, address);
                writeByte(port, MPU6050_ADDR + address, 0x19, (byte)divider); // Sampling rate divider
                writeByte(port, MPU6050_ADDR + address, 0x6A, 0x04); // reset FIFO
                writeByte(port, MPU6050_ADDR + address, 0x6A, 0x40); // enable FIFO
                writeByte(port, MPU6050_ADDR + address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)
            }
        }
    }
#endif
}

void clearSensors() {
#ifdef _FIFO
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
                clear1Sensor(port, address);
            }
        }
    }
#endif
}

static int detectSensor(int port, int address) {
    if (!my.hasI2C[port]) return 0;
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

#ifdef _FIFO

int readFifo(int port, int address) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    byte lsb, msb, overflow;
    int fifoCount, count;

    overflow = readByte(port, MPU6050_ADDR + address, MPU6050_INT_STAT);
    if (overflow & 0x10) {
        Serial.printf("Sensor %d%c FIFO overflow, clearing\n", port + 1, address + 'A');
        clear1Sensor(port, address);
    }
    
    msb = readByte(port, MPU6050_ADDR + address, MPU6050_FIFO_CNT_H);
    lsb = readByte(port, MPU6050_ADDR + address, MPU6050_FIFO_CNT_L);
    fifoCount = msb << 8 | lsb;
    Serial.printf("FIFO %d bytes to read\n", fifoCount);

    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(MPU6050_FIFO_DATA) != 1) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> write", port + 1, 'A' + address);
        return 0;
    }
    if (wire.endTransmission() != 0) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> endTransmission", port + 1, 'A' + address);
        return 0;
    }
    if (wire.requestFrom(MPU6050_ADDR + address, fifoCount) != fifoCount) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> requestFrom", port + 1, 'A' + address);
        return 0;
    }
    if (wire.readBytes(fifoBuffer + count, fifoCount) != fifoCount) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> readBytes", port + 1, 'A' + address);
        return 0;
    }
    wire.endTransmission();
        
    return fifoCount;
}

#else // _FIFO

#define MPU6050_DATA_REG   0x3B
#define MPU6050_DATA_SIZE  12
int readSensor(int port, int address, unsigned char *dataPoint) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(MPU6050_DATA_REG) != 1) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> write", port + 1, 'A' + address);
        return 0;
    }
    if (wire.endTransmission() != 0) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> endTransmission", port + 1, 'A' + address);
        return 0;
    }
    if (wire.requestFrom(MPU6050_ADDR + address, MPU6050_DATA_SIZE) != MPU6050_DATA_SIZE) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> requestFrom", port + 1, 'A' + address);
        return 0;
    }
    if ((n = wire.readBytes(dataPoint, MPU6050_DATA_SIZE)) != MPU6050_DATA_SIZE) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> readBytes", port + 1, 'A' + address);
        memset(dataPoint, 0, 12);
        return 0;
    }
    wire.endTransmission();
    return 1;
}
#endif

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

    Serial.printf("Sensors %s\n", my.sensorList);
}
