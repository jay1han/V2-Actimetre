#include <Wire.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include "Actimetre.h"

#define MPU6050_FIFO_CNT_H  0x72
#define MPU6050_FIFO_CNT_L  0x73
#define MPU6050_FIFO_DATA   0x74

// GENERAL

void writeByte(int port, int address, int memory, unsigned char cmd) {
    if (!my.hasI2C[port]) {
        Serial.printf("writeByte(port=%d, address=%d)\n", port, address);
        return;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("writeByte() -> write(memory)");
    if (wire.write(cmd) != 1) ERROR_FATAL("writeByte() -> write(cmd)");
    if (wire.endTransmission(true) != 0) ERROR_FATAL("writeByte() -> endTransmission");
}

unsigned char readByte(int port, int address, int memory) {
    if (!my.hasI2C[port]) {
        Serial.printf("readByte(port=%d, address=%d)\n", port, address);
        return 0;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    unsigned char data;
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("readByte() -> write");
    if (wire.endTransmission(false) != 0) ERROR_FATAL("readByte() -> endTransmission0");
    if (wire.requestFrom(MPU6050_ADDR + address, 1) != 1) ERROR_FATAL("readByte() -> requestFrom");
    data = wire.read();
    if (wire.endTransmission(true) != 0) ERROR_FATAL("readByte() -> endTransmission1");

    return data;
}

int readWord(int port, int address, int memory) {
    if (!my.hasI2C[port]) {
        Serial.printf("readWord(port=%d, address=%d)\n", port, address);
        return 0;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    byte bytes[2];
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write((unsigned char)memory) != 1) ERROR_FATAL("readWord() -> write");
    if (wire.endTransmission(false) != 0) ERROR_FATAL("readWord() -> endTransmission0");
    if (wire.requestFrom(MPU6050_ADDR + address, 2) != 2) ERROR_FATAL("readWord() -> requestFrom");
    if (wire.readBytes(bytes, 2) != 2) ERROR_FATAL("readWord() -> readBytes");
    if (wire.endTransmission(true) != 0) ERROR_FATAL("readWord() -> endTransmission1");

    return (int)bytes[0] << 8 | bytes[1];
}

// SENSORS (MPU-6050)

static void initSensor(int port, int address) {
    Serial.printf("Initializing %d%c", port + 1, 'A' + address);
    my.sensorType = readByte(port, address, 0x75);
    Serial.printf(" WAI=0x%02X, ", my.sensorType);
    if (my.sensorType == WAI_6050) {
        if (my.boardType >= BOARD_S3_6500) ERROR_FATAL("Wrong board type");
        Serial.print("MPU-6050");
    } else if (my.sensorType == WAI_6500) {
        if (my.boardType < BOARD_S3_6500) ERROR_FATAL("Wrong board type");
        Serial.print("MPU-6500");
    } else {
        Serial.println("BAD. Rebooting");
        RESTART(2);
    }

    if (my.sensorType == WAI_6050) {
        writeByte(port, address, 0x6C, 0x01); // Disable gz
        writeByte(port, address, 0x6B, 0x09); // Disable temp, Gx clock source
        writeByte(port, address, 0x19, 79);   // Sampling rate divider = 79 (100Hz)
        writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
        writeByte(port, address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    } else {
        writeByte(port, address, 0x6C, 0x01); // Disable gz
        writeByte(port, address, 0x6B, 0x08); // Disable temperature, osc clock source
        writeByte(port, address, 0x19, 9);    // Sampling rate divider
        writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
        writeByte(port, address, 0x1A, 0x01); // DLPF = 1
        writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
        writeByte(port, address, 0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0
        writeByte(port, address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    }

    Serial.println(" OK!");
}

#ifdef _V3
#define DUMP_SIZE   250
static byte dump[DUMP_SIZE];
static void clear1Sensor(int port, int address) {
    if (!my.hasI2C[port]) {
        Serial.printf("clear1Sensor(port=%d, address=%d)\n", port, address);
        return;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    int fifoCount = readWord(port, address, MPU6050_FIFO_CNT_H);
    Serial.printf("Reset sensor %d%c FIFO %d bytes\n", port + 1, address + 'A', fifoCount);
    writeByte(port, address, 0x6A, 0x04); // reset FIFO
    writeByte(port, address, 0x6A, 0x40); // enable FIFO
}

static void clear1SensorSome(int port, int address, int fifoCount) {
    if (!my.hasI2C[port]) {
        Serial.printf("clear1SensorSome(port=%d, address=%d)\n", port, address);
        return;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    int count;
    Serial.printf("Clearing %d bytes from FIFO", fifoCount);

    while (fifoCount > 0) {
        count = fifoCount;
        if (count > DUMP_SIZE) count = DUMP_SIZE;
        Serial.print(".");
        
        wire.beginTransmission(MPU6050_ADDR + address);
        if (wire.write(MPU6050_FIFO_DATA) != 1) {
            ERROR_FATAL("clear1Some() -> write");
            return;
        }
        if (wire.endTransmission(false) != 0) {
            ERROR_FATAL("clear1Some() -> endTransmission0");
            return;
        }
        if (wire.requestFrom(MPU6050_ADDR + address, count) != count) {
            ERROR_FATAL("clear1Some() -> requestFrom");
            return;
        }
        if (wire.readBytes(dump, count) != count) {
            ERROR_FATAL("clear1Some() -> readBytes");
            return;
        }
        if (wire.endTransmission(true) != 0) {
            ERROR_FATAL("clear1Some() -> endTransmission1");
            return;
        }

        fifoCount -= DUMP_SIZE;
    }
    Serial.println("Cleared");
}
#endif

#ifdef _V3
static void setSensor1Frequency(int port, int address, int frequency) {
    if (my.sensorType == WAI_6050) {
        int divider = 8000 / frequency - 1;
        Serial.printf("Sampling rate divider %d\n", divider);
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        writeByte(port, address, 0x19, (byte)divider); // Sampling rate divider
        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    } else {
        if (frequency <= 1000) {
            int divider = 1000 / frequency - 1;
            Serial.printf("Sampling rate divider %d\n", divider);
            writeByte(port, address, 0x6A, 0x04); // reset FIFO
            writeByte(port, address, 0x6C, 0x01); // Disable gz
            writeByte(port, address, 0x19, (byte)divider); // Sampling rate divider
            writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
            writeByte(port, address, 0x1A, 0x01); // DLPF = 1
            writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
            writeByte(port, address, 0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0
            writeByte(port, address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)
            writeByte(port, address, 0x6A, 0x40); // enable FIFO
        } else { 
            writeByte(port, address, 0x6A, 0x04); // reset FIFO

            if (frequency == 4000) {  // only accel
                writeByte(port, address, 0x6C, 0x07); // Disable gyro
                writeByte(port, address, 0x1A, 0x00); // DLPF = 0
                writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
                writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
                writeByte(port, address, 0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
                writeByte(port, address, 0x23, 0x08); // enable FIFO for accel (6 bytes per sample)
            } else if (frequency == 8000) { // only gyro
                writeByte(port, address, 0x6C, 0x39); // Disable accel and Gz
                writeByte(port, address, 0x1A, 0x07); // DLPF = 7
                writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
                writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
                writeByte(port, address, 0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
                writeByte(port, address, 0x23, 0x60); // enable FIFO for gx, gy (4 bytes per sample)
            } else {
                Serial.printf("Unhandled frequency %d\n", frequency);
                RESTART(2);
            }
            
            writeByte(port, address, 0x6A, 0x40); // enable FIFO
        }
    }
}
#endif

void setSensorsFrequency(int frequency) {
#ifdef _V3
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensorPresent[port][address]) {
                setSensor1Frequency(port, address, frequency);
                clear1Sensor(port, address);
            }
        }
    }
#endif
}

void clearSensors() {
#ifdef _V3
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
    if (wire.endTransmission(true) == 0) {
        Serial.printf("Found sensor %d%c ", port + 1, 'A' + address);
        my.sensorPresent[port][address] = 1;
        initSensor(port, address);
        return 1;
    } else {
        return 0;
    }
}

#ifdef _V3

int readFifo(int port, int address, byte *message) {
    if (!my.hasI2C[port]) {
        Serial.printf("readFifo(port=%d, address=%d)\n", port, address);
        return 0;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    byte *buffer = message + HEADER_LENGTH;
    int fifoCount;

    fifoCount = readWord(port, address, MPU6050_FIFO_CNT_H);
    fifoCount = (fifoCount / my.dataLength) * my.dataLength;
    if (fifoCount < my.dataLength) return 0;
    if (fifoCount > my.maxMeasures * my.dataLength) {
        fifoCount = my.maxMeasures * my.dataLength;
    }

    formatHeader(message, fifoCount / my.dataLength);
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(MPU6050_FIFO_DATA) != 1) {
        ERROR_FATAL("readFifo() -> write");
        return 0;
    }
    if (wire.endTransmission(false) != 0) {
        ERROR_FATAL("readFifo() -> endTransmission0");
        return 0;
    }
    if (wire.requestFrom(MPU6050_ADDR + address, fifoCount) != fifoCount) {
        ERROR_FATAL("readFifo() -> requestFrom");
        return 0;
    }
    if (wire.readBytes(buffer, fifoCount) != fifoCount) {
        ERROR_FATAL("readFifo() -> readBytes");
        return 0;
    }
    if (wire.endTransmission(true) != 0) {
        ERROR_FATAL("readFifo() -> endTransmission1");
        return 0;
    }
        
    return fifoCount / my.dataLength;
}

#else // _V3

#define MPU6050_DATA_REG   0x3B
#define MPU6050_DATA_SIZE  12
int readSensor(int port, int address, unsigned char *dataPoint) {
    if (!my.hasI2C[port]) {
        Serial.printf("readSensor(port=%d, address=%d)\n", port, address);
        return 0;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(MPU6050_DATA_REG) != 1) {
        Serial.printf("ERROR on sensor %d%c: readByte() -> write", port + 1, 'A' + address);
        return 0;
    }
    if (wire.endTransmission(false) != 0) {
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
    wire.endTransmission(true);
    return 1;
}
#endif

// GLOBAL

void deviceScanInit() {
    Serial.println("Checking I2C devices");

    my.displayPort = -1;
#ifndef _V3    
    if (my.hasI2C[0]) {
        Wire.beginTransmission(SSD1306_ADDR);
        if (Wire.endTransmission(true) == 0) {
            Serial.println("Display found on port 0");
            my.displayPort = 0;
            initDisplay();
        }
    }
    if (my.displayPort < 0 && my.hasI2C[1]) {
        Wire1.beginTransmission(SSD1306_ADDR);
        if (Wire1.endTransmission(true) == 0) {
            Serial.println("Display found on port 1");
            my.displayPort = 1;
            initDisplay();
        }
    }
    if (my.displayPort < 0) {
        Serial.println("No display found");
    }
#endif
    
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
        RESTART(5);
    }

#ifndef _V3    
    my.msgLength = HEADER_LENGTH + DATA_LENGTH * my.nSensors;
#endif
    
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
