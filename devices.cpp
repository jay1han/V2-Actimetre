#include <Wire.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include "Actimetre.h"

#define MPU6050_FIFO_CNT_H  0x72
#define MPU6050_FIFO_CNT_L  0x73
#define MPU6050_FIFO_DATA   0x74
#define MPU6050_INT_STATUS  0x3A
#define MPU6050_FIFO_OVER   0x10

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
    byte sensorType = readByte(port, address, 0x75);
    Serial.printf(" WAI=0x%02X, ", sensorType);
    if (sensorType != WAI_6050 && sensorType != WAI_6500) {
        Serial.println("BAD. Rebooting");
        RESTART(2);
    }
    my.sensor[port][address].type = sensorType;

    if (sensorType == WAI_6050) {
        writeByte(port, address, 0x6C, 0x01); // Disable gz
        writeByte(port, address, 0x6B, 0x09); // Disable temp, Gx clock source
        writeByte(port, address, 0x19, 79);   // Sampling rate divider = 79 (100Hz)
        writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
        writeByte(port, address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)
        writeByte(port, address, 0x38, 0x10); // enable FIFO interrupts
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        delay(1);
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
        writeByte(port, address, 0x38, 0x10); // enable FIFO interrupts
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        delay(1);
        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    }

    Serial.println(" OK!");
}

static void clear1Sensor(int port, int address) {
    if (!my.hasI2C[port]) {
        Serial.printf("clear1Sensor(port=%d, address=%d)\n", port, address);
        return;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;

    int fifoCount = readWord(port, address, MPU6050_FIFO_CNT_H);
    Serial.printf("Reset sensor %d%c FIFO %d bytes\n", port + 1, address + 'A', fifoCount);
    writeByte(port, address, 0x6A, 0x04); // reset FIFO
    delay(1);
    writeByte(port, address, 0x6A, 0x40); // enable FIFO
}

static void setSensor1Frequency(int port, int address) {
    if (my.sensor[port][address].type == WAI_6050) {
        int divider = 8000 / my.cycleFrequency - 1;
        Serial.printf("Sampling rate divider %d\n", divider);
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        delay(1);
        if (my.cycleFrequency <= 1000) {
            writeByte(port, address, 0x6C, 0x01); // Disable gz
            writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
            writeByte(port, address, 0x23, 0x68); // enable FIFO for gx, gy, accel (10 bytes per sample)
        } else {
            writeByte(port, address, 0x6C, 0x39); // Disable accel and Gz
            writeByte(port, address, 0x23, 0x60); // enable FIFO for gx, gy (4 bytes per sample)
        }
        writeByte(port, address, 0x19, (byte)divider); // Sampling rate divider
        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    } else {
        if (my.cycleFrequency <= 1000) {
            int divider = 1000 / my.cycleFrequency - 1;
            Serial.printf("Sampling rate divider %d\n", divider);
            writeByte(port, address, 0x6A, 0x04); // reset FIFO
            delay(1);
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
            delay(1);

            if (my.cycleFrequency <= 4000) {  // only accel
                writeByte(port, address, 0x6C, 0x07); // Disable gyro
                writeByte(port, address, 0x1A, 0x00); // DLPF = 0
                writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
                writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
                writeByte(port, address, 0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
                writeByte(port, address, 0x23, 0x08); // enable FIFO for accel (6 bytes per sample)
            } else if (my.cycleFrequency == 8000) { // only gyro
                writeByte(port, address, 0x6C, 0x39); // Disable accel and Gz
                writeByte(port, address, 0x1A, 0x07); // DLPF = 7
                writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
                writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
                writeByte(port, address, 0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
                writeByte(port, address, 0x23, 0x60); // enable FIFO for gx, gy (4 bytes per sample)
            } else {
                Serial.printf("Sensor %d%c unhandled frequency %d\n", 1 + port, 'A' + address, my.cycleFrequency);
                RESTART(2);
            }
            
            writeByte(port, address, 0x6A, 0x40); // enable FIFO
        }
    }
}

void setSensorsFrequency() {
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensor[port][address].type) {
                setSensor1Frequency(port, address);
                clear1Sensor(port, address);
            }
        }
    }
}

void setSamplingMode() {
    int perCycle = 100;
    
    for (int port = 0; port < 2; port++) {
        for (int address = 0; address < 2; address++) {
            int samplingMode;
            if (my.cycleFrequency <= 1000) {
                samplingMode = SAMPLE_ALL;
            } else if (my.sensor[port][address].type == WAI_6050) {
                samplingMode = SAMPLE_GYRO;
            } else if (my.cycleFrequency <= 4000) {
                samplingMode = SAMPLE_ACCEL;
            } else {
                samplingMode = SAMPLE_GYRO;
            }
            my.sensor[port][address].samplingMode = samplingMode;

            switch (samplingMode) {
            case SAMPLE_ACCEL:
                my.sensor[port][address].maxMeasures = 40;
                my.sensor[port][address].dataLength  = 6;
                if (perCycle > 30) perCycle = 30;
                break;

            case SAMPLE_GYRO:
                my.sensor[port][address].maxMeasures = 60;
                my.sensor[port][address].dataLength  = 4;
                if (perCycle > 40) perCycle = 40;
                break;

            default:
                my.sensor[port][address].maxMeasures = 25;
                my.sensor[port][address].dataLength  = 10;
                if (perCycle > 20) perCycle = 20;
                break;
            }

            Serial.printf("Sensor %d%c type %02X mode %d (max %d, sample %d bytes)\n",
                          port + 1, 'A' + address, my.sensor[port][address].type,
                          my.sensor[port][address].samplingMode,
                          my.sensor[port][address].maxMeasures,
                          my.sensor[port][address].dataLength);
        }
    }

    my.cycleMicroseconds = perCycle * 1000000 / my.cycleFrequency;
    Serial.printf("Sampling frequency %dHz(code %d), cycle time %dus\n",
                  my.cycleFrequency, my.frequencyCode, my.cycleMicroseconds);
}

void clearSensors() {
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensor[port][address].type) {
                clear1Sensor(port, address);
            }
        }
    }
}

static int detectSensor(int port, int address) {
    if (!my.hasI2C[port]) return 0;
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.endTransmission(true) == 0) {
        Serial.printf("Found sensor %d%c ", port + 1, 'A' + address);
        initSensor(port, address);
        return 1;
    } else {
        return 0;
    }
}

int readFifo(int port, int address, byte *message) {
    if (!my.hasI2C[port]) {
        Serial.printf("readFifo(port=%d, address=%d)\n", port, address);
        return 0;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    byte *buffer = message + HEADER_LENGTH;

    if (readByte(port, address, MPU6050_INT_STATUS) & MPU6050_FIFO_OVER) {
        char error[64];
        sprintf(error, "FIFO overflow %d%c", port + 1, address + 'A');
//        ERROR_REPORT(error);
        Serial.println(error);
        nMissed[Core1I2C]++;
        clear1Sensor(port, address);
        return 0;
    }
    
    int fifoCount = readWord(port, address, MPU6050_FIFO_CNT_H) & (my.sensor[port][address].type == WAI_6050 ? 1023 : 511);
    int dataLength  = my.sensor[port][address].dataLength;
    if (fifoCount < dataLength) return 0;
    int maxMeasures = my.sensor[port][address].maxMeasures;
    int timeOffset = 0;
    if (fifoCount > maxMeasures * dataLength) {
        timeOffset = (fifoCount / dataLength - maxMeasures) * (1000000 / my.cycleFrequency) ;
        fifoCount = maxMeasures * dataLength;
    }
    formatHeader(port, address, message, fifoCount / dataLength, timeOffset);

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
        
    return fifoCount / dataLength;
}

// GLOBAL

void deviceScanInit() {
    Serial.println("Checking I2C devices");

    my.displayPort = -1;

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

    my.sensorList[0] = 0;
    for (port = 0; port < 2; port++) {
        if (my.sensor[port][0].type || my.sensor[port][1].type)
            sprintf(my.sensorList + strlen(my.sensorList), "%d", port + 1);
        for (address = 0; address < 2; address++)
            if (my.sensor[port][address].type)
                sprintf(my.sensorList + strlen(my.sensorList), "%c", 'A' + address);
    }

    Serial.printf("Sensors %s\n", my.sensorList);
}
