#include <Wire.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include "Actimetre.h"

#define MPU6050_FIFO_CNT_H  0x72
#define MPU6050_FIFO_CNT_L  0x73
#define MPU6050_FIFO_DATA   0x74
#define MPU6050_INT_STATUS  0x3A
#define MPU6050_DATA_RDY    0x01
#define MPU6050_FIFO_OVER   0x10

// GENERAL

static char *sensorName(int port, int address) {
    static char name[3] = "01";
    name[0] = port + '1';
    if (my.sensor[port][address].type == WAI_6050) {
        name[1] = address + 'A';
    } else if (my.sensor[port][address].type == WAI_6500) {
        name[1] = address + 'a';
    } else name[1] = 'X' + address;
    return name;
}

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

static int initSensor(int port, int address) {
    Serial.printf("Initializing %s", sensorName(port, address));
    writeByte(port, address, 0x6B, 0x80);    // Reset
    delay(100);
    
    byte sensorType = readByte(port, address, 0x75);
    Serial.printf(" WAI=0x%02X, ", sensorType);
    if (sensorType != WAI_6050 && sensorType != WAI_6500) {
        Serial.println("BAD. Rebooting");
        RESTART(2);
    }
    my.sensor[port][address].type = sensorType;
    my.sensor[port][address].lastMessage = 0;
    my.sensor[port][address].nSamples = 0;
    my.sensor[port][address].nCycles = 0;
    Serial.printf(sensorName(port, address));

    if (sensorType == WAI_6050) {
        my.sensor[port][address].fifoOverflow = 1000;
//        writeByte(port, address, 0x6C, 0x01); // Disable gz
        writeByte(port, address, 0x6B, 0x09); // Disable temp, Gx clock source
        writeByte(port, address, 0x19, 7);   // Sampling rate divider = 7 (1kHz)
        writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
        writeByte(port, address, 0x23, 0x78); // enable FIFO for gx, gy, gz, accel (12 bytes per sample)
//        writeByte(port, address, 0x38, 0x11); // enable interrupts
//        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    } else {
        my.sensor[port][address].fifoOverflow = 500;
//        writeByte(port, address, 0x6C, 0x01); // Disable gz
        writeByte(port, address, 0x6B, 0x08); // Disable temperature, osc clock source
        writeByte(port, address, 0x19, 0);    // Sampling rate divider
        writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
        writeByte(port, address, 0x1A, 0x01); // DLPF = 1
        writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
        writeByte(port, address, 0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0
        writeByte(port, address, 0x23, 0x78); // enable FIFO for gx, gy, gz, accel (12 bytes per sample)
//        writeByte(port, address, 0x38, 0x11); // enable interrupts
//        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    }

    int fifoBytes = readWord(port, address, MPU6050_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" FIFO %d", fifoBytes);
    writeByte(port, address, 0x6A, 0x04); // reset FIFO
    fifoBytes = readWord(port, address, MPU6050_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" -> %d", fifoBytes);
    
    Serial.println(" OK!");
    return sensorType;
}

static int clear1Sensor(int port, int address) {
    if (!my.hasI2C[port]) {
        Serial.printf("clear1Sensor(port=%d, address=%d)\n", port, address);
        return 0;
    }
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    wire.setClock(MPU_BAUDRATE);

    int fifoBytes = readWord(port, address, MPU6050_FIFO_CNT_H) & 0x1FFF;
    Serial.printf("Reset sensor %s FIFO %d", sensorName(port, address), fifoBytes);
    
    writeByte(port, address, 0x6A, 0x04); // reset FIFO
    writeByte(port, address, 0x6A, 0x40); // enable FIFO
    fifoBytes = readWord(port, address, MPU6050_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" -> %d bytes\n", fifoBytes);
    return fifoBytes;
}

static void setSensor1Frequency(int port, int address) {
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    wire.setClock(MPU_BAUDRATE);
    
    if (my.sensor[port][address].type == WAI_6050) {
        int divider = 8000 / my.sampleFrequency - 1;
        Serial.printf("Sampling rate divider %d\n", divider);
        writeByte(port, address, 0x6A, 0x04); // reset FIFO
        if (my.sampleFrequency <= 1000) {
            writeByte(port, address, 0x6C, 0x00); // Enable accel
            writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
            writeByte(port, address, 0x23, 0x78); // enable FIFO for gx, gy, gz, accel (12 bytes per sample)
        } else {
            writeByte(port, address, 0x6C, 0x38); // Disable accel
            writeByte(port, address, 0x23, 0x70); // enable FIFO for gx, gy, gz (6 bytes per sample)
        }
        writeByte(port, address, 0x19, (byte)divider); // Sampling rate divider
        writeByte(port, address, 0x6A, 0x40); // enable FIFO
    } else {
        if (my.sampleFrequency <= 1000) {
            int divider = 1000 / my.sampleFrequency - 1;
            Serial.printf("Sampling rate divider %d\n", divider);
            writeByte(port, address, 0x6A, 0x04); // reset FIFO
            writeByte(port, address, 0x6B, 0x09); // Disable temperature, Gx clock source
            writeByte(port, address, 0x6C, 0x00); // Enable accel
            writeByte(port, address, 0x19, (byte)divider); // Sampling rate divider
            writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
            writeByte(port, address, 0x1A, 0x01); // DLPF = 1
            writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
            writeByte(port, address, 0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0
            writeByte(port, address, 0x23, 0x78); // enable FIFO for gx, gy, gz, accel (12 bytes per sample)
            writeByte(port, address, 0x6A, 0x40); // enable FIFO
        } else { 
            writeByte(port, address, 0x6A, 0x04); // reset FIFO

            if (my.sampleFrequency <= 4000) {  // only accel
                writeByte(port, address, 0x6B, 0x08); // Disable temperature, osc clock source
                writeByte(port, address, 0x6C, 0x07); // Disable gyro
                writeByte(port, address, 0x1A, 0x00); // DLPF = 0
                writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
                writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
                writeByte(port, address, 0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
                writeByte(port, address, 0x23, 0x08); // enable FIFO for accel (6 bytes per sample)
            } else if (my.sampleFrequency == 8000) { // only gyro
                writeByte(port, address, 0x6B, 0x09); // Disable temperature, Gx clock source
                writeByte(port, address, 0x6C, 0x38); // Disable accel
                writeByte(port, address, 0x1A, 0x07); // DLPF = 7
                writeByte(port, address, 0x1B, 0x00); // FCHOICE_B = b00
                writeByte(port, address, 0x1C, 0x08); // Accel range +/-4g
                writeByte(port, address, 0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
                writeByte(port, address, 0x23, 0x70); // enable FIFO for gx, gy, gz (6 bytes per sample)
            } else {
                Serial.printf("Sensor %s unhandled frequency %d\n", sensorName(port, address), my.sampleFrequency);
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

int setSamplingMode() {
    int perCycle = 100;
    int budget = 0;
    
    for (int port = 0; port < 2; port++) {
        for (int address = 0; address < 2; address++) {
            int samplingMode;
            if (my.sampleFrequency <= 1000) {
                samplingMode = SAMPLE_ALL;
            } else if (my.sensor[port][address].type == WAI_6050) {
                samplingMode = SAMPLE_GYRO;
            } else if (my.sampleFrequency <= 4000) {
                samplingMode = SAMPLE_ACCEL;
            } else {
                samplingMode = SAMPLE_GYRO;
            }
            my.sensor[port][address].samplingMode = samplingMode;

            switch (samplingMode) {
            case SAMPLE_ACCEL:
            case SAMPLE_GYRO:
                my.sensor[port][address].maxMeasures = 40;
                my.sensor[port][address].fifoThreshold = 10;
                my.sensor[port][address].dataLength  = 6;
                if (perCycle > 30) perCycle = 30;
                break;

            default:
                my.sensor[port][address].maxMeasures = 20;
                my.sensor[port][address].fifoThreshold = 5;
                my.sensor[port][address].dataLength  = 12;
                if (perCycle > 15) perCycle = 15;
                break;
            }
            if (my.sensor[port][address].type)
                budget += my.sampleFrequency * my.sensor[port][address].dataLength;

            Serial.printf("Sensor %s type %02X ",
                          sensorName(port, address),
                          my.sensor[port][address].type);
            Serial.printf("mode %d (max %d, sample %d bytes)\n",
                          my.sensor[port][address].samplingMode,
                          my.sensor[port][address].maxMeasures,
                          my.sensor[port][address].dataLength);
        }
    }
    budget *= 8 * 2;
    
    my.cycleMicroseconds = perCycle * 1000000 / my.sampleFrequency;
    Serial.printf("Sampling frequency %dHz(code %d), cycle time %dus.",
                  my.sampleFrequency, my.frequencyCode, my.cycleMicroseconds);
    Serial.printf(" Req. %d baud\n", budget);
    my.I2Cbudget = budget;
    return budget;
}

void clearSensors() {
    for (int port = 0; port <= 1; port++) {
        for (int address = 0; address <= 1; address++) {
            if (my.sensor[port][address].type) {
                clear1Sensor(port, address);
                my.sensor[port][address].nSamples = 0;
            }
        }
    }
}

static int detectSensor(int port, int address) {
    if (!my.hasI2C[port]) return 0;
    TwoWire &wire = (port == 0) ? Wire : Wire1;
    wire.setClock(MPU_BAUDRATE);
    
    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.endTransmission(true) == 0) {
        return initSensor(port, address);
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
    wire.setClock(MPU_BAUDRATE);
    
    byte *buffer = message + HEADER_LENGTH;

    int fifoBytes = readWord(port, address, MPU6050_FIFO_CNT_H) & 0x1FFF;
    if (fifoBytes > my.sensor[port][address].fifoOverflow) {
        char error[64];
        sprintf(error, "FIFO overflow %s", sensorName(port, address));
        Serial.println(error);
        ERROR_REPORT(error);
        my.nMissed[Core1I2C]++;
        fifoBytes = clear1Sensor(port, address);
    }

    int dataLength = my.sensor[port][address].dataLength;
    if (fifoBytes < dataLength) {
        Serial.printf("No data on %d%c\n", port + 1, 'A' + address);
        return 0;
    }
    
    int maxMeasures = my.sensor[port][address].maxMeasures;
    int timeOffset = 0;
    int moreToRead = fifoBytes - maxMeasures * dataLength;
    if (moreToRead > 0) {
        timeOffset = (moreToRead / dataLength) * (1000000 / my.sampleFrequency) ;
        fifoBytes = maxMeasures * dataLength;
    } else {
        fifoBytes = (fifoBytes / dataLength) * dataLength;
    }
    int fifoCount = fifoBytes / dataLength;
    int64_t now = formatHeader(port, address, message, fifoCount, timeOffset);

    if (my.sensor[port][address].lastMessage != 0) {
        int64_t span = (now - my.sensor[port][address].lastMessage) / (1000000 / my.sampleFrequency);
        if (fifoCount > (int)span + 2) {
            char error[64];
            sprintf(error, "FIFO mix-up %s: %d samples / %d cycles", sensorName(port, address), fifoCount, (int)span);
            Serial.println(error);
            ERROR_FATAL(error);
        }
        my.sensor[port][address].nCycles = (now - my.sensor[port][address].startClock) / (1000000 / my.sampleFrequency);
    } else {
        my.sensor[port][address].startClock = now - fifoCount * (1000000 / my.sampleFrequency);
        my.sensor[port][address].nCycles = fifoCount;
    }
    my.sensor[port][address].lastMessage = now;
    my.sensor[port][address].nSamples += fifoCount;

    wire.beginTransmission(MPU6050_ADDR + address);
    if (wire.write(MPU6050_FIFO_DATA) != 1) {
        ERROR_FATAL("readFifo() -> write");
        return 0;
    }
    if (wire.endTransmission(false) != 0) {
        ERROR_FATAL("readFifo() -> endTransmission0");
        return 0;
    }
    if (wire.requestFrom(MPU6050_ADDR + address, fifoBytes) != fifoBytes) {
        ERROR_FATAL("readFifo() -> requestFrom");
        return 0;
    }
    if (wire.readBytes(buffer, fifoBytes) != fifoBytes) {
        ERROR_FATAL("readFifo() -> readBytes");
        return 0;
    }
    if (wire.endTransmission(true) != 0) {
        ERROR_FATAL("readFifo() -> endTransmission1");
        return 0;
    }
    
    if (moreToRead / dataLength > my.sensor[port][address].fifoThreshold) {
        Serial.printf("FIFO %s more to read %d\n", sensorName(port, address), fifoCount);
        return 2;
    }
    return 1;
}

// GLOBAL

void deviceScanInit() {
    Serial.print("Checking I2C devices\n");

    my.displayPort = -1;

    if (my.hasI2C[0]) {
        Wire.setClock(DISPLAY_BAUDRATE);
        Wire.beginTransmission(SSD1306_ADDR);
        if (Wire.endTransmission(true) == 0) {
            Serial.print("SSD1306 found on port 0\n");
            my.displayPort = 0;
            initDisplay();
        }
    }
    if (my.displayPort < 0 && my.hasI2C[1]) {
        Wire1.setClock(DISPLAY_BAUDRATE);
        Wire1.beginTransmission(SSD1306_ADDR);
        if (Wire1.endTransmission(true) == 0) {
            Serial.print("ssd1306 found on port 1\n");
            my.displayPort = 1;
            initDisplay();
        }
    }
    if (my.displayPort < 0) {
        Serial.print("No display found\n");
    }
    
    int port, address;
    my.sensorBits = 0;
    my.nSensors = 0;
    for (port = 0; port <= 1; port++)
        for (address = 0; address <= 1; address++)
            if (detectSensor(port, address)) {
                my.sensorBits |= 1 << (port * 4 + address);
                if (my.sensor[port][address].type == WAI_6500)
                    my.sensorBits |= 1 << (port * 4 + address + 2);
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
        {
            sprintf(my.sensorList + strlen(my.sensorList), "%d", port + 1);
            for (address = 0; address < 2; address++) {
                if (my.sensor[port][address].type == WAI_6050) {
                    sprintf(my.sensorList + strlen(my.sensorList), "%c", 'A' + address);
                } else if (my.sensor[port][address].type == WAI_6500) {
                    sprintf(my.sensorList + strlen(my.sensorList), "%c", 'a' + address);
                }
            }
        }
    }

    Serial.printf("Sensors %s\n", my.sensorList);
    setSamplingMode();
}
