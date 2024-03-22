#ifndef ACTIMETRE_H
#define ACTIMETRE_H

#define VERSION_STR "300"
#define _V3

// CONSTANTS

#define ACTISERVER  "Actis"
#define LONGPRESS_MILLIS  2000L

#ifdef _V3
#define I2C_BAUDRATE 3400000
#else
#define I2C_BAUDRATE 1000000
#endif

#define SSD1306_ADDR 0x3C
#define MPU6050_ADDR 0x68

#define LCD_H_RES 128
#define LCD_V_RES 64

#define MEASURE_SECS     60

#ifdef _V3
#define HEADER_LENGTH    8     // epoch(3), count(1), rssi(high)+freq(low) (1), usec(3)
#define DATA_LENGTH      10    // accel(6) gyro(4)
#define MAX_MEASURES     25
#define PER_CYCLE        20
#define READING_BASE     (1000000L * PER_CYCLE)
#define BUFFER_LENGTH    (MAX_MEASURES * DATA_LENGTH + HEADER_LENGTH)
#define QUEUE_SIZE       800
#else
#define QUEUE_SIZE       50
#define HEADER_LENGTH    5     // epoch(3), msec(2)
#define DATA_LENGTH      12    // msec(2), accel(6), gyro(4)
#define BUFFER_LENGTH    (4 * DATA_LENGTH + HEADER_LENGTH)
#endif

// TYPES

typedef enum {Core0Net, Core1I2C, CoreNumMax} CoreNum;

typedef enum {
    BOARD_S2 = 0,
    BOARD_S2_CONNECTORS,
    BOARD_S2_NO_UART,
    BOARD_S3_I2C,
    BOARD_S3 = BOARD_S3_I2C,
#ifdef _V3    
    BOARD_S3_NEWBOX,
    BOARD_S3_6500,
#endif    
    BOARD_TYPES
} BoardType;

// GLOBALS

typedef struct {
    BoardType boardType;
#ifdef _V3    
    byte sensorType;
#endif    
    bool hasI2C[2];
    bool ledRGB;
    int dualCore;
    char boardName[4];
    unsigned char mac[6];
    char macString[15];
    unsigned int clientId;
    char clientName[12];

    unsigned int serverId;
    char ssid[10];
    char serverIP[20];
    int rssi;
    time_t bootTime;
    int frequencyCode;

    int displayPort;
    int sensorPresent[2][2];
    unsigned char sensorBits;
    int nSensors;
#ifndef _V3    
    int msgLength;
#endif    
    char sensorList[10];
} MyInfo;

extern MyInfo my;
extern int cycleFrequency;
extern unsigned long cycleMicroseconds;

extern int nError;
extern int nMissed[];
extern float avgCycleTime[];

// INTERFACES

// display.cpp
void initDisplay();
void displayTitle(char *title);
void displaySensors();
void displayLoop(int firstLoop);
void writeLine(char *message);

// reseau.cpp
void netInit();
int isConnected();
void queueMessage(void *message);
void netCore0(void *dummy_to_match_argument_signature);
extern float queueFill;

#ifdef _V3
extern byte msgQueueStore[QUEUE_SIZE][BUFFER_LENGTH];
extern int msgIndex;
#endif

// devices.cpp
#ifdef _V3
int readFifo(int port, int address, byte *buffer);
#else
int readSensor(int port, int address, unsigned char *data);
#endif
void clearSensors();
void setSensorsFrequency(int frequency);
extern int nError;
void deviceScanInit();

// boards.cpp
extern BoardType boardType;
extern uint8_t PIN_BUTTON, PIN_LED, 
    PIN_UART_GND, PIN_UART_TX, PIN_UART_RX,
    PIN_I2C0_SDA, PIN_I2C0_SCL, PIN_I2C0_GND, PIN_I2C0_VCC,
    PIN_I2C1_SDA, PIN_I2C1_SCL, PIN_I2C1_GND, PIN_I2C1_VCC,
    PIN_I2C0_SDA_MUX, PIN_I2C0_SCL_MUX, PIN_I2C0_VCC_MUX, PIN_I2C0_GND_MUX;
void setupBoard();
void blinkLed(int color);
int buttonPressed();
void manageButton();
void setupCore0(void (*core0Loop)(void*));

//                      BBGGRR
#define COLOR_WHITE   0xFFFFFF
#define COLOR_RED     0x0000FF
#define COLOR_GREEN   0x00FF00
#define COLOR_BLUE    0xFF0000
#define COLOR_BLACK   0x000000
#define COLOR_SWAP    (-1)

// clock.cpp
void initClock(time_t bootEpoch);
void getTimeSinceBoot(time_t *sec, int *usec);
int getRelMicroseconds(time_t sec, int usec);
unsigned long millis_diff_10(unsigned long end, unsigned long start);
unsigned long micros_diff(unsigned long end, unsigned long start);
void waitNextCycle();
#ifndef _V3
int timeRemaining();
void catchUpCycle();
#endif
void logCycleTime(CoreNum coreNum, unsigned long time_spent);
void clearCycleTime();
extern unsigned int upTime;

// Actimetre.ino
void ERROR_FATAL(char *where);
void RESTART();
void longPress();
void shortPress();
#ifdef _V3
void formatHeader(unsigned char *message, int count);
#endif

#endif //ACTIMETRE_H
