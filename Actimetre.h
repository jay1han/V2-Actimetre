#ifndef ACTIMETRE_H
#define ACTIMETRE_H

#define VERSION_STR "v200>"

// CONSTANTS

#define ACTISERVER  "Actis"
#define MQTT_TOPIC  "Acti"
#define LONGPRESS_MILLIS  2000L

#define I2C_BAUDRATE 1000000  

#define SSD1306_ADDR 0x3C
#define MPU6050_ADDR 0x68

#define LCD_H_RES 128
#define LCD_V_RES 64

#define SCREENSAVER_MINS 5
#define MEASURE_CYCLES   3000
#define MEASURE_SECS     60
#define HEADER_LENGTH    5     // epoch(3), msec(2) 
#define DATA_LENGTH      12    // msec(2), accel(6), gyro(4)
#define BUFFER_LENGTH    (4 * DATA_LENGTH + HEADER_LENGTH)

// TYPES

typedef struct {
    long micros;
    unsigned char readBuffer[12];
} DataPoint;
typedef enum {Core0Net, Core1I2C, CoreNumMax} CoreNum;

typedef enum {
    BOARD_S2 = 0,
    BOARD_S2_CONNECTORS,
    BOARD_S2_NO_UART,
    BOARD_S3_I2C,
    BOARD_S3 = BOARD_S3_I2C,
    BOARD_TYPES
} BoardType;

// GLOBALS

typedef struct {
    BoardType boardType;
    int dualCore;
    char boardName[4];
    unsigned char mac[6];
    char macString[15];
    unsigned int clientId;
    char clientName[12];

    unsigned int serverId;
    char ssid[10];
    char serverIP[20];
    unsigned int serverPort;
    int rssi;
    time_t bootTime;
    float temperature;

    int displayPresent;
    int sensorPresent[2][2];
    unsigned char sensorBits;
    int nSensors;
    int msgLength;
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
void sendMessageProcess(unsigned char *message);
void netCore0(void *dummy_to_match_argument_signatue);
extern QueueHandle_t mqttQueue;
extern int mqttQueueSize;
extern int nUnqueue;

// devices.cpp
int readSensor(int port, int address, DataPoint *data);
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
void setupCore0(void (*code0Loop)(void*));

#define COLOR_WHITE   0x0FFF
#define COLOR_BLUE    0x000F
#define COLOR_RED     0x00F0
#define COLOR_GREEN   0x0F00
#define COLOR_YELLOW  0x0FF0
#define COLOR_MAGENTA 0x00FF
#define COLOR_CYAN    0x0F0F
#define COLOR_BLACK   0x0000

// clock.cpp
void feedWatchdog();
void initClock();
void initClockNoNTP();
int isMinutePast();
int isCastTime();
void getTimeSinceBoot(time_t *sec, int *usec);
int getRelMicroseconds(time_t sec, int usec);
unsigned long millis_diff(unsigned long end, unsigned long start);
unsigned long millis_diff_10(unsigned long end, unsigned long start);
unsigned long micros_diff(unsigned long end, unsigned long start);
unsigned long micros_diff_10(unsigned long end, unsigned long start);
void waitNextCycle(unsigned long cycle_time);
void logCycleTime(CoreNum coreNum, unsigned long time_spent);
void clearCycleTime();

// Actimetre.ino
void ERROR_FATAL(char *where);
void longPress();
void shortPress();

#endif //ACTIMETRE_H
