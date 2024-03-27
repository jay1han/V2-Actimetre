#include <time.h>
#include <Esp.h>
#include <time.h>
#include <esp_sleep.h>
#include "Actimetre.h"

#define ONE_KILO    1000L
#define TEN_KILO    10000L
#define ONE_MEGA    1000000L
#define TEN_MEGA    10000000L

#define ROLLOVER_MILLIS      296L
#define ROLLOVER_TEN_MILLIS  7296L
#define ROLLOVER_MICROS      967296L
#define ROLLOVER_TEN_MICROS  4967296L

static bool init_complete = false;
static time_t minuteTimer = 0;
static int64_t nextMicros;
unsigned int upTime = 0;

static int64_t getAbsMicros() {
    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    return (int64_t)timeofday.tv_sec * 1000000L + (int64_t)timeofday.tv_usec;
}

#ifdef _V3
void waitNextCycle() {
    while (nextMicros - getAbsMicros() >= 10L);
    nextMicros = getAbsMicros() + (int64_t)my.cycleMicroseconds;
}

#else
int timeRemaining() {
    int64_t remain = nextMicros - getAbsMicros();
    if (remain < 10L) return 0;
    else return (int)remain;
}

void waitNextCycle() {
    if (time(NULL) - minuteTimer >= 60) {
        minuteTimer = time(NULL);
        upTime++;
    }

    if (timeRemaining() > 1000) displayLoop(0);
    while (timeRemaining() > 2000) delayMicroseconds(1000);
    while (timeRemaining() >= 10);
    nextMicros += (int64_t)my.cycleMicroseconds;
}

void catchUpCycle() {
    while (timeRemaining() < 50) nextMicros += (int64_t)my.cycleMicroseconds;
    waitNextCycle();
}
#endif

void clearNextCycle() {
    nextMicros = getAbsMicros();
}

void initClock(time_t bootEpoch) {
    struct timeval timeofday = {bootEpoch, 0};
    settimeofday(&timeofday, 0);
    nextMicros = (int64_t)bootEpoch * 1000000L + 1000000L;
    
    minuteTimer = time(NULL);
    my.bootTime = bootEpoch;
    init_complete = true;

    struct tm timeinfo;
    gmtime_r(&bootEpoch, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void getTimeSinceBoot(time_t *r_sec, int *r_usec) {
    int64_t clock = getAbsMicros() - (int64_t)my.bootTime * 1000000L;
    int64_t sec = clock / 1000000L;
    int64_t usec = clock % 1000000L;
    if (r_sec != NULL) *r_sec = (time_t)sec;
    if (r_usec != NULL) *r_usec = (int)usec;
}

int getRelMicroseconds(time_t secRef, int usecRef) {
    int64_t diff = getAbsMicros() - (int64_t)(my.bootTime + secRef) * 1000000L - (int64_t)usecRef;
    return (int)diff;
}

unsigned long millis_diff_10(unsigned long end, unsigned long start) {
    if (end >= start)
        return (((end % TEN_KILO) + TEN_KILO) - (start % TEN_KILO)) % TEN_KILO;
    else
        return (((end % TEN_KILO) + ROLLOVER_TEN_MILLIS + TEN_KILO) - (start % TEN_KILO)) % TEN_KILO;
}

unsigned long micros_diff(unsigned long end, unsigned long start) {
    if (end >= start)
        return (((end % ONE_MEGA) + ONE_MEGA) - (start % ONE_MEGA)) % ONE_MEGA;
    else
        return (((end % ONE_MEGA) + ROLLOVER_MICROS + ONE_MEGA) - (start % ONE_MEGA)) % ONE_MEGA;
}

int nMissed[CoreNumMax] = {0, 0};
float avgCycleTime[CoreNumMax] = {0.0, 0.0};
static unsigned long nCycles[CoreNumMax] = {0, 0};
static time_t clear = time(NULL);

void logCycleTime(CoreNum coreNum, unsigned long time_spent) {
    time_t life;
    getTimeSinceBoot(&life, NULL);
    if (life > 0xFEFFFF) {
        ERROR_FATAL("Alive over 6 months, rebooting");
    }

    avgCycleTime[coreNum] = (avgCycleTime[coreNum] * nCycles[coreNum] + time_spent) / (nCycles[coreNum] + 1);
    nCycles[coreNum] ++;

    static time_t print = time(NULL);
    
    if (coreNum == Core0Net && time(NULL) != print) {
        Serial.printf("M%d,%d E%d Q%.1f Avg %.1f,%.1f\n", nMissed[1], nMissed[0], nError, queueFill,
                      avgCycleTime[1] / 1000.0, avgCycleTime[0] / 1000.0);
        print = time(NULL);
        if (time(NULL) - clear > MEASURE_SECS) {
            clearCycleTime();
        }
    }
    if (coreNum == Core1I2C && (nError >= MEASURE_SECS || nMissed[1] >= MEASURE_SECS)) {
        ERROR_FATAL("System slowdown, rebooting");
    }
    
}

void clearCycleTime() {
    nCycles[0] = nCycles[1] = 0;
    nMissed[0] = nMissed[1] = 0;
    nError = 0;
    clear = time(NULL);
}
