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

static unsigned long micros_last = 0L;
static unsigned long micros_offset = 0L;
static unsigned long micros_actual;
static bool init_complete = false;
static time_t minuteTimer = 0;
static time_t castTimer = 0;

static void startHalfMinuteCount() {
    minuteTimer = time(NULL);
    castTimer   = time(NULL);
}

int isCastTime() {
    unsigned long seconds = time(NULL) - castTimer;
    if (seconds >= 15) {
        castTimer = time(NULL);
        return 1;
    } else {
        return 0;
    }
}

int isHalfMinutePast() {
    unsigned long seconds = time(NULL) - minuteTimer;
    if (seconds >= 30) {
        minuteTimer = time(NULL);
        return 1;
    } else {
        return 0;
    }
}

void waitNextCycle(unsigned long cycle_time) {
    unsigned long remain, now;

    do {
        now = micros_diff(micros(), cycle_time);
        if (now < cycleMicroseconds) {
            remain = cycleMicroseconds - now;
            if (remain > 20L) {
                delayMicroseconds(remain / 2);
            }
        } else remain = 0L;
    } while (remain > 20L);
}

void initClock() {
    time_t now = time(NULL);
    struct tm timeinfo;

    Serial.print("initializing clock ");
    writeLine("Getting time");
    
    while (now < 946080000L) {  // Approx. 2000/1/1
        delay(100);
        now = time(NULL);
    }
    while (time(NULL) == now);
    my.bootTime = now;
    startHalfMinuteCount();
    
    micros_last = micros();
    micros_offset = ONE_MEGA - (micros_last % ONE_MEGA);
    Serial.printf("offset=%ld ", micros_offset);
    init_complete = true;

    now = time(NULL);
    gmtime_r(&now, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void initClockNoNTP() {
    startHalfMinuteCount();
    
    Serial.print("setting dummy clock ");
    timeval epoch = {1671408000L, 0};  // Approx. 2023/1/1
    settimeofday((const timeval*) &epoch, 0);

    time_t now = time(NULL);
    while (time(NULL) == now);
    my.bootTime = now;

    micros_last = micros();
    micros_offset = ONE_MEGA - (micros_last % ONE_MEGA);
    Serial.printf("offset=%ld ", micros_offset);
    init_complete = true;

    struct tm timeinfo;
    now = time(NULL);
    gmtime_r(&now, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void getTime(unsigned long *sec, unsigned long *usec) {
    if (!init_complete) initClock();
    if (micros() < micros_last) {
        micros_offset = (micros_offset + ROLLOVER_MICROS) % ONE_MEGA;
    }
    time((time_t*)sec);
    micros_last = micros();
    micros_actual = ((micros_last % ONE_MEGA) + micros_offset) % ONE_MEGA;
    *usec = micros_actual;
}

unsigned long millis_diff(unsigned long end, unsigned long start) {
    if (end >= start)
        return (((end % ONE_KILO) + ONE_KILO) - (start % ONE_KILO)) % ONE_KILO;
    else
        return (((end % ONE_KILO) + ROLLOVER_MILLIS + ONE_KILO) - (start % ONE_KILO)) % ONE_KILO;
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

unsigned long micros_diff_10(unsigned long end, unsigned long start) {
    if (end >= start)
        return (((end % TEN_MEGA) + TEN_MEGA) - (start % TEN_MEGA)) % TEN_MEGA;
    else
        return (((end % TEN_MEGA) + ROLLOVER_TEN_MICROS + TEN_MEGA) - (start % TEN_MEGA)) % TEN_MEGA;
}

int nMissed[CoreNumMax] = {0, 0};
#define BOGUS_CYCLE     500000L
float avgCycleTime[CoreNumMax];

void logCycleTime(CoreNum coreNum, unsigned long time_spent) {
    static unsigned long mark = time(NULL);
    static unsigned long nCycles[CoreNumMax] = {0, 0};

    if (time_spent > BOGUS_CYCLE) return;  // don't count outliers

    if (time(NULL) - mark > MEASURE_CYCLE) {
        nCycles[0] = nCycles[1] = 0;
        nMissed[0] = nMissed[1] = 0;
        nUnqueue = 0;
        nError = 0;
        mark = time(NULL);
    }
    avgCycleTime[coreNum] = (avgCycleTime[coreNum] * nCycles[coreNum] + time_spent) / (nCycles[coreNum] + 1);
    nCycles[coreNum] ++;
}

