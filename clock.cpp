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
static hw_timer_t *watchdogTimer;
static int64_t nextMicros;

static int64_t getAbsMicros() {
    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    return (int64_t)timeofday.tv_sec * 1000000L + (int64_t)timeofday.tv_usec;
}

int timeRemaining() {
    int64_t remain = nextMicros - getAbsMicros();
    if (remain < 0L) return 0;
    else return (int)remain;
}

unsigned int upTime = 0;

void waitNextCycle() {
    timerWrite(watchdogTimer, 0);
    if (time(NULL) - minuteTimer >= 60) {
        minuteTimer = time(NULL);
        upTime++;
    }

    if (timeRemaining() > 1000) displayLoop(0);
    while (timeRemaining() > 2000) delayMicroseconds(1000);
    while (timeRemaining() >= 10);
    while (timeRemaining() < 10) nextMicros += (int64_t)cycleMicroseconds;
}

static void watchdogReset() {
    ESP.restart();
}

void initClock(time_t bootEpoch) {
    struct timeval timeofday = {bootEpoch, 0};
    settimeofday(&timeofday, 0);
    nextMicros = (int64_t)bootEpoch * 1000000L + 500000L;
    
    minuteTimer = time(NULL);
    my.bootTime = bootEpoch;
    init_complete = true;

    struct tm timeinfo;
    gmtime_r(&bootEpoch, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    watchdogTimer = timerBegin(0, 80, true);
    if (watchdogTimer != NULL) {
        timerAttachInterrupt(watchdogTimer, watchdogReset, true);
        timerAlarmWrite(watchdogTimer, 2000000, false);
        timerAlarmEnable(watchdogTimer);
    }
}

void getTimeSinceBoot(time_t *r_sec, int *r_usec) {
    int64_t clock = getAbsMicros() - (int64_t)my.bootTime * 1000000L;
    int64_t sec = clock / 1000000L;
    int64_t usec = clock % 1000000L;
    *r_sec = (time_t)sec;
    *r_usec = (int)usec;
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

#define BOGUS_CYCLE     50000L
int nMissed[CoreNumMax] = {0, 0};
float avgCycleTime[CoreNumMax] = {0.0, 0.0};
static unsigned long mark = time(NULL);
static unsigned long nCycles[CoreNumMax] = {0, 0};

void logCycleTime(CoreNum coreNum, unsigned long time_spent) {
    if (time_spent > BOGUS_CYCLE) return;  // don't count outliers

    if (time(NULL) - mark > MEASURE_SECS) {
        nCycles[0] = nCycles[1] = 0;
        nMissed[0] = nMissed[1] = 0;
        nError = 0;
        mark = time(NULL);
    }
    avgCycleTime[coreNum] = (avgCycleTime[coreNum] * nCycles[coreNum] + time_spent) / (nCycles[coreNum] + 1);
    nCycles[coreNum] ++;

    if (nCycles[coreNum] < 10) return;
    
    if (nMissed[0] >= 100 || nError >= 10 || nMissed[1] >= 100
        || avgCycleTime[0] > cycleMicroseconds || avgCycleTime[1] > cycleMicroseconds) {
        Serial.printf("M%d,%d E%d Q%.1f Avg %.1f,%.1f\n", nMissed[1], nMissed[0], nError, queueFill,
                      avgCycleTime[1] / 1000.0, avgCycleTime[0] / 1000.0);
        Serial.println("System slowdown, rebooting");
        ESP.restart();
    }
}

void clearCycleTime() {
    nCycles[0] = nCycles[1] = 0;
    nMissed[0] = nMissed[1] = 0;
    nError = 0;
    mark = time(NULL);
}
