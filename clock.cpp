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

static int micros_last = 0;
static int micros_offset = 0;
static int micros_actual;
static bool init_complete = false;
static time_t minuteTimer = 0;

static void startMinuteCount() {
    minuteTimer = time(NULL);
}

int isMinutePast() {
    unsigned long seconds = time(NULL) - minuteTimer;
    if (seconds >= 60) {
        minuteTimer = time(NULL);
        return 1;
    } else {
        return 0;
    }
}

void waitNextCycle(unsigned long cycle_time) {
    unsigned long remain, elapsed;

    do {
        elapsed = micros_diff(micros(), cycle_time);
        if (elapsed < cycleMicroseconds) {
            remain = cycleMicroseconds - elapsed;
            if (remain > 200L) {
                delayMicroseconds(remain / 2);
            }
        } else remain = 0L;
    } while (remain > 200L);

    while (micros_diff(micros(), cycle_time) < (cycleMicroseconds - 10));
}

void initClock() {
    time_t now = time(NULL);
    struct tm timeinfo;

    while (time(NULL) == now);
    now = time(NULL);
    startMinuteCount();
    
    micros_last = micros();
    micros_offset = ONE_MEGA - (micros_last % ONE_MEGA);
    Serial.printf("offset=%d ", micros_offset);
    init_complete = true;

    my.bootTime = now;
    gmtime_r(&now, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

void getTimeSinceBoot(time_t *sec, int *usec) {
    if (!init_complete) initClock();
    if (sec != NULL) *sec = time(NULL) - my.bootTime;
    if (micros() < micros_last) {
        micros_offset = (micros_offset + ROLLOVER_MICROS) % ONE_MEGA;
    }
    micros_last = micros();
    micros_actual = ((micros_last % ONE_MEGA) + micros_offset) % ONE_MEGA;
    *usec = micros_actual;
}

int getRelMicroseconds(time_t secRef, int usecRef) {
    if (micros() < micros_last) {
        micros_offset = (micros_offset + ROLLOVER_MICROS) % ONE_MEGA;
    }
    micros_last = micros();
    micros_actual = ((micros_last % ONE_MEGA) + micros_offset) % ONE_MEGA;
    time_t diff_sec = time(NULL) - secRef - my.bootTime;
    int diff_usec = micros_actual - usecRef;
    if (diff_usec < 0) {
        diff_usec += 1000000;
        diff_sec -= 1;
    }
    return diff_sec * 1000000 + diff_usec;
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

#define BOGUS_CYCLE     50000L
int nMissed[CoreNumMax] = {0, 0};
float avgCycleTime[CoreNumMax] = {0.0, 0.0};
static unsigned long mark = time(NULL);
static unsigned long nCycles[CoreNumMax] = {0, 0};

void logCycleTime(CoreNum coreNum, unsigned long time_spent) {
    if (time_spent > BOGUS_CYCLE) return;  // don't count outliers

    if (time(NULL) - mark > MEASURE_SECS || nCycles[1] > MEASURE_CYCLES) {
        nCycles[0] = nCycles[1] = 0;
        nMissed[0] = nMissed[1] = 0;
        nUnqueue = 0;
        nError = 0;
        mark = time(NULL);
    }
    avgCycleTime[coreNum] = (avgCycleTime[coreNum] * nCycles[coreNum] + time_spent) / (nCycles[coreNum] + 1);
    nCycles[coreNum] ++;

    if (nCycles[coreNum] < 10) return;
    
    if (nMissed[0] >= 100 || nError >= 10 || nMissed[1] >= 100 || nUnqueue >= 100
        || avgCycleTime[0] > cycleMicroseconds || avgCycleTime[1] > cycleMicroseconds) {
        Serial.printf("M%d,%d E%d Q%d Avg %.1f,%.1f\n", nMissed[1], nMissed[0], nError, nUnqueue,
                      avgCycleTime[1] / 1000.0, avgCycleTime[0] / 1000.0);
        Serial.println("System slowdown, rebooting");
        ESP.restart();
    }
}

void clearCycleTime() {
    nCycles[0] = nCycles[1] = 0;
    nMissed[0] = nMissed[1] = 0;
    nUnqueue = 0;
    nError = 0;
    mark = time(NULL);
}
