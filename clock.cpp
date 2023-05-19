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

static unsigned long micros_last = 0;
static unsigned long micros_offset = 0;
static unsigned long micros_actual;
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

void waitNextCycle() {
    unsigned long startMicros = (micros() / cycleMicroseconds) * cycleMicroseconds;
    long remain = cycleMicroseconds - micros_diff(micros(), startMicros);

    Serial.printf("Cycle %u, remain %d", startMicros, remain);
    if (remain < 0) {
        Serial.println("Missed Cycle");
    } else {
        if (remain > 2000L) delayMicroseconds(remain - 2000L);
        do {
            remain = cycleMicroseconds - micros_diff(micros(), startMicros);
        } while (remain > 2);
    }
    Serial.printf(", final %u\n", micros());
}

void initClock(time_t bootEpoch) {
    micros_last = micros();
    struct timeval timeofday = {bootEpoch, micros_last % ONE_MEGA} ;
    settimeofday(&timeofday, 0);
    micros_offset = 0;
    
    startMinuteCount();
    my.bootTime = bootEpoch;
    init_complete = true;

    struct tm timeinfo;
    gmtime_r(&bootEpoch, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
}

static int getMicrosActual(time_t *sec) {
    if (micros() < micros_last) {
        micros_offset = micros_offset + (ROLLOVER_MICROS % ONE_MEGA);
    }
    micros_last = micros();
    micros_actual = (micros_last % ONE_MEGA) + micros_offset;
    while (micros_actual >= ONE_MEGA) {
	*sec ++;
	micros_actual -= ONE_MEGA;
    }
    return micros_actual;
}

void getTimeSinceBoot(time_t *sec, int *usec) {
    if (sec != NULL) *sec = time(NULL) - my.bootTime;
    *usec = getMicrosActual(sec);
}

int getRelMicroseconds(time_t secRef, int usecRef) {
    time_t sec = time(NULL);
    int diff_usec = getMicrosActual(&sec) - usecRef;
    if (diff_usec < 0) {
        diff_usec += 1000000;
        sec -= 1;
    }
    return (sec - secRef - my.bootTime) * 1000000 + diff_usec;
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

    if (time(NULL) - mark > MEASURE_SECS) {
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
