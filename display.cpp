#include <cstring>
#include <HardwareSerial.h>
#include <Esp.h>
#include <Wire.h>
#include "Actimetre.h"

// SSD1306 DRIVER

#define LCD_PAGES (LCD_V_RES / 8)
#define LCD_BUFFER_SIZE (LCD_H_RES * LCD_PAGES)

#include "fontdata.h"

#define FONT_PITCH_16 (FONT_WIDTH_16 + 1)
#define CHAR_PER_LINE_16 (LCD_H_RES / FONT_PITCH_16)

static void write_cmd(unsigned char cmd) {
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x80);
    Wire.write(cmd);
    Wire.endTransmission();
}

static void ssd1306_reset_page() {
    write_cmd(0x00);
    write_cmd(0x10);
    write_cmd(0xB0);
}

static const unsigned char ssd1306_init_cmd[] = {
    0xAE | 0x00,          // SET_DISP            off
    0xA0 | 0x01,          // SET_REG_REMAP       horizontal reverse start
    0xA8, LCD_V_RES - 1,  // SET_MUX_RATIO
    0xC0 | 0x08,          // SET_COM_OUT_DIR     horizontal reverse scan
    0xDA, 0x12,           // SET_COM_PIN_CFG     must be 0x02 if aspect ratio > 2:1
    0xD9, 0xF1,           // SET_PRECHARGE
    0xDB, 0x30,           // SET_VCOM_DESEL
    0x81, 0x7F,           // SET_CONTRAST
    0xA4,                 // SET_ENTIRE_ON
    0xA6 | 0x00,          // SET_NORM_INV (0x01 for inverse)
    0x8D, 0x14,           // SET_CHARGE_PUMP
    0xAE | 0x01,          // SET_DISP            on
    0x20, 0x02,           // SET_MEM_ADDRESS     page mode
};

static void ssd1306_init() {
    int i;
    for (i = 0; i < sizeof(ssd1306_init_cmd); i++) {
        write_cmd(ssd1306_init_cmd[i]);
    }
}

static const unsigned char ssd1306_off_cmd[] = {
    0xAE
};

static void ssd1306_off() {
    int i;
    for (i = 0; i < sizeof(ssd1306_off_cmd); i++) {
        write_cmd(ssd1306_off_cmd[i]);
    }
}

static const unsigned char ssd1306_on_cmd[] = {
    0xAF
};

static void ssd1306_on() {
    int i;
    for (i = 0; i < sizeof(ssd1306_on_cmd); i++) {
        write_cmd(ssd1306_on_cmd[i]);
    }
}

static unsigned char displayBuffer[LCD_BUFFER_SIZE];

static void write_page(int page) {
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x40);
    Wire.write(displayBuffer + page * LCD_H_RES, LCD_H_RES);
    Wire.endTransmission();
    ssd1306_reset_page();
}

static void ssd1306_showpages(int page0, int page1) {
    int page;

    for (page = page0; page <= page1; page++) {
        write_cmd(0x00);
        write_cmd(0x10);
        write_cmd(0xB0 | page);
        write_page(page);
    }
}

#define RSSI_POS 10
#define RSSI_X   (FONT_PITCH_16 * RSSI_POS)

static void ssd1306_show_rssi() {
    write_cmd(0xB0);
    write_cmd(0x00 | (RSSI_X & 0x0F));
    write_cmd(0x10 | (RSSI_X >> 4));
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x40);
    Wire.write(displayBuffer + RSSI_X, FONT_WIDTH_16);
    Wire.endTransmission();

    write_cmd(0xB1);
    write_cmd(0x00 | (RSSI_X & 0x0F));
    write_cmd(0x10 | (RSSI_X >> 4));
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x40);
    Wire.write(displayBuffer + LCD_H_RES + RSSI_X, FONT_WIDTH_16);
    Wire.endTransmission();

    ssd1306_reset_page();
}

static void ssd1306_showall() {
    ssd1306_showpages(0, LCD_PAGES - 1);
}

static char strbuf[50];
static unsigned halfMinutes = 0;
static int screenSaving = 0;

static void write_char16(int x, int y, char c) {
    int col, target, index = c - 32;

    target = y * LCD_H_RES + x * FONT_PITCH_16;
    for (col = 0; col < FONT_WIDTH_16; col++) {
        displayBuffer[target + col] = fontdata_top[index][col];
        displayBuffer[target + col + LCD_H_RES] = fontdata_bot[index][col];
    }
    displayBuffer[target + col] = 0x00;
    displayBuffer[target + col + LCD_H_RES] = 0x00;
}

static void writeLine16(int line, const char *message) {
    if (!my.displayPresent) return;

    int i;
    for (i = 0; i < strlen(message) && i < CHAR_PER_LINE_16; i++) {
        write_char16(i, line, message[i]);
    }
    int j;
    for (j = i * FONT_PITCH_16; j < LCD_H_RES; j++) {
        displayBuffer[line * LCD_H_RES + j] = 0x00;
        displayBuffer[(line + 1) * LCD_H_RES + j] = 0x00;
    }
    ssd1306_showpages(line, line + 1);
}

void initDisplay() {
    if (!my.displayPresent) return;
    ssd1306_init();
    memset(displayBuffer, 0x00, LCD_H_RES * LCD_V_RES / 8);
    ssd1306_showall();
}

static void displayPanel(int line) {
    if (nMissed[0] >= 100 || nError >= 100 || nMissed[1] >= 100 || nUnqueue >= 100
        || avgCycleTime[0] > cycleMicroseconds || avgCycleTime[1] > cycleMicroseconds) {
        Serial.println("\nSystem slowdown, rebooting");
        ESP.restart();
    }

    switch(line) {
    case 0:
        if (my.dualCore)
            sprintf(strbuf, "%dh%02d %.1f,%.1f", halfMinutes / 120, (halfMinutes % 120) / 2, avgCycleTime[1] / 1000.0, avgCycleTime[0] / 1000.0);
        else
            sprintf(strbuf, "%dh%02d %.1f", halfMinutes / 120, (halfMinutes % 120) / 2, avgCycleTime[1] / 1000.0);
        writeLine16(4, strbuf);
        break;

    case 1:
        if (my.dualCore)
            sprintf(strbuf, "M%d,%d Q%d E%d", nMissed[1], nMissed[0], nUnqueue, nError);
        else
            sprintf(strbuf, "M%d Q%d E%d", nMissed[1], nUnqueue, nError);
        writeLine16(6, strbuf);
        break;
    }
}

static char sensorLine[20];
void displayTitle(char *title) {
    writeLine16(0, title);
}

void displaySensors() {
    sprintf(sensorLine, "%-6s %s@%d", my.sensorList, my.boardName, cycleFrequency);
    writeLine16(2, sensorLine);
}

static void displayRssi() {
    int bars;
    if (my.rssi == 0) return;
    
    if (my.rssi > -32) bars = 7;
    else if (my.rssi <= -116) bars = 0;
    else bars = (my.rssi + 116) / 12;

    write_char16(RSSI_POS, 0, 0x81 + bars);
    ssd1306_show_rssi();
}

void displayLoop(int firstLoop) {
    static int scanLine = 1;
    static int saver = 0;

    if (firstLoop) {
        ssd1306_on();
        saver = 0;
        displayPanel(0);
        displayPanel(1);
        displayRssi();
    } else {
        if(isHalfMinutePast()) {
            halfMinutes ++;
            saver ++;
            
            scanLine = (scanLine + 1) % 2;
            if (saver < (SCREENSAVER_SECS / 30)) {
                displayPanel(scanLine);
                displayRssi();
            } else if (saver == (SCREENSAVER_SECS / 30)) {
                Serial.println("Screensaver");
                ssd1306_off();
            } else if (saver > (SCREENSAVER_SECS / 30)) {
                Serial.printf("%dh%02d %.1f,%.1f ", halfMinutes / 120, (halfMinutes % 120) / 2,
                              avgCycleTime[1] / 1000.0, avgCycleTime[0] / 1000.0);
                Serial.printf("M%d,%d Q%d E%d\n", nMissed[1], nMissed[0], nUnqueue, nError);
            }
        }
    }
}

void writeLine(char *message) {
    int scroll;
    for (scroll = 4; scroll < 6; scroll++)
        memcpy(displayBuffer + scroll * LCD_H_RES,
               displayBuffer + (scroll + 2) * LCD_H_RES,
               LCD_H_RES);
    ssd1306_showpages(4, 5);
    if (strlen(message) > CHAR_PER_LINE_16)
        message[CHAR_PER_LINE_16] = 0;
    writeLine16(6, message);
}
