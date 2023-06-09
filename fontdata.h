#define FONT_WIDTH_16 8

const unsigned char fontdata_top[128][FONT_WIDTH_16] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,},
    {0x00,0x3F,0x3F,0x00,0x00,0x3F,0x3F,0x00,},
    {0x30,0xFF,0xFF,0x30,0x30,0xFF,0xFF,0x30,},
    {0x30,0x78,0xCC,0xFF,0xFF,0xCC,0x8C,0x00,},
    {0x0F,0x0F,0x8F,0xCF,0xE0,0x70,0x38,0x18,},
    {0x3C,0xFF,0xC3,0xFF,0xBC,0x00,0x80,0x80,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x00,0x00,0xF0,0xF8,0x1C,0x0E,0x07,0x03,},
    {0x00,0x00,0x03,0x07,0x0E,0x1C,0xF8,0xF0,},
    {0x0C,0x1C,0x30,0xFC,0xFC,0x30,0x1C,0x0C,},
    {0xC0,0xC0,0xC0,0xF8,0xF8,0xC0,0xC0,0xC0,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x80,0xC0,0xE0,0x60,0x00,},
    {0xFC,0xFE,0x87,0xC3,0xE3,0x77,0xFE,0xFC,},
    {0x00,0x0C,0x0E,0xFF,0xFF,0x00,0x00,0x00,},
    {0x0C,0x0E,0x07,0x83,0xC3,0xE7,0x7E,0x3C,},
    {0x06,0x07,0x03,0xC3,0xC3,0xE7,0xFE,0x3C,},
    {0xE0,0xF0,0xB8,0x9C,0x8E,0xFF,0xFF,0x80,},
    {0x3F,0x3F,0x33,0x33,0x33,0x73,0xE3,0xC3,},
    {0xF0,0xF8,0xDC,0xCE,0xC7,0xC3,0x83,0x00,},
    {0x03,0x03,0xC3,0xE3,0x73,0x3B,0x1F,0x0F,},
    {0x3C,0x3E,0xE7,0xC3,0xC3,0xE7,0x3E,0x3C,},
    {0x3C,0x7E,0xE7,0xC3,0xC3,0xE7,0xFE,0xFC,},
    {0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,},
    {0xC0,0xE0,0x30,0x38,0x1C,0x0E,0x07,0x03,},
    {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,},
    {0x03,0x07,0x0E,0x1C,0x38,0x30,0xE0,0xC0,},
    {0x0C,0x0E,0x07,0x03,0x83,0xC7,0xFE,0x7C,},
    {0xFC,0xFE,0x07,0xE3,0xF3,0x37,0xFE,0xFC,},
    {0xF8,0xFC,0x0E,0x07,0x07,0x0E,0xFC,0xF8,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xF7,0x3E,0x1C,},
    {0xFC,0xFE,0x07,0x03,0x03,0x07,0x0E,0x0C,},
    {0x03,0xFF,0xFF,0x03,0x03,0x07,0xFE,0xFC,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xC3,0xC3,0x03,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xC3,0xC3,0x03,},
    {0xFC,0xFE,0x07,0x03,0xC3,0xC3,0xC7,0xC6,},
    {0xFF,0xFF,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF,},
    {0x00,0x03,0x03,0xFF,0xFF,0x03,0x03,0x00,},
    {0x00,0x00,0x00,0x03,0x03,0xFF,0xFF,0x03,},
    {0xFF,0xFF,0xE0,0x30,0x18,0x0C,0x07,0x03,},
    {0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0xFF,0xFE,0x1C,0x78,0x78,0x1C,0xFE,0xFF,},
    {0xFF,0xFF,0x38,0x70,0xE0,0xC0,0xFF,0xFF,},
    {0xFC,0xFE,0x07,0x03,0x03,0x07,0xFE,0xFC,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xE7,0x7E,0x3C,},
    {0xFC,0xFE,0x07,0x03,0x03,0x07,0xFE,0xFC,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xC3,0x7E,0x3C,},
    {0x3C,0x7E,0xE7,0xC3,0xC3,0xC7,0x8E,0x0C,},
    {0x03,0x03,0x03,0xFF,0xFF,0x03,0x03,0x03,},
    {0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,},
    {0xFF,0xFF,0x80,0x00,0x00,0x80,0xFF,0xFF,},
    {0xFF,0xFF,0x00,0x80,0x80,0x00,0xFF,0xFF,},
    {0x07,0x0F,0x3C,0xF0,0xF0,0x3C,0x0F,0x07,},
    {0x0F,0x1F,0x30,0xF0,0xF0,0x30,0x1F,0x0F,},
    {0x03,0x03,0x83,0xC3,0xE3,0x73,0x3F,0x1F,},
    {0x00,0xFF,0xFF,0x03,0x03,0x03,0x03,0x00,},
    {0x0C,0x0C,0x30,0x30,0xC0,0xC0,0x00,0x00,},
    {0x00,0x03,0x03,0x03,0x03,0xFF,0xFF,0x00,},
    {0x80,0xC0,0xE0,0x70,0x70,0xE0,0xC0,0x80,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x03,0x07,0x0E,0x1C,0x38,0x30,},
    {0x00,0x20,0x30,0x30,0x30,0x70,0xF0,0xE0,},
    {0xFF,0xFF,0x30,0x30,0x30,0x70,0xE0,0xC0,},
    {0xC0,0xE0,0x70,0x30,0x30,0x30,0x70,0x60,},
    {0xC0,0xE0,0x70,0x30,0x30,0x30,0xFF,0xFF,},
    {0xC0,0xE0,0x70,0x30,0x30,0x70,0xE0,0xC0,},
    {0x00,0xC0,0xC0,0xFC,0xFE,0xC7,0xC3,0x03,},
    {0xC0,0xE0,0x70,0x30,0x30,0x30,0xF0,0xF0,},
    {0xFF,0xFF,0x30,0x30,0x30,0x70,0xE0,0xC0,},
    {0x00,0x30,0x30,0xF3,0xF3,0x00,0x00,0x00,},
    {0x00,0x00,0x30,0x30,0xF3,0xF3,0x00,0x00,},
    {0xFF,0xFF,0x00,0x80,0xC0,0xE0,0x70,0x30,},
    {0x03,0x03,0xFF,0xFF,0x00,0x00,0x00,0x00,},
    {0xF0,0xE0,0x70,0xF0,0xF0,0x70,0xE0,0xC0,},
    {0xF0,0xF0,0x30,0x30,0x30,0x70,0xE0,0xC0,},
    {0xC0,0xE0,0x70,0x30,0x30,0x70,0xE0,0xC0,},
    {0xF0,0xF0,0x30,0x30,0x30,0x70,0xE0,0xC0,},
    {0xC0,0xE0,0x70,0x30,0x30,0x30,0xF0,0xF0,},
    {0xF0,0xF0,0x60,0x70,0x30,0x30,0x70,0x60,},
    {0xC0,0xE0,0x70,0x30,0x30,0x30,0x60,0x40,},
    {0x30,0x30,0xFF,0xFF,0x30,0x30,0x00,0x00,},
    {0xF0,0xF0,0x00,0x00,0x00,0x00,0xF0,0xF0,},
    {0xF0,0xF0,0x00,0x00,0x00,0x00,0xF0,0xF0,},
    {0xF0,0xF0,0x00,0x00,0x00,0x00,0xF0,0xF0,},
    {0x30,0x70,0xE0,0xC0,0xC0,0xE0,0x70,0x30,},
    {0xF0,0xF0,0x00,0x00,0x00,0x00,0xF0,0xF0,},
    {0x30,0x30,0x30,0x30,0x30,0xB0,0xF0,0x70,},
    {0x00,0x00,0xC0,0xDC,0xFE,0x37,0x03,0x03,},
    {0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,},
    {0x03,0x03,0x37,0xFE,0xDC,0xC0,0x00,0x00,},
    {0xC0,0xE0,0x70,0x60,0xC0,0xC0,0xE0,0x60,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0xFC,0xFC,0x03,0xE3,0xF3,0x33,0xFF,0xFF,},
    {0x00,0x00,0x00,0xC0,0x30,0x00,0xFF,0xFF,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0xC0,0xF0,0xF0,0x00,0x00,},
    {0x00,0x00,0x00,0xC0,0xF0,0xFC,0xFC,0x00,},
    {0x00,0x00,0x00,0xC0,0xF0,0xFC,0xFF,0xFF,},
    {0x00,0x03,0x03,0xFF,0xFF,0x03,0x03,0x00,},
    {0x00,0x00,0x00,0x03,0x03,0xFF,0xFF,0x03,},
    {0xFF,0xFF,0xE0,0x30,0x18,0x0C,0x07,0x03,},
    {0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0xFF,0xFE,0x1C,0x78,0x78,0x1C,0xFE,0xFF,},
    {0xFF,0xFF,0x38,0x70,0xE0,0xC0,0xFF,0xFF,},
    {0xFC,0xFE,0x07,0x03,0x03,0x07,0xFE,0xFC,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xE7,0x7E,0x3C,},
    {0xFC,0xFE,0x07,0x03,0x03,0x07,0xFE,0xFC,},
    {0xFF,0xFF,0xC3,0xC3,0xC3,0xC3,0x7E,0x3C,},
    {0x3C,0x7E,0xE7,0xC3,0xC3,0xC7,0x8E,0x0C,},
    {0x03,0x03,0x03,0xFF,0xFF,0x03,0x03,0x03,},
    {0xFF,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF,},
    {0xFF,0xFF,0x80,0x00,0x00,0x80,0xFF,0xFF,},
    {0xFF,0xFF,0x00,0x80,0x80,0x00,0xFF,0xFF,},
    {0x07,0x0F,0x3C,0xF0,0xF0,0x3C,0x0F,0x07,},
    {0x0F,0x1F,0x30,0xF0,0xF0,0x30,0x1F,0x0F,},
    {0x03,0x03,0x83,0xC3,0xE3,0x73,0x3F,0x1F,},
    {0x00,0xFF,0xFF,0x03,0x03,0x03,0x03,0x00,},
    {0x0C,0x0C,0x30,0x30,0xC0,0xC0,0x00,0x00,},
    {0x00,0x03,0x03,0x03,0x03,0xFF,0xFF,0x00,},
    {0x80,0xC0,0xE0,0x70,0x70,0xE0,0xC0,0x80,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
};

const unsigned char fontdata_bot[128][FONT_WIDTH_16] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x33,0x33,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x03,0x3F,0x3F,0x03,0x03,0x3F,0x3F,0x03,},
    {0x00,0x0C,0x0C,0x3F,0x3F,0x0C,0x07,0x03,},
    {0x06,0x07,0x03,0x01,0x3C,0x3C,0x3C,0x3C,},
    {0x0F,0x1F,0x30,0x31,0x3F,0x1F,0x3B,0x31,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x03,0x07,0x0E,0x1C,0x38,0x30,},
    {0x00,0x00,0x30,0x38,0x1C,0x0E,0x07,0x03,},
    {0x0C,0x0E,0x03,0x0F,0x0F,0x03,0x0E,0x0C,},
    {0x00,0x00,0x00,0x07,0x07,0x00,0x00,0x00,},
    {0x00,0x00,0xC0,0xE0,0x7C,0x3C,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,},
    {0x00,0x06,0x07,0x03,0x01,0x00,0x00,0x00,},
    {0x0F,0x1F,0x3B,0x31,0x30,0x38,0x1F,0x0F,},
    {0x00,0x30,0x30,0x3F,0x3F,0x30,0x30,0x00,},
    {0x3C,0x3E,0x37,0x33,0x31,0x30,0x30,0x30,},
    {0x18,0x38,0x30,0x30,0x30,0x39,0x1F,0x0F,},
    {0x01,0x01,0x01,0x01,0x01,0x3F,0x3F,0x01,},
    {0x0C,0x1C,0x38,0x30,0x30,0x38,0x1F,0x0F,},
    {0x0F,0x1F,0x39,0x30,0x30,0x39,0x1F,0x0F,},
    {0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,0x00,},
    {0x0F,0x1F,0x39,0x30,0x30,0x39,0x1F,0x0F,},
    {0x00,0x30,0x30,0x38,0x1C,0x0E,0x07,0x03,},
    {0x00,0x00,0x00,0x03,0x03,0x00,0x00,0x00,},
    {0x00,0x18,0x1C,0x0F,0x07,0x00,0x00,0x00,},
    {0x00,0x01,0x03,0x07,0x0E,0x1C,0x38,0x30,},
    {0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,},
    {0x30,0x38,0x1C,0x0E,0x07,0x03,0x01,0x00,},
    {0x00,0x00,0x00,0x37,0x37,0x01,0x00,0x00,},
    {0x0F,0x1F,0x38,0x31,0x33,0x33,0x33,0x03,},
    {0x3F,0x3F,0x03,0x03,0x03,0x03,0x3F,0x3F,},
    {0x3F,0x3F,0x30,0x30,0x30,0x3B,0x1F,0x0E,},
    {0x0F,0x1F,0x38,0x30,0x30,0x38,0x1C,0x0C,},
    {0x30,0x3F,0x3F,0x30,0x30,0x38,0x1F,0x0F,},
    {0x3F,0x3F,0x30,0x30,0x30,0x30,0x30,0x30,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x0F,0x1F,0x38,0x30,0x30,0x30,0x3F,0x3F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x3F,0x3F,},
    {0x00,0x30,0x30,0x3F,0x3F,0x30,0x30,0x00,},
    {0x0C,0x1C,0x38,0x30,0x38,0x1F,0x0F,0x00,},
    {0x3F,0x3F,0x01,0x03,0x06,0x0C,0x38,0x30,},
    {0x3F,0x3F,0x30,0x30,0x30,0x30,0x30,0x30,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x3F,0x3F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x01,0x3F,0x3F,},
    {0x0F,0x1F,0x38,0x30,0x30,0x38,0x1F,0x0F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x0F,0x1F,0x38,0x33,0x37,0x3E,0x3F,0x3F,},
    {0x3F,0x3F,0x01,0x03,0x07,0x0C,0x38,0x30,},
    {0x0C,0x1C,0x38,0x30,0x30,0x39,0x1F,0x0F,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x0F,0x1F,0x38,0x30,0x30,0x38,0x1F,0x0F,},
    {0x00,0x03,0x0F,0x3C,0x3C,0x0F,0x03,0x00,},
    {0x3F,0x1F,0x0E,0x07,0x07,0x0E,0x1F,0x3F,},
    {0x38,0x3C,0x07,0x03,0x03,0x07,0x3C,0x38,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x3E,0x3F,0x33,0x31,0x30,0x30,0x30,0x30,},
    {0x00,0x3F,0x3F,0x30,0x30,0x30,0x30,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,},
    {0x00,0x30,0x30,0x30,0x30,0x3F,0x3F,0x00,},
    {0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,},
    {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x0C,0x1E,0x3B,0x33,0x33,0x33,0x3F,0x3F,},
    {0x3F,0x3F,0x30,0x30,0x30,0x38,0x1F,0x0F,},
    {0x0F,0x1F,0x38,0x30,0x30,0x30,0x38,0x18,},
    {0x0F,0x1F,0x38,0x30,0x30,0x30,0x3F,0x3F,},
    {0x0F,0x1F,0x3B,0x33,0x33,0x33,0x33,0x03,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x03,0x07,0xCE,0xCC,0xCC,0xEE,0x7F,0x3F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x3F,0x3F,},
    {0x00,0x30,0x30,0x3F,0x3F,0x30,0x30,0x00,},
    {0xC0,0xC0,0xC0,0xE0,0x7F,0x3F,0x00,0x00,},
    {0x3F,0x3F,0x03,0x07,0x0F,0x1C,0x38,0x30,},
    {0x30,0x30,0x3F,0x3F,0x30,0x30,0x00,0x00,},
    {0x3F,0x3F,0x00,0x3F,0x3F,0x00,0x3F,0x3F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x3F,0x3F,},
    {0x0F,0x1F,0x38,0x30,0x30,0x38,0x1F,0x0F,},
    {0xFF,0xFF,0x0C,0x0C,0x0C,0x0E,0x07,0x03,},
    {0x03,0x07,0x0E,0x0C,0x0C,0x0C,0xFF,0xFF,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x08,0x19,0x33,0x33,0x33,0x3B,0x1E,0x0C,},
    {0x00,0x00,0x0F,0x1F,0x38,0x30,0x30,0x00,},
    {0x0F,0x1F,0x38,0x30,0x30,0x30,0x3F,0x3F,},
    {0x03,0x07,0x1E,0x38,0x38,0x1E,0x07,0x03,},
    {0x1F,0x3F,0x30,0x1F,0x1F,0x30,0x3F,0x1F,},
    {0x30,0x38,0x1C,0x0F,0x0F,0x1C,0x38,0x30,},
    {0x03,0x07,0xCE,0xCC,0xCC,0xEC,0x7F,0x3F,},
    {0x38,0x3C,0x36,0x33,0x33,0x31,0x30,0x30,},
    {0x00,0x00,0x00,0x0E,0x1F,0x3B,0x30,0x30,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x30,0x30,0x3B,0x1F,0x0E,0x00,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x0F,0x0F,0x30,0x31,0x33,0x33,0x33,0x03,},
    {0x30,0x2C,0x23,0x20,0x20,0x00,0x33,0x33,},
    {0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x30,0x3C,0x3C,0x00,0x00,0x00,0x00,0x00,},
    {0x30,0x3C,0x3F,0x3F,0x00,0x00,0x00,0x00,},
    {0x30,0x3C,0x3F,0x3F,0x3F,0x00,0x00,0x00,},
    {0x30,0x3C,0x3F,0x3F,0x3F,0x3F,0x00,0x00,},
    {0x30,0x3C,0x3F,0x3F,0x3F,0x3F,0x3F,0x00,},
    {0x30,0x3C,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,},
    {0x00,0x30,0x30,0x3F,0x3F,0x30,0x30,0x00,},
    {0x0C,0x1C,0x38,0x30,0x38,0x1F,0x0F,0x00,},
    {0x3F,0x3F,0x01,0x03,0x06,0x0C,0x38,0x30,},
    {0x3F,0x3F,0x30,0x30,0x30,0x30,0x30,0x30,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x3F,0x3F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x01,0x3F,0x3F,},
    {0x0F,0x1F,0x38,0x30,0x30,0x38,0x1F,0x0F,},
    {0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,},
    {0x0F,0x1F,0x38,0x33,0x37,0x3C,0x1F,0x0F,},
    {0x3F,0x3F,0x01,0x03,0x07,0x0C,0x38,0x30,},
    {0x0C,0x1C,0x38,0x30,0x30,0x39,0x1F,0x0F,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x0F,0x1F,0x38,0x30,0x30,0x38,0x1F,0x0F,},
    {0x00,0x03,0x0F,0x3C,0x3C,0x0F,0x03,0x00,},
    {0x3F,0x1F,0x0E,0x07,0x07,0x0E,0x1F,0x3F,},
    {0x38,0x3C,0x07,0x03,0x03,0x07,0x3C,0x38,},
    {0x00,0x00,0x00,0x3F,0x3F,0x00,0x00,0x00,},
    {0x3E,0x3F,0x33,0x31,0x30,0x30,0x30,0x30,},
    {0x00,0x3F,0x3F,0x30,0x30,0x30,0x30,0x00,},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,},
    {0x00,0x30,0x30,0x30,0x30,0x3F,0x3F,0x00,},
    {0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,},
    {0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,},
};
