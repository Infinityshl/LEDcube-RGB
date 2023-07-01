#include <Arduino.h>
#include "arduinoFFT.h"
#include "IRremote.hpp"
#include "WiFi.h"

#define IR_RECV 35
#define MICRO_INPUT 34
#define CLK_ST 4
#define POS_0_DATA 26
#define POS_1_DATA 14
#define POS_2_DATA 13
#define POS_3_DATA 22
#define POS_4_DATA 19
#define POS_5_DATA 5
#define POS_0_CLK 25
#define POS_1_CLK 27
#define POS_2_CLK 12
#define POS_3_CLK 23
#define POS_4_CLK 21
#define POS_5_CLK 18
#define NEG_DATA 33
#define NEG_CLK 32

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define MODE1 0xBA45FF00
#define MODE2 0xB946FF00
#define MODE3 0xB847FF00
#define MODE4 0xBB44FF00
#define MODE5 0xBF40FF00
#define MODE6 0xBC43FF00
#define UP 0xE718FF00
#define DOWN 0xAD52FF00
#define LEFT 0xF708FF00
#define RIGHT 0xA55AFF00

arduinoFFT FFT = arduinoFFT();

const uint16_t samples = 64; 
const double samplingFrequency = 4000; 

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples]; 
double vImag[samples]; 

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

unsigned char neg, pos[6];
unsigned long long irctrl;
unsigned long currentime, tag, lastmode;
unsigned long hig[6];

const char *id = "RGBcube";
const char *psw = "rgbrgbrgb";
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;
int i, j;

unsigned long long judge() {
    if (irctrl == MODE1 || irctrl == MODE2 || irctrl == MODE3 || irctrl == MODE4 || irctrl == MODE5 || irctrl == MODE6) return irctrl;
    return 0ll;
}
void print() {
    digitalWrite(CLK_ST, 0);
    shiftOut(NEG_DATA, NEG_CLK, MSBFIRST, neg);
    shiftOut(POS_0_DATA, POS_0_CLK, MSBFIRST, pos[0]);
    shiftOut(POS_1_DATA, POS_1_CLK, MSBFIRST, pos[1]);
    shiftOut(POS_2_DATA, POS_2_CLK, MSBFIRST, pos[2]);
    shiftOut(POS_3_DATA, POS_3_CLK, MSBFIRST, pos[3]);
    shiftOut(POS_4_DATA, POS_4_CLK, MSBFIRST, pos[4]);
    shiftOut(POS_5_DATA, POS_5_CLK, MSBFIRST, pos[5]);
    digitalWrite(CLK_ST, 1);
}
void print_number(int i) {
    pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0x00;
    switch (i) {
        case 0:
            neg = 0xfd; pos[0] = 0xff; print();
            neg = 0xfb; pos[0] = 0xe1; print();
            neg = 0xf7; pos[0] = 0xe1; print();
            neg = 0xef; pos[0] = 0xff; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 1:
            neg = 0xfd; pos[0] = 0x00; print();
            neg = 0xfb; pos[0] = 0xff; print();
            neg = 0xf7; pos[0] = 0xff; print();
            neg = 0xef; pos[0] = 0x00; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 2:
            neg = 0xfd; pos[0] = 0xef; print();
            neg = 0xfb; pos[0] = 0xe9; print();
            neg = 0xf7; pos[0] = 0xe9; print();
            neg = 0xef; pos[0] = 0xf9; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 3:
            neg = 0xfd; pos[0] = 0xe9; print();
            neg = 0xfb; pos[0] = 0xe9; print();
            neg = 0xf7; pos[0] = 0xe9; print();
            neg = 0xef; pos[0] = 0xff; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 4:
            neg = 0xfd; pos[0] = 0xc4; print();
            neg = 0xfb; pos[0] = 0xcc; print();
            neg = 0xf7; pos[0] = 0xd4; print();
            neg = 0xef; pos[0] = 0xff; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 5:
            neg = 0xfd; pos[0] = 0xf9; print();
            neg = 0xfb; pos[0] = 0xe9; print();
            neg = 0xf7; pos[0] = 0xe9; print();
            neg = 0xef; pos[0] = 0xef; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 6:
            neg = 0xfd; pos[0] = 0xff; print();
            neg = 0xfb; pos[0] = 0xe9; print();
            neg = 0xf7; pos[0] = 0xe9; print();
            neg = 0xef; pos[0] = 0xef; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 7:
            neg = 0xfd; pos[0] = 0xe1; print();
            neg = 0xfb; pos[0] = 0xe2; print();
            neg = 0xf7; pos[0] = 0xe4; print();
            neg = 0xef; pos[0] = 0xf8; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 8:
            neg = 0xfd; pos[0] = 0xff; print();
            neg = 0xfb; pos[0] = 0xe9; print();
            neg = 0xf7; pos[0] = 0xe9; print();
            neg = 0xef; pos[0] = 0xff; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        case 9:
            neg = 0xfd; pos[0] = 0xf9; print();
            neg = 0xfb; pos[0] = 0xe9; print();
            neg = 0xf7; pos[0] = 0xe9; print();
            neg = 0xef; pos[0] = 0xff; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
        default:
            neg = 0xfd; pos[0] = 0x00; print();
            neg = 0xfb; pos[0] = 0xdb; print();
            neg = 0xf7; pos[0] = 0xdb; print();
            neg = 0xef; pos[0] = 0x00; print();
            neg = 0xdf; pos[0] = 0x00; print();
            break;
    }
}
void show_RGB() {
    for (i = 1; i < 6; ++i) pos[i] = 0x00;
    // R
    currentime = tag = millis();
    while (currentime - tag < 1000) {
        neg = 0xfd; pos[0] = 0xff; print();
        neg = 0xfb; pos[0] = 0xe8; print();
        neg = 0xf7; pos[0] = 0xec; print();
        neg = 0xef; pos[0] = 0xea; print();
        neg = 0xdf; pos[0] = 0xf9; print();
        currentime = millis();
    }
    pos[0] = 0x00; neg = 0x00; print();
    delay(500);
    // G 
    currentime = tag = millis();
    while (currentime - tag < 1000) {
        neg = 0xfe; pos[0] = 0xff; print();
        neg = 0xfd; pos[0] = 0xe1; print();
        neg = 0xfb; pos[0] = 0xe9; print();
        neg = 0xf7; pos[0] = 0xe9; print();
        neg = 0xef; pos[0] = 0xef; print();
        neg = 0xdf; pos[0] = 0xe8; print();
        currentime = millis();
    }
    pos[0] = 0x00; neg = 0x00; print();
    delay(500);
    // B
    currentime = tag = millis();
    while (currentime - tag < 1000) {
        neg = 0xfd; pos[0] = 0xff; print();
        neg = 0xfb; pos[0] = 0xe9; print();
        neg = 0xf7; pos[0] = 0xe9; print();
        neg = 0xef; pos[0] = 0xe9; print();
        neg = 0xdf; pos[0] = 0xd6; print();
        currentime = millis();
    }
    pos[0] = 0x00; neg = 0x00; print();
    delay(500);
}
void show_layer() {
    while (1) {
        for (i = 0; i < 6; ++i) { // left to right
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;
            neg = ~(1 << i);
            print();
            currentime = tag = millis();
            while (currentime - tag < 500) {
                currentime = millis();
            }
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = neg = 0x00; print();
            delay(100);
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
            
        }
        for (i = 5; i >= 0; --i) { // right to left
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;
            neg = ~(1 << i);
            print();
            currentime = tag = millis();
            while (currentime - tag < 500) {
                currentime = millis();
            }
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = neg = 0x00; print();
            delay(100);
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        for (i = 0; i < 6; ++i) { // front to back
            neg = 0xff;
            for (j = 0; j < 6; ++j) {
                if (i == j) pos[j] = 0xff;
                else pos[j] = 0x00;
            }
            currentime = tag = millis();
            while (currentime - tag < 500) {
                for (j = 0; j < 6; ++j) {
                    neg = ~(1 << j); print();
                }
                currentime = millis();
            }
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = neg = 0x00; print();
            delay(100);
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        for (i = 5; i >= 0; --i) { // back to front
            for (j = 0; j < 6; ++j) {
                if (i == j) pos[j] = 0xff;
                else pos[j] = 0x00;
            }
            currentime = tag = millis();
            while (currentime - tag < 500) {
                for (j = 0; j < 6; ++j) {
                    neg = ~(1 << j); print();
                }
                currentime = millis();
            }
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = neg = 0x00; print();
            delay(100);
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        for (i = 0; i < 6; ++i) { // down to up
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = (1 << i);
            currentime = tag = millis();
            while (currentime - tag < 500) {
                for (j = 0; j < 6; ++j) {neg = ~(1 << j); print();}
                currentime = millis();
            }
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = neg = 0x00; print();
            delay(100);
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        for (i = 5; i >= 0; --i) { // up to down
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = (1 << i);
            currentime = tag = millis();
            while (currentime - tag < 500) {
                for (j = 0; j < 6; ++j) {neg = ~(1 << j); print();}
                currentime = millis();
            }
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = neg = 0x00; print();
            delay(100);
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }

    }
}
void show_ball() {
    while (1) {
        currentime = tag = millis();
        while (currentime - tag < 500) {
            pos[0] = pos[1] = pos[4] = pos[5] = 0x00;
            pos[2] = pos[3] = 0xcc; neg = 0xf7;
            print();
            pos[0] = pos[1] = pos[4] = pos[5] = 0x00;
            pos[2] = pos[3] = 0xcc; neg = 0xfb;
            print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();
        while (currentime - tag < 500) {
            pos[0] = pos[5] = 0x00;
            pos[1] = pos[2] = pos[3] = pos[4] = 0xde;
            for (i = 1; i < 5; ++i) {
                neg = ~(1 << i);
                print();
            }
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();
        while (currentime - tag < 500) {
            pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;
            for (i = 0; i < 6; ++i) {
                neg = ~(1 << i); 
                print();
            }
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();
        while (currentime - tag < 500) {
            for (i = 0; i < 6; ++i) {
                if (i == 0) {pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;}
                else if (i == 1) {pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;}
                else if (i == 2) {pos[0] = pos[1] = pos[4] = pos[5] = 0xff; pos[2] = pos[3] = 0xf3;}
                else if (i == 3) {pos[0] = pos[1] = pos[4] = pos[5] = 0xff; pos[2] = pos[3] = 0xf3;}
                else if (i == 4) {pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;}
                else {pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;}
                neg = ~(1 << i);
                print();
                if (IrReceiver.decode()) {
                    irctrl = IrReceiver.decodedIRData.decodedRawData;
                    if (judge()) {
                        irctrl = judge();
                        IrReceiver.resume();
                        return ;
                    }
                    IrReceiver.resume();
                }
            }
            currentime = millis();
        }
        currentime = tag = millis();
        while (currentime - tag < 500) {
            for (i = 0; i < 6; ++i) {
                if (i == 0) {pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;}
                else if (i == 1) {pos[0] = pos[5] = 0xff; pos[1] = pos[2] = pos[3] = pos[4] = 0xe1;}
                else if (i == 2) {pos[0] = pos[5] = 0xff; pos[1] = pos[2] = pos[3] = pos[4] = 0xe1;}
                else if (i == 3) {pos[0] = pos[5] = 0xff; pos[1] = pos[2] = pos[3] = pos[4] = 0xe1;}
                else if (i == 4) {pos[0] = pos[5] = 0xff; pos[1] = pos[2] = pos[3] = pos[4] = 0xe1;}
                else {pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;}
                neg = ~(1 << i);
                print();
                if (IrReceiver.decode()) {
                    irctrl = IrReceiver.decodedIRData.decodedRawData;
                    if (judge()) {
                        irctrl = judge();
                        IrReceiver.resume();
                        return ;
                    }
                    IrReceiver.resume();
                }
            }
            currentime = millis();
        }
        currentime = tag = millis();
        while (currentime - tag < 500) {
            for (i = 0; i < 6; ++i) pos[i] = 0x00;
            neg = 0x00;
            print();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
            currentime = millis();
        }

    }
}
void sepa_number(int i, int &x, int &y) {
    y = i % 10;
    i /= 10;
    x = i % 10;
}
void show_time() {
    struct tm timeinfo;
    int x, y;
    while (1) {
        getLocalTime(&timeinfo);
        currentime = tag = millis();
        sepa_number(timeinfo.tm_hour, x, y);
        while (currentime - tag < 2000) {
            print_number(x);
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();
        while (currentime - tag < 2000) {
            print_number(y);
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();
        while (currentime - tag < 1000) {
            print_number(10);
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        sepa_number(timeinfo.tm_min, x, y);
        currentime = tag = millis();
        while (currentime - tag < 2000) {
            print_number(x);
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();
        while (currentime - tag < 2000) {
            print_number(y);
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        neg = pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
        delay(1000);
        if (IrReceiver.decode()) {
            irctrl = IrReceiver.decodedIRData.decodedRawData;
            if (judge()) {
                irctrl = judge();
                IrReceiver.resume();
                return ;
            }
            IrReceiver.resume();
        }
    }
}
void show_rotate() {
    while (1) {
        currentime = tag = millis();  // 1
        while (currentime - tag < 200) {
            neg = ~(1 << 0); pos[5] = 0xff; pos[5] = pos[1] = pos[2] = pos[3] = pos[4] = 0x00; print();
            neg = ~(1 << 1); pos[4] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[2] = 0xff; pos[0] = pos[1] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[1] = 0xff; pos[0] = pos[4] = pos[2] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 5); pos[0] = 0xff; pos[5] = pos[1] = pos[2] = pos[3] = pos[4] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 2
        while (currentime - tag < 200) {
            neg = ~(1 << 1); pos[4] = pos[5] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = 0x00; print();
            neg = ~(1 << 2); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[2] = 0xff; pos[0] = pos[1] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[1] = pos[0] = 0xff; pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 3
        while (currentime - tag < 200) {
            neg = ~(1 << 2); pos[3] = pos[4] = pos[5] = 0xff; pos[0] = pos[2] = pos[1] = 0x00; print();
            neg = ~(1 << 3); pos[0] = pos[1] = pos[2] = 0xff; pos[3] = pos[4] = pos[5] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 4
        while (currentime - tag < 200) {
            neg = ~(1 << 2); pos[3] = pos[4] = pos[5] = 0x00; pos[0] = pos[2] = pos[1] = 0xff; print();
            neg = ~(1 << 3); pos[0] = pos[1] = pos[2] = 0x00; pos[3] = pos[4] = pos[5] = 0xff; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 5
        while (currentime - tag < 200) {
            neg = ~(1 << 1); pos[0] = pos[1] = 0xff; pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[2] = 0xff; pos[0] = pos[1] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[4] = pos[5] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = 0x00; print(); 
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 6
        while (currentime - tag < 200) {
            neg = ~(1 << 0); pos[0] = 0xff; pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 1); pos[1] = 0xff; pos[0] = pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[2] = 0xff; pos[0] = pos[1] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[4] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 5); pos[5] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 7
        while (currentime - tag < 200) {
            neg = ~(1 << 0); pos[1] = 0xff; pos[0] = pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 1); pos[1] = 0xff; pos[0] = pos[2] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[2] = 0xff; pos[1] = pos[0] = pos[3] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[3] = 0xff; pos[1] = pos[2] = pos[0] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[4] = 0xff; pos[1] = pos[2] = pos[3] = pos[0] = pos[5] = 0x00; print();
            neg = ~(1 << 5); pos[4] = 0xff; pos[1] = pos[2] = pos[3] = pos[0] = pos[5] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 8
        while (currentime - tag < 200) {
            neg = ~(1 << 0); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 1); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[4] = 0xff; pos[1] = pos[2] = pos[0] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[4] = 0xff; pos[1] = pos[2] = pos[0] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 5); pos[4] = 0xff; pos[1] = pos[2] = pos[0] = pos[3] = pos[5] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 9
        while (currentime - tag < 200) {
            neg = ~(1 << 0); pos[4] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 1); pos[4] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[4] = 0xff; pos[0] = pos[1] = pos[2] = pos[3] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 5); pos[3] = 0xff; pos[0] = pos[1] = pos[2] = pos[4] = pos[5] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
        currentime = tag = millis();  // 10
        while (currentime - tag < 200) {
            neg = ~(1 << 0); pos[4] = 0xff; pos[0] = pos[2] = pos[3] = pos[1] = pos[5] = 0x00; print();
            neg = ~(1 << 1); pos[4] = 0xff; pos[0] = pos[2] = pos[3] = pos[1] = pos[5] = 0x00; print();
            neg = ~(1 << 2); pos[3] = 0xff; pos[1] = pos[0] = pos[2] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 3); pos[2] = 0xff; pos[1] = pos[3] = pos[0] = pos[4] = pos[5] = 0x00; print();
            neg = ~(1 << 4); pos[1] = 0xff; pos[4] = pos[2] = pos[3] = pos[0] = pos[5] = 0x00; print();
            neg = ~(1 << 5); pos[1] = 0xff; pos[4] = pos[2] = pos[3] = pos[0] = pos[5] = 0x00; print();
            currentime = millis();
            if (IrReceiver.decode()) {
                irctrl = IrReceiver.decodedIRData.decodedRawData;
                if (judge()) {
                    irctrl = judge();
                    IrReceiver.resume();
                    return ;
                }
                IrReceiver.resume();
            }
        }
    }
}
void draw(int idx, double val, int *flagg) {
    val = abs(val);
    if (val > 6) val = 6;
    unsigned long vall = (unsigned long)val;
    // Serial.print(vall); Serial.print(" ");
    if (hig[idx] < vall) hig[idx] = vall;
    if (*flagg) {
        if (hig[idx] > 0) hig[idx]--;
        if (idx == 5) *flagg = 0;
    }
}
void showlight() {
    neg = 0x00;
    for (i = 0; i < 6; ++i) {
        if (hig[i] == 0) pos[i] = 0x00;
        else if (hig[i] == 1) pos[i] = 0x01;
        else if (hig[i] == 2) pos[i] = 0x03;
        else if (hig[i] == 3) pos[i] = 0x07;
        else if (hig[i] == 4) pos[i] = 0x0f;
        else if (hig[i] == 5) pos[i] = 0x1f;
        else if (hig[i] == 6) pos[i] = 0x3f;
    }
    print();
}
void musicbeat() {
    int t, dt = 100, flag = 0;
    while (1) {
        microseconds = micros();
        for (i = 0; i < samples; ++i) {
            vReal[i] = analogRead(MICRO_INPUT);
            // Serial.print(vReal[i]), Serial.print(" ");
            vImag[i] = 0;
            while (micros() - microseconds < sampling_period_us) {;}
            microseconds += sampling_period_us;
        }
        // Serial.println("");
        FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
        FFT.ComplexToMagnitude(vReal, vImag, samples);
        draw(0, (vImag[8]+vImag[9]+vImag[10]) / 1000, &flag);
        draw(1, (vImag[15]+vImag[16]+vImag[17]) / 1000, &flag);
        draw(2, (vImag[22]+vImag[23]+vImag[24]) / 1000, &flag);
        draw(3, (vImag[29]+vImag[30]+vImag[31]) / 1000, &flag);
        draw(4, (vImag[36]+vImag[37]+vImag[38]) / 1000, &flag);
        draw(5, (vImag[43]+vImag[44]+vImag[45]) / 1000, &flag);
        showlight();
        // for (i = 0; i < 6; ++i) Serial.print(hig[i]), Serial.print(" ");
        // Serial.println("");
        if ((millis() - t) > dt) {
            flag = 1; t = millis();
        }
        if (IrReceiver.decode()) {
            irctrl = IrReceiver.decodedIRData.decodedRawData;
            if (judge()) {
                irctrl = judge();
                IrReceiver.resume();
                return ;
            }
            IrReceiver.resume();
        }

    }
}
void show_on() {
    for (i = 0; i < 6; ++i) pos[i] = 0xff;
    while (1) {
        neg = 0xfe; print();
        neg = 0xfd; print();
        neg = 0xfb; print();
        neg = 0xf7; print();
        neg = 0xef; print();
        neg = 0xdf; print();
        IrReceiver.decode();
        irctrl = IrReceiver.decodedIRData.decodedRawData;
        if (judge()) {
            irctrl = judge();
            IrReceiver.resume();
            return ;
        }
        IrReceiver.resume();
    }
}

void setup() {
    pinMode(POS_0_DATA, OUTPUT);
    pinMode(POS_1_DATA, OUTPUT);
    pinMode(POS_2_DATA, OUTPUT);
    pinMode(POS_3_DATA, OUTPUT);
    pinMode(POS_4_DATA, OUTPUT);
    pinMode(POS_5_DATA, OUTPUT);
    pinMode(POS_0_CLK, OUTPUT);
    pinMode(POS_1_CLK, OUTPUT);
    pinMode(POS_2_CLK, OUTPUT);
    pinMode(POS_3_CLK, OUTPUT);
    pinMode(POS_4_CLK, OUTPUT);
    pinMode(POS_5_CLK, OUTPUT);
    pinMode(NEG_DATA, OUTPUT);
    pinMode(NEG_CLK, OUTPUT);
    pinMode(CLK_ST, OUTPUT);
    // pinMode(IR_RECV, INPUT);
    pinMode(MICRO_INPUT, INPUT);
    Serial.begin(9600);
    IrReceiver.begin(IR_RECV);
    WiFi.begin(id, psw);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    sampling_period_us = round(1000000 * (1.0 / samplingFrequency));

    pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = 0xff;
    currentime = tag = millis();
    while (currentime - tag < 1500) {
        for (i = 0; i < 6; ++i) {
            neg = ~(1 << i);
            print();
        }
        currentime = millis();
    }
    for (i = 0; i < 6; ++i) pos[i] = 0x00;
    neg = 0x00;
    print();
    delay(1000);
    show_RGB();
    hig[0] = hig[1] = hig[2] = hig[3] = hig[4] = hig[5] = 0;
    lastmode = 1;

    IrReceiver.decode();
    irctrl = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();
}
void loop() {
    // 1 show on=
    // 2 show layer
    // 3 show ball
    // 4 show rotate
    // 5 show time
    // 6 show music
    if (irctrl == MODE1) {
        lastmode = 1;
    }
    else if (irctrl == MODE2) {
        lastmode = 2;
    }
    else if (irctrl == MODE3) {
        lastmode = 3;
    }
    else if (irctrl == MODE4) {
        lastmode = 4;
    }
    else if (irctrl == MODE5) {
        lastmode = 5;
    }
    else if (irctrl == MODE6) {
        lastmode = 6;
    }
    else lastmode = 1;

    switch (lastmode) {
        case 1:
            show_on();
            break;
        case 2:
            show_layer();
            break;
        case 3:
            show_ball();
            break;
        case 4:
            show_rotate();
            break;
        case 5:
            show_time();
            break;
        case 6:
            musicbeat();
            break;
    }

    /*
    IrReceiver.decode();
    irctrl = IrReceiver.decodedIRData.decodedRawData;
    if (judge()) {
        irctrl = judge();
        return ;
    }
    IrReceiver.resume();
    */
}