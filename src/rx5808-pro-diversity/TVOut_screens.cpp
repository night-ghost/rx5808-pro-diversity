/*
 * TVOUT by Myles Metzel
 * Refactored and GUI reworked by Marko Hoepken
 * Universal version my Marko Hoepken
 * Diversity Receiver Mode and GUI improvements by Shea Ivey

The MIT License (MIT)

Copyright (c) 2015 Marko Hoepken

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "settings.h"

#ifdef TVOUT_SCREENS
#include "screens.h" // function headers
#include <Arduino.h>


#include <TVout.h>
#include <fontALL.h>

// Set you TV format (PAL = Europe = 50Hz, NTSC = INT = 60Hz)
//#define TV_FORMAT NTSC
#define TV_FORMAT PAL

#define TV_COLS 128
#define TV_ROWS 96
#define TV_Y_MAX TV_ROWS-1
#define TV_X_MAX TV_COLS-1
#define TV_SCANNER_OFFSET 14
#define SCANNER_BAR_SIZE 52
#define SCANNER_LIST_X_POS 54
#define SCANNER_LIST_Y_POS 16
#define SCANNER_MARKER_SIZE 1

#define MENU_Y_SIZE 15
#define TV_Y_GRID 14
#define TV_Y_OFFSET 3

#define MENU_START_X 10
#define MENU_START_Y 5

#define FONT_SMALL_X


TVout TV;

screens::screens() {
    last_channel = -1;
    last_rssi = 0;
}


static byte TV_BUFFER[TV_COLS*TV_ROWS/8]; // video memory 128 * 

char screens::begin(const char *call_sign) {
    // 0 if no error.
    // 1 if x is not divisable by 8.
    // 2 if y is to large (NTSC only cannot fill PAL vertical resolution by 8bit limit)
    // 4 if there is not enough memory for the frame buffer.
    return TV.begin(TV_FORMAT, TV_COLS, TV_ROWS, &TV_BUFFER);
}


void screens::reset() {
    TV.clear_screen();
    TV.select_font(font8x8);
}

void screens::flip() {
}

void screens::drawTitleBox(const char *title) {
    TV.draw_rect(0,0,TV_COLS-1,TV_ROWS-1,  WHITE);
//    TV.print_P(((126-strlen(title)*FONT_X)/2), 3,  title);
    TV.print_P(2, 3,  title);

    TV.draw_rect(0,0,TV_COLS-1, MENU_Y_SIZE-3,  WHITE,INVERT);
}

void screens::mainMenu(uint8_t menu_id) {
    reset(); // start from fresh screen.
    drawTitleBox(PSTR("MODE SELECTION"));

    TV.print_P(MENU_START_X, MENU_START_Y+1*MENU_Y_SIZE, PSTR("Auto Search"));
    TV.print_P(MENU_START_X, MENU_START_Y+2*MENU_Y_SIZE, PSTR("Band Scanner"));
    TV.print_P(MENU_START_X, MENU_START_Y+3*MENU_Y_SIZE, PSTR("Manual Mode"));
#ifdef USE_DIVERSITY
    TV.print_P(MENU_START_X, MENU_START_Y+4*MENU_Y_SIZE, PSTR("Diversity"));
#endif
    TV.print_P(MENU_START_X, MENU_START_Y+5*MENU_Y_SIZE, PSTR("Setup Menu"));

    TV.print_P(MENU_START_X, MENU_START_Y+6*MENU_Y_SIZE, PSTR("Exit menu"));
    // selection by inverted box

    TV.draw_rect(0,MENU_START_Y-2+(menu_id+1)*MENU_Y_SIZE,TV_COLS-1,MENU_Y_SIZE-3,  WHITE, INVERT);
}

void screens::seekMode(uint8_t state) {
    last_channel = -1;
    reset(); // start from fresh screen.
    if (state == STATE_MANUAL)
    {
        drawTitleBox("MANUAL MODE");
    }
    else if(state == STATE_SEEK)
    {
        drawTitleBox("AUTO SEEK MODE");
    }
    TV.draw_line(0,1*TV_Y_GRID,TV_X_MAX,1*TV_Y_GRID,WHITE);     TV.print_P(5,TV_Y_OFFSET  +1*TV_Y_GRID,  PSTR("BAND: "));
    TV.draw_line(0,2*TV_Y_GRID,TV_X_MAX,2*TV_Y_GRID,WHITE);     TV.print_P(5,TV_Y_OFFSET-1+2*TV_Y_GRID,  PSTR("1 2 3 4 5 6 7 8"));
    TV.draw_line(0,3*TV_Y_GRID,TV_X_MAX,3*TV_Y_GRID,WHITE);     TV.print_P(5,TV_Y_OFFSET  +3*TV_Y_GRID,  PSTR("FREQ:     GHz"));
    TV.draw_line(0,4*TV_Y_GRID,TV_X_MAX,4*TV_Y_GRID,WHITE);

    TV.select_font(font4x6);


    TV.draw_line(0,5*TV_Y_GRID-4,TV_X_MAX,5*TV_Y_GRID-4,WHITE); TV.print_P(5,TV_Y_OFFSET  +4*TV_Y_GRID,  PSTR("RSSI:"));

    // frame for tune graph
    TV.draw_rect(0,TV_ROWS - TV_SCANNER_OFFSET,TV_X_MAX,13,  WHITE); // lower frame

    TV.print_P(2,   (TV_ROWS - TV_SCANNER_OFFSET + 2), "5645");
    TV.print_P(57,  (TV_ROWS - TV_SCANNER_OFFSET + 2), "5800");
    TV.print_P(111, (TV_ROWS - TV_SCANNER_OFFSET + 2), "5945");

}

void screens::updateSeekMode(uint8_t state, uint8_t channelIndex, uint8_t channel, uint8_t rssi, uint16_t channelFrequency, uint8_t rssi_seek_threshold, bool locked) {
    // display refresh handler
    TV.select_font(font8x8);
    PGM_P str;
    
    if(channelIndex != last_channel) { // only updated on changes
        // show current used channel of bank
        if(       channelIndex > 31)  {
            str= PSTR("C/Race   ");
        } else if(channelIndex > 23) {
            str= PSTR("F/Airwave");
        } else if (channelIndex > 15) {
            str= PSTR("E        ");
        } else if (channelIndex > 7)  {
            str= PSTR("B        ");
        } else {
            str= PSTR("A        ");
        }
        TV.print_P(MENU_START_X+40,TV_Y_OFFSET+1*TV_Y_GRID,  str);
        // show channel inside band
        uint8_t active_channel = channelIndex%CHANNEL_BAND_SIZE; // get channel inside band

        TV.draw_rect(1 ,TV_Y_OFFSET-2+2*TV_Y_GRID, TV_COLS-3,12,  BLACK, BLACK); // mark current channel
        TV.print_P(5 ,  TV_Y_OFFSET-1+2*TV_Y_GRID,  PSTR("1 2 3 4 5 6 7 8"));
        // set new marker
        TV.draw_rect(active_channel*16+2 ,TV_Y_OFFSET-2+2*TV_Y_GRID,12,12,  WHITE, INVERT); // mark current channel

        // clear last square
        TV.draw_rect(1, (TV_ROWS - TV_SCANNER_OFFSET + FONT_Y), TV_COLS-3, SCANNER_MARKER_SIZE,  BLACK, BLACK);
        // draw next
        TV.draw_rect((channel * 3)+5, (TV_ROWS - TV_SCANNER_OFFSET + FONT_Y),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  WHITE, WHITE);

        // show frequence
        TV.print(MENU_START_X+40,TV_Y_OFFSET+3*TV_Y_GRID, channelFrequency);
    }
    
    // show signal strength
#define RSSI_BAR_SIZE 100
    uint8_t rssi_scaled=map(rssi, 1, 100, 1, RSSI_BAR_SIZE);
    // clear last bar
    TV.draw_rect(25, TV_Y_OFFSET+4*TV_Y_GRID, RSSI_BAR_SIZE,4 , BLACK, BLACK);
    //  draw new bar
    TV.draw_rect(25, TV_Y_OFFSET+4*TV_Y_GRID, rssi_scaled, 4 , WHITE, WHITE);
    // print bar for spectrum

#define SCANNER_BAR_MINI_SIZE 14
    rssi_scaled=map(rssi, 1, 100, 1, SCANNER_BAR_MINI_SIZE);
    // clear last bar
    TV.draw_rect((channel * 3)+4, (TV_ROWS - TV_SCANNER_OFFSET - SCANNER_BAR_MINI_SIZE), 2, SCANNER_BAR_MINI_SIZE , BLACK, BLACK);
    //  draw new bar
    TV.draw_rect((channel * 3)+4, (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled), 2, rssi_scaled , WHITE, WHITE);

    // handling for seek mode after screen and RSSI has been fully processed
    if(state == STATE_SEEK){  // SEEK MODE

        rssi_scaled=map(rssi_seek_threshold, 1, 100, 1, SCANNER_BAR_MINI_SIZE);

        TV.draw_rect(1,(TV_ROWS - TV_SCANNER_OFFSET - SCANNER_BAR_MINI_SIZE),2,SCANNER_BAR_MINI_SIZE-1,BLACK,BLACK);
        TV.draw_line(1,(TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled),3,(TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled), WHITE);
        TV.draw_rect(TV_X_MAX-2,(TV_ROWS - TV_SCANNER_OFFSET - SCANNER_BAR_MINI_SIZE),1,SCANNER_BAR_MINI_SIZE-1,BLACK,BLACK);
        TV.draw_line(TV_X_MAX-2,(TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled),TV_X_MAX,(TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled), WHITE);
        if(last_channel != channelIndex) {
            // fix title flicker
            TV.draw_rect(0,0,TV_COLS-1,14, WHITE,BLACK);
            if(locked) {
                TV.print_P(((TV_COLS-1-14*FONT_X)/2), TV_Y_OFFSET,  PSTR("AUTO MODE LOCK"));
            } else {
                TV.print_P(((TV_COLS-1-14*FONT_X)/2), TV_Y_OFFSET,  PSTR("AUTO MODE SEEK"));
            }
            TV.draw_rect(0,0,TV_COLS-1,14,  WHITE,INVERT);
        }
    }

    last_channel = channelIndex;
}

void screens::bandScanMode(uint8_t state) {
    reset(); // start from fresh screen.
    best_rssi = 0;
    
    if(state==STATE_SCAN)    {
        drawTitleBox("BAND SCANNER");
    } else {
        drawTitleBox("RSSI SETUP");
    }
    TV.select_font(font4x6);
    if(state==STATE_SCAN) {

        TV.draw_line(50,1*TV_Y_GRID,50, 1*TV_Y_GRID+9,WHITE);
        TV.print_P(2, SCANNER_LIST_Y_POS, PSTR("BEST:"));
    } else {
        TV.print_P(10, SCANNER_LIST_Y_POS, PSTR("RSSI Min:     RSSI Max:   "));
    }
    TV.draw_rect(0,1*TV_Y_GRID,TV_X_MAX,9,  WHITE); // list frame
    TV.draw_rect(0,TV_ROWS - TV_SCANNER_OFFSET,TV_X_MAX,13,  WHITE); // lower frame

    TV.select_font(font4x6);
    TV.print_P(2,   (TV_ROWS - TV_SCANNER_OFFSET + 2), PSTR("5645"));
    TV.print_P(57,  (TV_ROWS - TV_SCANNER_OFFSET + 2), PSTR("5800"));
    TV.print_P(111, (TV_ROWS - TV_SCANNER_OFFSET + 2), PSTR("5945"));
}

void screens::updateBandScanMode(bool in_setup, uint8_t channel, uint8_t rssi, uint8_t channelName, uint16_t channelFrequency, uint16_t rssi_setup_min_a, uint16_t rssi_setup_max_a) {
    // force tune on new scan start to get right RSSI value
    static uint8_t writePos=SCANNER_LIST_X_POS;
    // channel marker
//    if(channel != last_channel) { // only updated on changes
        // clear last square
        TV.draw_rect(1, (TV_ROWS - TV_SCANNER_OFFSET + 8),TV_COLS-3,SCANNER_MARKER_SIZE,  BLACK, BLACK);
        // draw next
        TV.draw_rect((channel * 3)+5, (TV_ROWS - TV_SCANNER_OFFSET + 8),SCANNER_MARKER_SIZE,SCANNER_MARKER_SIZE,  WHITE, -1);
//        TV.draw_lone(
//    }
    // print bar for spectrum

    uint8_t rssi_scaled=map(rssi, 1, 100, 5, SCANNER_BAR_SIZE);
    // clear last bar
    TV.draw_rect((channel * 3)+4, (TV_ROWS - TV_SCANNER_OFFSET - SCANNER_BAR_SIZE)-5, 2, SCANNER_BAR_SIZE+5 , BLACK, BLACK);
    //  draw new bar
    TV.draw_rect((channel * 3)+4, (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled), 2, rssi_scaled , WHITE, WHITE);
    // print channelname

    if(!in_setup) {
        if (rssi > RSSI_SEEK_TRESHOLD) {
            if(best_rssi < rssi) {
                best_rssi = rssi;
                TV.print(22, SCANNER_LIST_Y_POS, channelName, HEX);
                TV.print(32, SCANNER_LIST_Y_POS, channelFrequency);
            }
            else {
                if(writePos+10>TV_COLS-2)   { // keep writing on the screen
                    writePos=SCANNER_LIST_X_POS;
                }
                TV.draw_rect(writePos, SCANNER_LIST_Y_POS, 8, 6,  BLACK, BLACK);
                TV.print(writePos, SCANNER_LIST_Y_POS, channelName, HEX);
                writePos += 10;
            }
            TV.draw_rect((channel * 3) - 5, (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled) - 5, 8, 7,  BLACK, BLACK);
            TV.print((channel * 3) - 4, (TV_ROWS - TV_SCANNER_OFFSET - rssi_scaled) - 5, channelName, HEX);
        }
    } else {
            TV.print_P(50, SCANNER_LIST_Y_POS, PSTR("   "));
            TV.print(50, SCANNER_LIST_Y_POS, rssi_setup_min_a , DEC);

            TV.print_P(110, SCANNER_LIST_Y_POS, PSTR("   "));
            TV.print(110, SCANNER_LIST_Y_POS, rssi_setup_max_a , DEC);
    }

    last_channel = channel;
}

void screens::screenSaver(uint8_t channelName, uint16_t channelFrequency, const char *call_sign) {
    screenSaver(-1, channelName, channelFrequency, call_sign);
}

void screens::screenSaver(uint8_t diversity_mode, uint8_t channelName, uint16_t channelFrequency, const char *call_sign) {
 // not used in TVOut ... yet
/*    reset();
    TV.select_font(font8x8);
    TV.print(0,0,channelName, HEX);
    TV.select_font(font6x8);
    TV.print(70,0,CALL_SIGN);
    TV.select_font(font8x8);
    TV.print(70,28,channelFrequency);
    TV.select_font(font4x6);
#ifdef USE_DIVERSITY
    switch(diversity_mode) {
        case useReceiverAuto:
            TV.print_P(70,18,PSTR("AUTO"));
            break;
        case useReceiverA:
            TV.print_P(70,18,PSTR("ANTENNA A"));
            break;
        case useReceiverB:
            TV.print_P(70,18,PSTR("ANTENNA B"));
            break;
    }
    TV.draw_rect(0, TV_COLS-1-19, 7, 9, WHITE,BLACK);
    TV.print(1,95-18,"A");
    TV.draw_rect(0, TV_COLS-1-9, 7, 9, WHITE,BLACK);
    TV.print(1,95-8,"B");
#endif
*/
}

void screens::updateScreenSaver(uint8_t rssi) {
    updateScreenSaver(-1, rssi, -1, -1);
}
void screens::updateScreenSaver(char active_receiver, uint8_t rssi, uint8_t rssiA, uint8_t rssiB) {
// not used in TVOut ... yet
}

#ifdef USE_DIVERSITY
void screens::diversity(uint8_t diversity_mode) {
    reset();
    drawTitleBox("DIVERSITY");
    TV.print_P(MENU_START_X, MENU_START_Y   +1*MENU_Y_SIZE, PSTR("Auto"));
    TV.print_P(MENU_START_X, MENU_START_Y   +2*MENU_Y_SIZE, PSTR("Receiver A"));
    TV.print_P(MENU_START_X, MENU_START_Y   +3*MENU_Y_SIZE, PSTR("Receiver B"));
    // RSSI Strength
    
    TV.print_P(MENU_START_X, MENU_START_Y+1 +4*MENU_Y_SIZE, PSTR("A:")); TV.draw_line(0,MENU_START_Y-2 +4*MENU_Y_SIZE, TV_X_MAX, MENU_START_Y-2 +4*MENU_Y_SIZE, WHITE);
    TV.print_P(MENU_START_X, MENU_START_Y+1 +5*MENU_Y_SIZE, PSTR("B:")); TV.draw_line(0,MENU_START_Y-2 +5*MENU_Y_SIZE, TV_X_MAX, MENU_START_Y-2 +5*MENU_Y_SIZE, WHITE);

    TV.draw_rect(0,MENU_START_Y-2 +(diversity_mode+1)*MENU_Y_SIZE,TV_COLS-1,12,  WHITE, INVERT);
}

void screens::updateDiversity(char active_receiver, uint8_t rssiA, uint8_t rssiB){
#define RSSI_BAR_SIZE 100
    uint8_t rssi_scaled=map(rssiA, 1, 100, 1, RSSI_BAR_SIZE);
    // clear last bar
    TV.draw_rect(25+rssi_scaled, MENU_START_Y+1+4*MENU_Y_SIZE, RSSI_BAR_SIZE-rssi_scaled, 8 , BLACK, BLACK);
    //  draw new bar
    TV.draw_rect(25, MENU_START_Y+1+4*MENU_Y_SIZE, rssi_scaled, 8 , WHITE, (active_receiver==useReceiverA ? WHITE:BLACK));

    // read rssi B
    rssi_scaled=map(rssiB, 1, 100, 1, RSSI_BAR_SIZE);
    // clear last bar
    TV.draw_rect(25+rssi_scaled, MENU_START_Y+1+5*MENU_Y_SIZE, RSSI_BAR_SIZE-rssi_scaled, 8 , BLACK, BLACK);
    //  draw new bar
    TV.draw_rect(25, MENU_START_Y+1+5*MENU_Y_SIZE, rssi_scaled, 8 , WHITE, (active_receiver==useReceiverB ? WHITE:BLACK));

}
#endif


void screens::setupMenu(){
}
void screens::updateSetupMenu(uint8_t menu_id,bool settings_beeps,bool settings_orderby_channel, const char *call_sign, char editing){
    reset();
    drawTitleBox("SETUP MENU");

    static const PROGMEM ord[]="ORDER: ";

    TV.print_P(MENU_START_X, MENU_START_Y+1*MENU_Y_SIZE, ord);
    
    PGM_P str;
    
    if(settings_orderby_channel) {
	str=PSTR("CHANNEL  ");
    } else {
        str=PSTR("FREQUENCY");
    }
    TV.print_P(MENU_START_X+(strlen(ord)*FONT_X), MENU_START_Y+1*MENU_Y_SIZE, str);


    TV.print_P(MENU_START_X, MENU_START_Y+2*MENU_Y_SIZE, PSTR("BEEPS: "));
    if(settings_beeps) {
	str=PSTR("ON ");    
    } else {
        str=PSTR("OFF");
    }
    
    TV.print_P(MENU_START_X+(strlen(ord)*FONT_X), MENU_START_Y+2*MENU_Y_SIZE, str);


/* NO NEED FOR CALL SIGN IN TVOUT MODE
    static const PROGMEM sign[]="SIGN : ";

    TV.print_P(MENU_START_X, MENU_START_Y+3*MENU_Y_SIZE, PSTR("SIGN : "));
    if(editing>=0) {
        for(uint8_t i=0; i<10; i++) {
            TV.print(MENU_START_X-MENU_START_Y+((strlen(sign)+i)*8), MENU_START_Y+3*MENU_Y_SIZE, call_sign[i]);
        }

        TV.draw_rect(MENU_START_X-MENU_START_Y+((strlen(sign))*8),MENU_START_Y-2+(menu_id+1)*MENU_Y_SIZE,TV_X_MAX-(5+((strlen(sign))*8)),12,  WHITE, INVERT);
        TV.draw_rect(MENU_START_X-MENU_START_Y+((strlen(sign)+editing)*8),MENU_START_Y-2+(menu_id+1)*MENU_Y_SIZE,8,12,  BLACK, INVERT);
    }
    else {
        TV.print(MENU_START_X-MENU_START_Y+((strlen(sign)*8), MENU_START_Y+3*MENU_Y_SIZE, call_sign);
    }
*/

    static const PROGMEM batt[]="Batt:  "
    TV.print_P(MENU_START_X, 5+3*MENU_Y_SIZE, batt);
	extern float readBattery();
    float volt = readBattery() / 100;
      TV.print(MENU_START_X+(strlen(batt)*8), MENU_START_Y+3*MENU_Y_SIZE, volt); TV.print_P(PSTR(" (")); TV.print(n_cells); TV.print_P(PSTR("s)"));


    TV.print_P(MENU_START_X, MENU_START_Y+4*MENU_Y_SIZE, PSTR("CALIBRATE RSSI"));

    TV.print_P(MENU_START_X, MENU_START_Y+5*MENU_Y_SIZE, PSTR("SAVE & EXIT"));

    TV.draw_rect(0,MENU_START_Y-2+(menu_id+1)*MENU_Y_SIZE,TV_COLS-1,12,  WHITE, INVERT);
}

void screens::save(uint8_t mode, uint8_t channelIndex, uint16_t channelFrequency, const char *call_sign) {
    reset();
    drawTitleBox("SAVE SETTINGS");
    
    PGM_P str;
    switch (mode)   {
    case STATE_SCAN: // Band Scanner
        str= PSTR("Scanner");
        break;
    case STATE_MANUAL: // manual mode
        str= PSTR("Manual");
        break;
    case STATE_SEEK: // seek mode
        str=  PSTR("Search");
        break;
    }

    TV.print_P(MENU_START_X,   MENU_START_Y+1*MENU_Y_SIZE, PSTR("Mode:"));
    TV.print_P(MENU_START_X+40,MENU_START_Y+1*MENU_Y_SIZE,  str);
    
    
    // print band
    if(        channelIndex > 31)  {
        str= PSTR("C/Race   ");
    }  else if(channelIndex > 23) {
        str=  PSTR("F/Airwave");
    }  else if (channelIndex > 15) {
        str=  PSTR("E        ");
    }  else if (channelIndex > 7)  {
        str=  PSTR("B        ");
    }  else  {
        str=  PSTR("A        ");
    }
    TV.print_P(MENU_START_X,    MENU_START_Y+2*MENU_Y_SIZE, PSTR("Band:"));
    TV.print_P(MENU_START_X+40, MENU_START_Y+2*MENU_Y_SIZE,  str);
    
    TV.print_P(MENU_START_X,    MENU_START_Y+3*MENU_Y_SIZE, PSTR("Chan:"));
    uint8_t active_channel = channelIndex%CHANNEL_BAND_SIZE+1; // get channel inside band

    TV.print(MENU_START_X+40,   MENU_START_Y+3*MENU_Y_SIZE,active_channel,DEC);
    TV.print_P(MENU_START_X,    MENU_START_Y+4*MENU_Y_SIZE, PSTR("FREQ:     MHz"));
    TV.print(MENU_START_X+40,   MENU_START_Y+4*MENU_Y_SIZE, channelFrequency);
    TV.print_P(MENU_START_X,    MENU_START_Y+5*MENU_Y_SIZE, PSTR("--- SAVED ---"));
}

void screens::updateSave(const char * msg) {
    TV.select_font(font4x6);
    TV.print(((TV_COLS-1-strlen(msg)*FONT_SMALL_X)/2), MENU_START_Y+9 +5*MENU_Y_SIZE, msg);
}


#endif
