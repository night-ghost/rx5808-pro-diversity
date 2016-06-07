/*
 * SPI driver based on fs_skyrf_58g-main.c Written by Simon Chambers
 * TVOUT by Myles Metzel
 * Scanner by Johan Hermen
 * Inital 2 Button version by Peter (pete1990)
 * Refactored and GUI reworked by Marko Hoepken
 * Universal version my Marko Hoepken
 * Diversity Receiver Mode and GUI improvements by Shea Ivey
 * OLED Version by Shea Ivey
 * Seperating display concerns by Shea Ivey

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

//#include "pollserial.h" - flickering screen 
#include "SingleSerial.h"
#include <avr/pgmspace.h>

#include "Arduino.h"


#include "settings.h"

// uncomment depending on the display you are using.
// this is an issue with the arduino preprocessor
#ifdef TVOUT_SCREENS
    #include <TVout.h>
    #include <fontALL.h>
#endif
#ifdef OLED_128x64_ADAFRUIT_SCREENS
//    #include <Adafruit_SSD1306.h>
//    #include <Adafruit_GFX.h>
//    #include <Wire.h>
//    #include <SPI.h>
#endif
#ifdef OLED_128x64_U8G_SCREENS
//    #include <U8glib.h>
#endif

#include "screens.h"
#include "compat.h"

extern TVout TV;



screens drawScreen;

inline void EEPROM_write(int n, byte b) { eeprom_write_byte((byte *)n,b); }
inline byte EEPROM_read(int n) { return eeprom_read_byte((byte *)n); }

void EEPROM_write_word(int n, int b) {
    EEPROM_write(n,(b & 0xff));
    EEPROM_write(n+1,(b >> 8));
}

uint16_t EEPROM_read_word(int n) { 
    return eeprom_read_byte((byte *)n) | (eeprom_read_byte((byte *)(n+1))  * 256); 
}

// Channels to sent to the SPI registers
const uint16_t channelTable[] PROGMEM = {
  // Channel 1 - 8
  0x2A05,    0x299B,    0x2991,    0x2987,    0x291D,    0x2913,    0x2909,    0x289F,    // Band A
  0x2903,    0x290C,    0x2916,    0x291F,    0x2989,    0x2992,    0x299C,    0x2A05,    // Band B
  0x2895,    0x288B,    0x2881,    0x2817,    0x2A0F,    0x2A19,    0x2A83,    0x2A8D,    // Band E
  0x2906,    0x2910,    0x291A,    0x2984,    0x298E,    0x2998,    0x2A02,    0x2A0C,    // Band F / Airwave
  0x281D,    0x288F,    0x2902,    0x2914,    0x2978,    0x2999,    0x2A0C,    0x2A1E     // Band C / Immersion Raceband
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] PROGMEM = {
  // Channel 1 - 8
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F / Airwave
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Band C / Immersion Raceband
};

// do coding as simple hex value to save memory.
const uint8_t channelNames[] PROGMEM = {
  0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, // Band A
  0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, // Band B
  0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, // Band E
  0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, // Band F / Airwave
  0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8  // Band C / Immersion Raceband
};

// All Channels of the above List ordered by Mhz
const uint8_t channelList[] PROGMEM = {
  19, 18, 32, 17, 33, 16, 7, 34, 8, 24, 6, 9, 25, 5, 35, 10, 26, 4, 11, 27, 3, 36, 12, 28, 2, 13, 29, 37, 1, 14, 30, 0, 15, 31, 38, 20, 21, 39, 22, 23
};

char channel = 0;
uint8_t channelIndex = 0;
uint8_t rssi = 0;
uint8_t rssi_scaled = 0;
uint8_t active_receiver = useReceiverA;
uint8_t diversity_mode = useReceiverAuto;
char diversity_check_count = 0;

uint8_t is_menu_mode =0;
uint8_t menu_timer=0;
uint8_t btn_pressed=0;

uint8_t rssi_seek_threshold = RSSI_SEEK_TRESHOLD;
uint8_t hight = 0;
uint8_t state = START_STATE;
uint8_t state_last_used= 1; //START_STATE + 1;
uint8_t last_state= START_STATE+1; // force screen draw
uint8_t writePos = 0;
uint8_t switch_count = 0;
uint8_t man_channel = 0;
uint8_t last_channel_index = 0;
uint8_t force_seek=0;
uint8_t seek_direction=1;
unsigned long time_of_tune = 0;        // will store last time when tuner was changed
unsigned long time_screen_saver = 0;
uint8_t last_active_channel=0;
uint8_t seek_found=0;
uint8_t scan_start=0;
uint8_t first_tune=1;
uint8_t force_menu_redraw=0;
uint16_t rssi_best=0; // used for band scaner
uint16_t rssi_min_a=0;
uint16_t rssi_max_a=0;
uint16_t rssi_setup_min_a=0;
uint16_t rssi_setup_max_a=0;
uint16_t rssi_min_b=0;
uint16_t rssi_max_b=0;
uint16_t rssi_setup_min_b=0;
uint16_t rssi_setup_max_b=0;
uint16_t rssi_seek_found=0;
uint16_t rssi_setup_run=0;

uint16_t freq;
uint16_t last_channel_freq;
uint16_t freq_best;


char call_sign[10];
bool settings_beeps = true;
bool settings_orderby_channel = true;

byte n_cells=0;

#define BTN_DOWN digitalRead(buttonDown)
#define BTN_MODE digitalRead(buttonMode)
#define BTN_UP digitalRead(buttonUp  )

// SETUP ----------------------------------------------------------------------------
void setup()
{
// 	TV.set_hbi_hook(pserial.begin(57600));
#ifdef DEBUG
//    Serial.begin(57600);
//    Serial.println(F("START:"));
#endif

    // IO INIT
    // initialize digital pin 13 LED as an output.
    pinMode(LED_PIN, OUTPUT); // status pin for TV mode errors
    digitalWrite(LED_PIN, HIGH);
    // buzzer
    pinMode(PIN_buzzer, OUTPUT); // Feedback buzzer (active buzzer, not passive piezo)
    digitalWrite(PIN_buzzer, LOW);
    
    // minimum control pins
    pinMode(buttonUp, INPUT_PULLUP);
    pinMode(buttonMode, INPUT_PULLUP);
    // optional control
    pinMode(buttonDown, INPUT_PULLUP);
#ifdef buttonSave
    pinMode(buttonSave, INPUT_PULLUP);
#endif

    //Receiver Setup
    pinMode(SWITCH_0_PIN,OUTPUT);
    pinMode(SWITCH_1_PIN,OUTPUT);

    pinMode(PULLDN_PIN,OUTPUT); // resistive pull-down for RSSI inputs
    digitalWrite(PULLDN_PIN,0);

    setReceiver(useReceiverA);
    pinMode(VIDEO_ON_PIN, OUTPUT);
    digitalWrite(VIDEO_ON_PIN, LOW);


    // SPI pins for RX control
    pinMode(slaveSelectPin, OUTPUT);
    pinMode(spiDataPin, OUTPUT);
    pinMode(spiClockPin, OUTPUT);



    // use values only of EEprom is not 255 = unsaved
    uint8_t eeprom_check = EEPROM_read(EEPROM_ADR_STATE);
    if(eeprom_check == 255) {// unused
        // save 8 bit
        EEPROM_write(EEPROM_ADR_STATE, START_STATE);
        
        EEPROM_write(EEPROM_ADR_TUNE,CHANNEL_MIN_INDEX);
        EEPROM_write(EEPROM_ADR_BEEP,settings_beeps);
        EEPROM_write(EEPROM_ADR_ORDERBY, settings_orderby_channel);
        EEPROM_write(EEPROM_ADR_DIVERSITY, diversity_mode);

        EEPROM_write_word(EEPROM_ADR_RSSI_MIN_A, RSSI_MIN_VAL);
        EEPROM_write_word(EEPROM_ADR_RSSI_MAX_A, RSSI_MAX_VAL);
        EEPROM_write_word(EEPROM_ADR_RSSI_MIN_B, RSSI_MIN_VAL);
        EEPROM_write_word(EEPROM_ADR_RSSI_MAX_B, RSSI_MAX_VAL);
        
        EEPROM_write_word(EEPROM_ADR_FREQ, 0); // by channel

        // save default call sign
        strcpy(call_sign, CALL_SIGN); // load callsign
        for(uint8_t i = 0;i<sizeof(call_sign);i++) {
            EEPROM_write(EEPROM_ADR_CALLSIGN+i,call_sign[i]);
        }

    }

    // read last setting from eeprom
    state                    = EEPROM_read(EEPROM_ADR_STATE);
    channelIndex             = EEPROM_read(EEPROM_ADR_TUNE);
    settings_beeps           = EEPROM_read(EEPROM_ADR_BEEP);
    settings_orderby_channel = EEPROM_read(EEPROM_ADR_ORDERBY);
    diversity_mode 	     = EEPROM_read(EEPROM_ADR_DIVERSITY);
    rssi_min_a  = EEPROM_read_word(EEPROM_ADR_RSSI_MIN_A);
    rssi_max_a  = EEPROM_read_word(EEPROM_ADR_RSSI_MAX_A);    
    rssi_min_b  = EEPROM_read_word(EEPROM_ADR_RSSI_MIN_B);
    rssi_max_b  = EEPROM_read_word(EEPROM_ADR_RSSI_MAX_B);
    
    freq        = EEPROM_read_word(EEPROM_ADR_FREQ);

    // load saved call sign
    for(uint8_t i = 0;i<sizeof(call_sign);i++) {
        call_sign[i] = EEPROM_read(EEPROM_ADR_CALLSIGN+i);
    }

    force_menu_redraw=1;

    // tune to first channel

    // Setup Done - LED ON
    digitalWrite(LED_PIN, HIGH);

    // Init Display
    if (drawScreen.begin(call_sign) > 0) {
        // on Error flicker LED
        while (true) { // stay in ERROR for ever
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
    // rotate the display output 180 degrees.
    // drawScreen.flip(); // OLED only!

}

// LOOP ----------------------------------------------------------------------------
void loop() {

// TODO: заменить залипающее меню на автомат состояний

    /*******************/
    /*   Mode Select   */
    /*******************/
    uint8_t in_menu;
    uint8_t in_menu_time_out;

//    readBattery();
    CalcNCells();

    if (BTN_MODE == LOW) { // key pressed ?
        time_screen_saver=0;

        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE/2); // debounce
        beep(50); // beep & debounce
        delay(KEY_DEBOUNCE/2); // debounce

	setMenu(true);

        uint8_t press_time=0;
        // on entry wait for release
        while(BTN_MODE == LOW && press_time < 10) {
            delay(100);
            press_time++;
        }
#define MAX_MENU 4
#define MENU_Y_SIZE 15

        char menu_id=state_last_used-1;
        
        // Show Mode Screen
        if(state==STATE_SEEK_FOUND) {
            state=STATE_SEEK;
        }
        in_menu=1;
        in_menu_time_out=50; // 50x 100ms = 5 seconds
/*
        Enter Mode menu
        Show current mode
        Change mode by MODE key
        Any Mode will refresh screen
        If not MODE changes in 5 seconds, it uses last used mode
*/
        
        do {
            if(press_time >= 10) { // if menu held for 1 second invoke quick save.
                // user held the mode button and wants to quick save.
                in_menu=0; // EXIT
                state = STATE_SAVE;
                break;
            }
            
            switch (menu_id) {
            case 0: // auto search
                state=STATE_SEEK;
                force_seek=1;
                seek_found=0;
            break;
            case 1: // Band Scanner
                state=STATE_SCAN;
                scan_start=1;
            break;
            case 2: // manual mode
                state=STATE_MANUAL;
            break;
            case 3: // Diversity
                state=STATE_DIVERSITY;
            break;

            case 4: // Setup Menu
                state=STATE_SETUP_MENU;
                break;
    
            case 5: // exit menu
                state=STATE_SCREEN_SAVER;
                break;
            
            } // end switch

            // draw mode select screen
            drawScreen.mainMenu(menu_id);

            while(BTN_MODE == LOW || BTN_UP == LOW || BTN_DOWN == LOW) {
                // wait for MODE release
                in_menu_time_out=50;
            }
            
            while(--in_menu_time_out && ((BTN_MODE == HIGH) && (BTN_UP == HIGH) && (BTN_DOWN == HIGH))) { // wait for next key press or time out
                delay(100); // timeout delay = 0.1s * 50 = 5s
            }
            if(in_menu_time_out==0 || BTN_MODE == LOW) { // timeout
                if(BTN_MODE != LOW) {
                
                    state=state_last_used; // exit to last state on timeout.
                    state=STATE_SCREEN_SAVER;
                }
                in_menu=0; // EXIT
//                beep(KEY_DEBOUNCE/2); // beep & debounce
//                delay(50); // debounce
//                beep(KEY_DEBOUNCE/2); // beep & debounce
                delay(50); // debounce
                
            } else { // no timeout, must be keypressed up or down
                /*********************/
                /*   Menu handler   */
                /*********************/
                if(     BTN_UP == LOW) {
                    menu_id++;
                }
                else if(BTN_DOWN == LOW) {
                    menu_id--;
                }

                if (menu_id > MAX_MENU) {
                    menu_id = 0; // next state
                }
                if(menu_id < 0) {
                    menu_id = MAX_MENU;
                }
                in_menu_time_out=50;
                beep(50); // beep & debounce
                delay(KEY_DEBOUNCE); // debounce
                while(BTN_MODE == LOW); // wait for release
            }
        } while(in_menu);
        
        last_state=255; // force redraw of current screen
        switch_count = 0;
        btn_pressed =1;
    } else { // key pressed - no
    // reset debounce
        switch_count = 0;
    }

#ifdef buttonSave
    // hardware save buttom support (if no display is used)
    if(digitalRead(buttonSave) == LOW) {
        state=STATE_SAVE;
        setMenu(true);
    }
#endif
    /***************************************/
    /*   Draw screen if mode has changed   */
    /***************************************/
    if(force_menu_redraw || state != last_state) {
        force_menu_redraw=0;


        setMenu(state!=STATE_SCREEN_SAVER || btn_pressed);
        btn_pressed = 0;


        /************************/
        /*   Main screen draw   */
        /************************/
        // changed state, clear and draw new screen

        // simple menu
        switch (state) {
            case STATE_SCAN: // Band Scanner
                state_last_used=state;
                // no break!

            case STATE_RSSI_SETUP: // RSSI setup
                // draw selected
                if(state==STATE_RSSI_SETUP) {
                    // prepare new setup
                    rssi_min_a=0;
                    rssi_max_a=400; // set to max range
                    rssi_setup_min_a=400;
                    rssi_setup_max_a=0;
                    rssi_min_b=0;
                    rssi_max_b=400; // set to max range
                    rssi_setup_min_b=400;
                    rssi_setup_max_b=0;
                    rssi_setup_run=RSSI_SETUP_RUN;
                }

                // trigger new scan from begin
                channel=CHANNEL_MIN;
                channelIndex = index_from_channel(channel); //pgm_read_byte(channelList + channel);
                rssi_best=0;
                scan_start=1;

                drawScreen.bandScanMode(state);
            break;

            case STATE_SEEK: // seek mode
                rssi_seek_threshold = RSSI_SEEK_TRESHOLD;
                rssi_best=0;
                force_seek=1;
                // no break!

            case STATE_MANUAL: // manual mode
                if (state == STATE_MANUAL) {
                    time_screen_saver=millis();
                } else if(state == STATE_SEEK) {
                    time_screen_saver=0; // dont show screen saver until we found a channel.
                }
                drawScreen.seekMode(state);

                // return user to their saved channel after bandscan
                if(state_last_used == STATE_SCAN || last_state == STATE_RSSI_SETUP) {
                    channelIndex=EEPROM_read(EEPROM_ADR_TUNE);
                }
                state_last_used=state;
            break;
            
            case STATE_DIVERSITY: {
                // simple menu
                char old_diversity_mode = diversity_mode;
                char menu_id=diversity_mode;
                uint8_t in_menu=1;
                time_screen_saver=millis(); // enable screen saver
                do{
                    diversity_mode = menu_id;
                    drawScreen.diversity(diversity_mode);
                    in_menu_time_out=50;
                    do {
                        readRSSI();
                        drawScreen.updateDiversity(active_receiver, readRSSI(useReceiverA), readRSSI(useReceiverB));
                        delay(100);
                    } while(--in_menu_time_out && (BTN_MODE == HIGH) && (BTN_UP == HIGH) && (BTN_DOWN == HIGH)); // wait for next button or time out
    
		    if(!in_menu_time_out){
			diversity_mode = old_diversity_mode; // restore last
			in_menu = 0; // exit menu by timeout
		    }
			    
    
                    if(BTN_MODE == LOW) {        
                        in_menu = 0; // exit menu on button
                    }
                    else if(BTN_UP == LOW) {
                        menu_id++;
                    }
                    else if(BTN_DOWN == LOW) {
                        menu_id--;
                    }
    
                    if(menu_id > useReceiverB) {
                        menu_id = 0;
                    }
                    if(menu_id < 0) {
                        menu_id = useReceiverB;
                    }
                    beep(50); // beep & debounce
                    delay(KEY_DEBOUNCE); // debounce
                } while(in_menu);
    
                state=state_last_used;

            } break;
            
            case STATE_SETUP_MENU:

            break;

            case STATE_SAVE: {
                EEPROM_write(EEPROM_ADR_TUNE,channelIndex);
                EEPROM_write(EEPROM_ADR_STATE,state_last_used);
                EEPROM_write(EEPROM_ADR_BEEP,settings_beeps);
                EEPROM_write(EEPROM_ADR_ORDERBY,settings_orderby_channel);

                // save call sign
                for(uint8_t i = 0;i<sizeof(call_sign);i++) {
                    EEPROM_write(EEPROM_ADR_CALLSIGN+i,call_sign[i]);
                }
                EEPROM_write(EEPROM_ADR_DIVERSITY,diversity_mode);
                
                uint16_t f= freq ? freq : pgm_read_word(channelFreqTable + channelIndex);
                drawScreen.save(state_last_used, channelIndex, f, call_sign);
                for (uint8_t loop=0;loop<5;loop++) {
                    beep(100); // beep
                    delay(100);
                }
                delay(1000);
                state=state_last_used; // return to saved function
                force_menu_redraw=1; // we change the state twice, must force redraw of menu

            } break;
            
            case STATE_SCREEN_SAVER:
                drawScreen.screenSaver(diversity_mode, pgm_read_byte(channelNames + channelIndex), pgm_read_word(channelFreqTable + channelIndex), call_sign);
//                do{
                    //delay(10); // timeout delay
                    drawScreen.updateScreenSaver(active_receiver, readRSSI(), readRSSI(useReceiverA), readRSSI(useReceiverB));

//                }
//                while((BTN_MODE == HIGH) && (BTN_UP == HIGH) && (BTN_DOWN == HIGH)); // wait for next button press
//                state=state_last_used;
//                time_screen_saver=0;
                return;
            break;
        } // end switch

        last_state=state;
    }
    /*************************************/
    /*   Processing depending of state   */
    /*************************************/

    /*****************************************/
    /*   Processing MANUAL MODE / SEEK MODE  */
    /*****************************************/
    if(state == STATE_MANUAL || state == STATE_SEEK)  {
        channel=channel_from_index(channelIndex); // get 0...40 index depending of current channel
        
        if(state == STATE_MANUAL) { // MANUAL MODE
            // handling of keys
            if( BTN_UP == LOW) {       // channel UP
                time_screen_saver=millis();
                beep(50); // beep & debounce
                delay(KEY_DEBOUNCE); // debounce
                channelIndex++;
                channel++;
                channel > CHANNEL_MAX ? channel = CHANNEL_MIN : false;
                if (channelIndex > CHANNEL_MAX_INDEX) {
                    channelIndex = CHANNEL_MIN_INDEX;
                }
            }

            if( BTN_DOWN == LOW) { // channel DOWN
                time_screen_saver=millis();
                beep(50); // beep & debounce
                delay(KEY_DEBOUNCE); // debounce
                channelIndex--;
                channel--;
                channel < CHANNEL_MIN ? channel = CHANNEL_MAX : false;
                if (channelIndex > CHANNEL_MAX_INDEX) { // negative overflow
                    channelIndex = CHANNEL_MAX_INDEX;
                }
            }

            if(!settings_orderby_channel) { // order by frequency
                //channelIndex = pgm_read_byte(channelList + channel);
                channelIndex = index_from_channel(channel);
            }

        }
        // show signal strength
        wait_rssi_ready();
        rssi = readRSSI();
        rssi_best = (rssi > rssi_best) ? rssi : rssi_best;


        // handling for seek mode after screen and RSSI has been fully processed
        if(state == STATE_SEEK) { // SEEK MODE

            // recalculate rssi_seek_threshold
            ((int)((float)rssi_best * RSSI_SEEK_TRESHOLD/100.0) > rssi_seek_threshold) ? 
        	(rssi_seek_threshold = (int)((float)rssi_best * RSSI_SEEK_TRESHOLD/100.0)) : 
        	false;

            if(!seek_found) {// search if not found
                if ((!force_seek) && (rssi > rssi_seek_threshold)) { // check for found channel
                    seek_found=1;
                    time_screen_saver=millis();
                    // beep twice as notice of lock
                    beep(100);
                    delay(100);
                    beep(100);
                } else { // seeking itself
                    force_seek=0;
                    // next channel
                    channel+=seek_direction;
                    if (channel > CHANNEL_MAX) {
                        // calculate next pass new seek threshold
                        rssi_seek_threshold = (int)((float)rssi_best * (float)(RSSI_SEEK_TRESHOLD/100.0));
                        channel=CHANNEL_MIN;
                        rssi_best = 0;
                    }
                    else if(channel < CHANNEL_MIN) {
                        // calculate next pass new seek threshold
                        rssi_seek_threshold = (int)((float)rssi_best * (float)(RSSI_SEEK_TRESHOLD/100.0));
                        channel=CHANNEL_MAX;
                        rssi_best = 0;
                    }
                    rssi_seek_threshold = rssi_seek_threshold < 5 ? 5 : rssi_seek_threshold; // make sure we are not stopping on everyting
                    //channelIndex = pgm_read_byte(channelList + channel);
                    channelIndex = index_from_channel(channel);
                }
            } else { // seek was successful
                if (BTN_UP == LOW || BTN_DOWN == LOW) { // restart seek if key pressed
                    if(BTN_UP == LOW) {
                        seek_direction = 1;
                    } else {
                        seek_direction = -1;
                    }
                    beep(50);
                    delay(KEY_DEBOUNCE); // debounce
                    force_seek=1;
                    seek_found=0;
                    time_screen_saver=0;
                }
            }
        }
//#ifndef TVOUT_SCREENS
        if(time_screen_saver!=0 && time_screen_saver+5000 < millis()) {
            state = STATE_SCREEN_SAVER;
        }
//#endif
        drawScreen.updateSeekMode(state, channelIndex, channel, rssi, pgm_read_word(channelFreqTable + channelIndex), rssi_seek_threshold, seek_found);
    }
    /****************************/
    /*   Processing SCAN MODE   */
    /****************************/
    else if (state == STATE_SCAN || state == STATE_RSSI_SETUP) {
        // force tune on new scan start to get right RSSI value
        
        if(scan_start) {
            scan_start=0;
#if 1
	    freq=FREQ_MIN;

            setRxFreq(freq);
            last_channel_freq=freq;
#else

            setRxChannel(channelIndex);
            last_channel_index=channelIndex;
#endif
        }


#if 1

	channel = channel_from_freq(freq);
	if(cn>=0) {
	    //channelIndex = pgm_read_byte(channelList + channel);
	    channelIndex = index_from_channel(channel);
	    uint8_t currentChannelName      = pgm_read_byte(channelNames + channelIndex);
	} else {
	    channelIndex=0;
	    currentChannelName=0xff;
	}
#else
        uint8_t  currentChannelName      = pgm_read_byte(channelNames + channelIndex);
        uint16_t freq                    = pgm_read_word(channelFreqTable  + channelIndex);
#endif

        // print bar for spectrum
        wait_rssi_ready();
        // value must be ready
        rssi = readRSSI();

        if(state == STATE_SCAN) {
            if (rssi > RSSI_SEEK_TRESHOLD) {
                if(rssi_best < rssi) {
                    rssi_best = rssi;
                    freq_best = freq;
                }
            }
        }

        drawScreen.updateBandScanMode((state == STATE_RSSI_SETUP), channel, rssi, currentChannelName, freq, rssi_setup_min_a, rssi_setup_max_a);

// 5640 to 5950 = 310 MHz, or 150 steps
// or 5495 ?
// or from 5500 to 6000 ? 500mhz 250 steps,avreage RSSI on screen in 2 step

#if 1
	if(freq < FREQ_MAX){
	    freq +=2; // 2MHz step
	} else { // at end
	    freq = FREQ_MIN;

            if(state == STATE_RSSI_SETUP) {
                if(!rssi_setup_run--) {
                    // setup done
                    EEPROM_write_word(EEPROM_ADR_RSSI_MIN_A, (rssi_min_a=rssi_setup_min_a));
                    EEPROM_write_word(EEPROM_ADR_RSSI_MAX_A, (rssi_max_a=rssi_setup_max_a));

                    EEPROM_write_word(EEPROM_ADR_RSSI_MIN_B, (rssi_min_b=rssi_setup_min_b));
                    EEPROM_write_word(EEPROM_ADR_RSSI_MAX_B, (rssi_max_b=rssi_setup_max_b));

                    state=EEPROM_read(EEPROM_ADR_STATE);
                    beep(1000);
                }
            }
	}
#else
        // next channel
        if (channel < CHANNEL_MAX) {
            channel++;
        } else { // we now at end
            channel=CHANNEL_MIN;
            if(state == STATE_RSSI_SETUP) {
                if(!rssi_setup_run--) {
                    // setup done
                    EEPROM_write_word(EEPROM_ADR_RSSI_MIN_A, (rssi_min_a=rssi_setup_min_a));
                    EEPROM_write_word(EEPROM_ADR_RSSI_MAX_A, (rssi_max_a=rssi_setup_max_a));

                    EEPROM_write_word(EEPROM_ADR_RSSI_MIN_B, (rssi_min_b=rssi_setup_min_b));
                    EEPROM_write_word(EEPROM_ADR_RSSI_MAX_B, (rssi_max_b=rssi_setup_max_b));

                    state=EEPROM_read(EEPROM_ADR_STATE);
                    beep(1000);
                }
            }
        }
#endif
        // new scan possible by press scan
        if (BTN_UP == LOW) { // force new full new scan
            beep(50); // beep & debounce
            delay(KEY_DEBOUNCE); // debounce
            last_state=255; // force redraw by fake state change ;-)
            channel=CHANNEL_MIN;
	    freq=FREQ_MIN;
            scan_start=1;
            rssi_best=0;
        }
        // update index after channel change
        //channelIndex = pgm_read_byte(channelList + channel);
        channelIndex = index_from_channel(channel);
    }


    if(state == STATE_SETUP_MENU) {
        // simple menu
        char menu_id=0;
        in_menu=1;
        drawScreen.setupMenu();
        time_screen_saver=millis();
        int editing = -1;
        do {
            in_menu_time_out=50;
            drawScreen.updateSetupMenu(menu_id, settings_beeps, settings_orderby_channel, call_sign, editing);
            while(--in_menu_time_out && ((BTN_MODE == HIGH) && (BTN_UP == HIGH) && (BTN_DOWN == HIGH))) { // wait for next key press or time out
	        float volt = readBattery() / 100;
	        if(in_menu_time_out % 10 == 0)
    	    	    TV.print(5+(6*8), 5+3*MENU_Y_SIZE, volt);
                delay(100); // timeout delay 5s
            }

            if(in_menu_time_out <= 0 ) {
                state = state_last_used;
                break; // Timed out, Don't save...
            }

            if(BTN_MODE == LOW) {       // channel UP
                // do something about the users selection
                switch(menu_id) {
                    case 0: // Channel Order Channel/Frequency
                        settings_orderby_channel = !settings_orderby_channel;
                        break;
                    case 1:// Beeps enable/disable
                        settings_beeps = !settings_beeps;
                        break;

                    case 2:// Edit Call Sign
                        editing++;
                        if(editing>9) {
                            editing=-1;
                        }
                        break;
                    case 3:// Calibrate RSSI
                        in_menu = 0;
                        for (uint8_t loop=0;loop<10;loop++) {
#define RSSI_SETUP_BEEP 25
                            beep(RSSI_SETUP_BEEP); // beep & debounce
                            delay(RSSI_SETUP_BEEP); // debounce
                        }
                        state=STATE_RSSI_SETUP;
                        break;
                    case 4:
                        in_menu = 0; // save & exit menu
                        state=STATE_SAVE;
                        break;
                }
            }
            else if(BTN_UP == LOW) {
                if(editing == -1) {
                    menu_id++;
#ifdef TVOUT_SCREENS
                    if(menu_id == 2) {
                        menu_id++;
                    }
#endif
                } else { // change current letter in place
                    call_sign[editing]++;
                    call_sign[editing] > '}' ? call_sign[editing] = ' ' : false; // loop to oter end
                }

            }
            else if(BTN_DOWN == LOW) {
                if(editing == -1) {
                    menu_id--;

#ifdef TVOUT_SCREENS
                    if(menu_id == 2) {
                        menu_id--;
                    }
#endif
                } else { // change current letter in place
                    call_sign[editing]--;
                    call_sign[editing] < ' ' ? call_sign[editing] = '}' : false; // loop to oter end
                }
            }

            if(menu_id > 4) {
                menu_id = 0;
            }
            if(menu_id < 0) {
                menu_id = 4;
            }

            beep(50); // beep & debounce
            do {
                delay(150);// wait for button release
            }
            while(editing==-1 && (BTN_MODE == LOW || BTN_UP == LOW || BTN_DOWN == LOW));
        } while(in_menu);
    }


    if(time_screen_saver!=0 && time_screen_saver+5000 < millis()) {
        state = STATE_SCREEN_SAVER;
    }


    /*****************************/
    /*   General house keeping   */
    /*****************************/
    if(freq && freq != last_channel_freq){ // надо установить частоту напрямую
	last_channel_freq = freq;
	setRxFreq(freq);
        time_of_tune=millis();
    }else if(last_channel_index != channelIndex) {        // tune channel on demand
        setRxChannel(channelIndex);
        last_channel_index=channelIndex;
        // keep time of tune to make sure that RSSI is stable when required
        time_of_tune=millis();
        // give 3 beeps when tuned to give feedback of correct start
    }

    if(first_tune)  {
            first_tune=0;
#define UP_BEEP 100
            beep(UP_BEEP);
            delay(UP_BEEP);
            beep(UP_BEEP);
            delay(UP_BEEP);
            beep(UP_BEEP);
    }

}

/*###########################################################################*/

void beep(uint16_t time){
    digitalWrite(LED_PIN, HIGH);
    
    if(settings_beeps){
        digitalWrite(PIN_buzzer, LOW);
        delay(time/2);
        digitalWrite(PIN_buzzer, HIGH); // deactivate beep
/* for passive buzzer
    if(settings_beeps){ 
        digitalWrite(PIN_buzzer, HIGH);
        uint32_t pt=millis()+time/2;
        while(pt>millis()){
    	    delayMicroseconds(137);
	    digitalWrite(PIN_buzzer, !digitalRead(PIN_buzzer)); // activate beep        
        }
        digitalWrite(PIN_buzzer, LOW); // deactivate beep
*/
    } else
        delay(time/2);

    digitalWrite(LED_PIN, LOW);
}


byte index_from_channel(byte ch){
    return pgm_read_byte(channelList + ch);
}


uint8_t channel_from_index(uint8_t channelIndex){
    uint8_t id=0;
    uint8_t channel=0;
    
    for (id=0;id<=CHANNEL_MAX;id++) {
        if( index_from_channel(id) == channelIndex) {
            channel=id;
            break;
        }
    }
    return channel;
}


uint8_t channel_from_freq(uint16_t freq){
    uint8_t id=0;
    uint8_t channel=0;
    
    for (id=0;id<=CHANNEL_MAX;id++) {
        if((pgm_read_byte(channelFreqTable + id) - freq) <=2 ) { // +- 2MHz
            channel=id;
            break;
        }
    }
    return channel;
}


void wait_rssi_ready()
{
    // CHECK FOR MINIMUM DELAY
    // check if RSSI is stable after tune by checking the time

    // module need >20ms to tune.
    // 25 ms will do a 40 channel scan in 1 second.
#define MIN_TUNE_TIME 25
    while(millis()-time_of_tune < MIN_TUNE_TIME) {
        // wait until tune time is full filled
        delay(1);
    }
}

uint16_t isDiversity(){
    return analogRead(rssiPinB) >= 5;
}

uint16_t readRSSI(){
    return readRSSI(-1);
}
uint16_t readRSSI(char receiver)
{
    uint16_t rssi = 0;
    uint16_t rssiA = 0;

    uint16_t rssiB = 0;
    for (uint8_t i = 0; i < RSSI_READS; i++) {
        rssiA += analogRead(rssiPinA);//random(RSSI_MAX_VAL-200, RSSI_MAX_VAL);//

        rssiB += analogRead(rssiPinB);//random(RSSI_MAX_VAL-200, RSSI_MAX_VAL);//
    }
    rssiA = rssiA/RSSI_READS; // average of RSSI_READS readings

    rssiB = rssiB/RSSI_READS; // average of RSSI_READS readings
    // special case for RSSI setup
    if(state==STATE_RSSI_SETUP)  { // RSSI setup
        if(rssiA < rssi_setup_min_a) {
            rssi_setup_min_a=rssiA;
        }
        
        if(rssiA > rssi_setup_max_a) {
            rssi_setup_max_a=rssiA;
        }

        if(rssiB < rssi_setup_min_b) {
            rssi_setup_min_b=rssiB;
        }
        
        if(rssiB > rssi_setup_max_b) {
            rssi_setup_max_b=rssiB;
        }
    }


    rssiA = constrain(rssiA, rssi_min_a, rssi_max_a);    //original 90---250
    rssiA=rssiA-rssi_min_a; // set zero point (value 0...160)
    rssiA = map(rssiA, 0, rssi_max_a-rssi_min_a , 1, 100);   // scale from 1..100%
    rssiB = constrain(rssiB, rssi_min_b, rssi_max_b);    //original 90---250
    rssiB=rssiB-rssi_min_b; // set zero point (value 0...160)
    rssiB = map(rssiB, 0, rssi_max_b-rssi_min_b , 1, 100);   // scale from 1..100%

    if(receiver == -1) { // we should chose receiver
        switch(diversity_mode) {
        case useReceiverAuto:
            // select receiver
            if((int)abs((float)(((float)rssiA - (float)rssiB) / (float)rssiB) * 100.0) >= DIVERSITY_CUTOVER) {
                if(rssiA > rssiB && diversity_check_count > 0) {
                    diversity_check_count--;
                }
                if(rssiA < rssiB && diversity_check_count < DIVERSITY_MAX_CHECKS) {
                    diversity_check_count++;
                }
                // have we reached the maximum number of checks to switch receivers?
                if(diversity_check_count == 0 || diversity_check_count >= DIVERSITY_MAX_CHECKS) {
                    receiver=(diversity_check_count == 0) ? useReceiverA : useReceiverB;
                } else {
                    receiver=active_receiver;
                }
            } else {
                receiver=active_receiver;
            }
            break;
    
        case useReceiverB:
            receiver=useReceiverB;
            break;
    
        case useReceiverA:
        default:
            receiver=useReceiverA;
            break;
        }
        // set the antenna LED and switch the video
        setReceiver(receiver);
    }

    if(receiver == useReceiverA || state==STATE_RSSI_SETUP) {
        rssi = rssiA;
    } else {
        rssi = rssiB;
    }
    return (rssi);
}

void setReceiver(uint8_t receiver) {
    if(!is_menu_mode) {
        if(receiver == useReceiverA) {
            digitalWrite(SWITCH_0_PIN, LOW);
	}  else  {
            digitalWrite(SWITCH_0_PIN, HIGH);
        }
    }
    active_receiver = receiver;
}

void setMenu(byte isMenu){
    is_menu_mode=isMenu;
//    menu_timer=timer;
    
    if(isMenu){
        digitalWrite(SWITCH_0_PIN, LOW);
        digitalWrite(SWITCH_1_PIN, HIGH);
    } else {
        digitalWrite(SWITCH_1_PIN, LOW);
    
	setReceiver(active_receiver);
    }
    
}

void setRxChannel(uint8_t channel)
{
  uint8_t i;

  freq=0;

  uint16_t channelData = pgm_read_word(channelTable + channel);

  uint16_t freq_l = pgm_read_word(channelFreqTable + channel);
  uint16_t channelData2 = calculateFreq(freq_l);

  last_channel_freq = freq;

//if(channelData2 != channelData)
//    Serial.printf_P(PSTR("Freq data not match! freq=%d table=%d calc=%d\n"),freq,channelData,channelData2);

  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0

//  SERIAL_ENABLE_HIGH(); -- high is a normal state
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0(); // register 8 - Receiver Control Register 1

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0(); // read from

  // remaining zeros
  for (i = 20; i > 0; i--) // zero register
    SERIAL_SENDBIT0();

  // Clock the data in
  SERIAL_ENABLE_HIGH(); // end of write
  
  
  //delay(2);
  delayMicroseconds(10);
//  SERIAL_ENABLE_LOW(); // enable

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
//  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)  {
    // Is bit high or low?
    if (channelData & 0x1)    {
      SERIAL_SENDBIT1();
    }   else    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(10);

//  digitalWrite(slaveSelectPin, LOW);
//  digitalWrite(spiClockPin, LOW);
//  digitalWrite(spiDataPin, LOW);
}


uint16_t calculateFreq(unsigned int freq){
    uint16_t   ratio=(freq - 479)/2;
    
    uint16_t   regN = ratio >> 5;	// N value
    byte       regA = ratio & 0x1F; // A value
    
    return (regN << 7) | regA;

}


void setRxFreq (unsigned int freq){
    byte i;
    last_channel_freq = freq;
    
    uint16_t channelData=calculateFreq(freq);
    
  //  SYN_RF_N_REG [12:0] SYN_RF_A_REG [6:0]
// For 5.8Ghz band, FLO = 2*(N*32+A)*(Fosc/R)
// Example: default FRF=5865MHz, FLO=5865-479=5386MHz, Fosc=8MHz, R=8
// 5385/2=(N*32+A)*8Mhz/8=2*(N*32+A)*1MHz
// N=84(=1010100), A=5(=0101)
// For 5.8GHz default: 02A05H

  

  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0

  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0(); // register 8 - Receiver Control Register 1

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0(); // read from

  // remaining zeros
  for (i = 20; i > 0; i--) // zero register
    SERIAL_SENDBIT0();

  // Clock the data in
  SERIAL_ENABLE_HIGH(); // end of write
  
  
  //delay(2);
  delayMicroseconds(10);
//  SERIAL_ENABLE_LOW(); // enable

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
//  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)  {
    // Is bit high or low?
    if (channelData & 0x1)    {
      SERIAL_SENDBIT1();
    }   else    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(10);

/*
  digitalWrite(SSP,LOW);
  SPI.transfer(data0);
  SPI.transfer(data1);
  SPI.transfer(data2);
  SPI.transfer(data3);
  digitalWrite(SSP,HIGH);
*/
}


void SERIAL_SENDBIT1()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, LOW);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(1);
}

static float volt;

uint16_t readBattery(){
    long sum=0;
    for(byte i=0; i<16; i++)
	sum+=analogRead(BATTERY_IN);

    if(volt==0) volt=sum/16.0;
    else
        volt = volt*0.99 + (sum / 16.0 * 0.01);
    return volt * 0.0283143507973 * 100;
}

byte CalcNCells(){
//    int v=getExtVoltage(); уже считано
    uint16_t v=readBattery;
    byte n;

                
    if(v < 1100) {
        n=0;
    }
    if(v<4400)
        n=1;
    //if(v<88)
    else if(v<8800)
        n=2;
    else if00(v<13200)
        n=3;
    else if(v<17600)
        n=4;
    else if(v<22000)
        n=5;
    else
        n=6;
                    
    if(n < n_cells) return 0;
    n_cells = n;
        
    return 1;
}