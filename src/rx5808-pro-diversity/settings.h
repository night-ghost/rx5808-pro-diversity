/*
 * Setings file by Shea Ivey

The MIT License (MIT)

Copyright (c) 2015 Shea Ivey

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

#ifndef settings_h
#define settings_h


// Choose the display you will be using
// you will also have to uncomment the includes in the main project.
#define TVOUT_SCREENS
//#define OLED_128x64_ADAFRUIT_SCREENS

// u8glib has performance issues.
//#define OLED_128x64_U8G_SCREENS


#define FONT_X 8
#define FONT_Y 8


// this will be displayed on the screensaver.
// Up to 10 letters
#define CALL_SIGN "CALL SIGN"

// Feature Togglels
//#define DEBUG

#define USE_DIVERSITY

// Buzzer
#define PIN_buzzer 3
// this two are minimum required
#define buttonUp 4
#define buttonMode 5
// optional comfort buttons
#define buttonDown 6
// pins 7 & 9 - video out
// pin 8 free ?!
#define spiDataPin 10
#define slaveSelectPin 11
#define spiClockPin 12





// Receiver PINS
#define SWITCH_0_PIN A0
#define SWITCH_1_PIN A1
#define VIDEO_ON_PIN A2
// a3 free

//#define buttonSave A3
#define BATTERY_IN A4
#define PULLDN_PIN A5
#define rssiPinA A6
#define rssiPinB A7

#define useReceiverAuto 0
#define useReceiverA 1
#define useReceiverB 2

// Diversity

    // rssi strenth should be 2% greater than other receiver before switch.
    // this pervents flicker when rssi values are close and delays diversity checks counter.
#define DIVERSITY_CUTOVER 2
    // number of checks a receiver needs to win over the other to switch receivers.
    // this pervents rapid switching.
    // 1 to 10 is a good range. 1 being fast switching, 10 being slow 100ms to switch.
#define DIVERSITY_MAX_CHECKS 5

// key debounce delay in ms
// NOTE: good values are in the range of 100-200ms
// shorter values will make it more reactive, but may lead to double trigger
#define KEY_DEBOUNCE 200

#define LED_PIN 13 // ARDUINO led PIN

// number of analog rssi reads to average for the current check.
#define RSSI_READS 50
// RSSI default raw range
#define RSSI_MIN_VAL 90
#define RSSI_MAX_VAL 300
// 75% threshold, when channel is printed in spectrum
#define RSSI_SEEK_FOUND 75
// 80% under max value for RSSI
#define RSSI_SEEK_TRESHOLD 80
// scan loops for setup run
#define RSSI_SETUP_RUN 3

#define STATE_NONE 0
#define STATE_SEEK_FOUND 1
#define STATE_SEEK 2
#define STATE_SCAN 3
#define STATE_MANUAL 4
#define STATE_DIVERSITY 5
#define STATE_SETUP_MENU 6
#define STATE_SAVE 7
#define STATE_RSSI_SETUP 8
#define STATE_SCREEN_SAVER 9

//#define START_STATE (STATE_SEEK)
#define START_STATE (STATE_SCREEN_SAVER)
#define MAX_STATE (STATE_MANUAL)

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#define CHANNEL_MAX_INDEX 39

#define FREQ_MIN 5640 
#define FREQ_MAX 5950

#define CHANNEL_MAX 39
#define CHANNEL_MIN 0

#define EEPROM_ADR_STATE 0

#define EEPROM_ADR_TUNE 1
#define EEPROM_ADR_RSSI_MIN_A 2
#define EEPROM_ADR_RSSI_MAX_A 4

#define EEPROM_ADR_DIVERSITY 6
#define EEPROM_ADR_RSSI_MIN_B 7
#define EEPROM_ADR_RSSI_MAX_B 9

#define EEPROM_ADR_BEEP 11
#define EEPROM_ADR_ORDERBY 12
#define EEPROM_ADR_CALLSIGN 20 // 10 chars

#define EEPROM_ADR_FREQ 32

#endif // file_defined

