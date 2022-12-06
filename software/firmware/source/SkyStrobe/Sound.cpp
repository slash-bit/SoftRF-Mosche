/*
 * Sound.cpp
 * Copyright (C) 2022 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "SkyStrobe.h"

#if defined(EXCLUDE_SOUND)
void  Sound_setup()       {}
bool  Sound_Notify(int level)  {}
void  Sound_loop()        {}
void  Sound_fini()        {}
#else

#if !defined(ESP32)
void  Sound_setup()       {}
void  Sound_Notify(int8_t level)   {}
void  Sound_loop()        {}
void  Sound_fini()        {}
#else

#include "Sound.h"
#include "Strobe.h"
#include "EEPROM.h"
#include "NMEAHelper.h"

#include <toneAC.h>

static unsigned long SoundTimeMarker = 0;
static unsigned long LEDTimeMarker = 0;
static int SoundBeeps = 0;     /* how many beeps */
static int SoundState = 0;     /* 1 = buzzing */
static int SoundToneHz = 0;    /* variable tone */
static int SoundBeepMS = 0;    /* how long each beep */

void ext_buzzer(bool state)
{
  if (SOC_GPIO_PIN_DCBUZZ != SOC_UNUSED_PIN)
      digitalWrite(SOC_GPIO_PIN_DCBUZZ, state? HIGH : LOW);
}

void red_LED(bool state)
{
  if (REDLEDPIN != SOC_UNUSED_PIN)
      digitalWrite(REDLEDPIN, state? LOW : HIGH);   /* LED connected from pin to +Vcc */
}

void green_LED(bool state)
{
  if (GREENLEDPIN != SOC_UNUSED_PIN)
      digitalWrite(GREENLEDPIN, state? LOW : HIGH);
}

void blue_LED(bool state)
{
  if (BLUELEDPIN != SOC_UNUSED_PIN)
      digitalWrite(BLUELEDPIN, state? LOW : HIGH);
}

// This is for the OTHER RGB LED on the EZSBC board, which
// otherwise displays serial (USB) activity in red and green.
// This LED is not available without USB power.
void blue2_LED(bool state)
{
//  if (SOC_GPIO_PIN_LED_BLUE2 != SOC_UNUSED_PIN)
//      digitalWrite(SOC_GPIO_PIN_LED_BLUE2, state? LOW : HIGH);
}

void toggle_green_LED()       // unlike LED_notify this does not depend on Sound_loop().
{
    static bool state = false;
    state = ! state;
    green_LED(state);
}

void toggle_red_LED()       // unlike LED_notify this does not depend on Sound_loop().
{
    static bool state = false;
    state = ! state;
    red_LED(state);
}

void LED_notify()           // called from Traffic_Add() to signal traffic received
{
    green_LED(true);
    LEDTimeMarker = millis();
}

void Sound_setup(void)
{  
  toneAC_setup(SOC_GPIO_PIN_BUZZER2, SOC_GPIO_PIN_BUZZER);

  if (SOC_GPIO_PIN_DCBUZZ != SOC_UNUSED_PIN) {
    pinMode(SOC_GPIO_PIN_DCBUZZ, OUTPUT);
    ext_buzzer(false);
  }

  if (REDLEDPIN != SOC_UNUSED_PIN) {
      pinMode(REDLEDPIN, OUTPUT);
      red_LED(false);
  }

  if (GREENLEDPIN != SOC_UNUSED_PIN) {
      pinMode(GREENLEDPIN, OUTPUT);
      green_LED(false);
  }

  if (BLUELEDPIN != SOC_UNUSED_PIN) {
      pinMode(BLUELEDPIN, OUTPUT);
      blue_LED(false);
  }

//  if (SOC_GPIO_PIN_LED_BLUE2 != SOC_UNUSED_PIN) {
//      pinMode(SOC_GPIO_PIN_LED_BLUE2, OUTPUT);
//      blue2_LED(false);
//  }

  SoundToneHz = 0;
  SoundBeepMS = 0;
  SoundBeeps = 0;
  SoundState = 0;
  SoundTimeMarker = 0;
}

void Sound_Notify(int8_t alarm_level)
{
  if (SoundTimeMarker != 0)   // beeping in progress
      return;

  if (alarm_level == ALARM_LEVEL_LOW) {
    SoundToneHz = ALARM_TONE_HZ_LOW;
    SoundBeepMS = ALARM_TONE_MS_LOW;
    SoundBeeps  = ALARM_BEEPS_LOW;
  } else if (alarm_level == ALARM_LEVEL_IMPORTANT) {
    SoundToneHz = ALARM_TONE_HZ_IMPORTANT;
    SoundBeepMS = ALARM_TONE_MS_IMPORTANT;
    SoundBeeps  = ALARM_BEEPS_IMPORTANT;
  } else if (alarm_level == ALARM_LEVEL_URGENT) {
    SoundToneHz = ALARM_TONE_HZ_URGENT;
    SoundBeepMS = ALARM_TONE_MS_URGENT;
    SoundBeeps  = ALARM_BEEPS_URGENT;
  } else {
    return;
  }

  ext_buzzer(true);
  int volume = 10;          // max volume
  int duration = 0;         // forever, until turned off
  bool background = true;   // return to main thread while sounding tone
  toneAC(SoundToneHz, volume, duration, background);

  red_LED(true);

  SoundState = 1;
  SoundTimeMarker = millis();

  if (settings->bridge != BRIDGE_SERIAL) {
        Serial.print("Sound notification at alarm level: ");
        Serial.println(alarm_level);
  }
  if (settings->protocol == PROTOCOL_NMEA) {
      char buf[16]; 
      snprintf_P(buf, sizeof(buf), PSTR("$PSKSA,%d*"), alarm_level);
      NMEA_Out(buf);
  }
}

void Sound_loop(void)
{
  if (SoundTimeMarker != 0 && millis() - SoundTimeMarker > SoundBeepMS) {

    if (SoundBeeps > 1) {

      if (SoundState == 1) {   /* a beep is ending */
          ext_buzzer(false);
          noToneAC();
          red_LED(false);
          SoundState = 0;
      } else {  /* sound is off, start another beep */
          ext_buzzer(true);
          toneAC(SoundToneHz, 10, 0, true);
          red_LED(true);
          SoundState = 1;
          --SoundBeeps;
      }
      SoundTimeMarker = millis();   /* reset timer for the next beep or gap */

    } else {   /* done beeping, turn it all off */

        ext_buzzer(false);
        noToneAC();
        Sound_setup();
    }
  }

  /* if green LED was turned on by LED_notify(), turn it off 300 ms later */
  if (LEDTimeMarker != 0 && millis() - LEDTimeMarker > 300) {
    green_LED(false);
    LEDTimeMarker = 0;
  }

  /* always blink the BLUE2 LED, it will only actually blink if USB connected */
  static uint32_t blue2_timer = 0;
  static bool blue2_on = false;
  if (millis() > blue2_timer) {
     blue2_on = !blue2_on;
     blue2_LED(blue2_on);
     blue2_timer = millis() + (blue2_on? 250 : 750);
  }

  /* strobe does a self test, do something similar with sound */
  uint32_t t = millis();
  if (t < StrobeSetupMarker + 1000 * STROBE_INITIAL_RUN) {
      if ((t & 0x6F80) == 0x6F80 && SoundTimeMarker == 0) {
          static int8_t level = ALARM_LEVEL_LOW;
          ++level;
          if (level > ALARM_LEVEL_URGENT)
              level = ALARM_LEVEL_LOW;
          Sound_Notify(level);
      }
  }
}

void Sound_fini(void)
{
    ext_buzzer(false);
    noToneAC();
    SoundTimeMarker = 0;
}

#endif  /* ESP32 */

#endif /* EXCLUDE_SOUND */
