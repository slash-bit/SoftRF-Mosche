/*
 * SoundHelper.cpp
 * Copyright (C) 2016-2021 Linar Yusupov
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

#include "../system/SoC.h"

#if defined(EXCLUDE_SOUND)
void  Sound_setup()       {}
void  Sound_Notify(int8_t level)   {}
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

/* need this for the alarm levels enum: */
#include "../protocol/radio/Legacy.h"
/* - perhaps should move those to another location? */

#include "../protocol/data/NMEA.h"

static unsigned long SoundTimeMarker = 0;
static int SoundBeeps = 0;     /* how many beeps */
static int SoundState = 0;     /* 1 = buzzing */
static int SoundToneHz = 0;    /* variable tone */
static int SoundBeepMS = 0;    /* how long each beep */
static int SoundPin = SOC_UNUSED_PIN;

#include <toneAC.h>

void ext_buzzer(bool state)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN)
      digitalWrite(SOC_GPIO_PIN_BUZZER, state? HIGH : LOW);
}

static int volume = 10;

void Sound_setup(void)
{
  if (settings->volume == BUZZER_EXT) {
      if (SOC_GPIO_PIN_BUZZER == SOC_UNUSED_PIN)
          return;
      pinMode(SOC_GPIO_PIN_BUZZER, OUTPUT);
      ext_buzzer(false);
  } else {
      toneAC_setup(SOC_GPIO_PIN_BUZZER2, SOC_GPIO_PIN_BUZZER);
      volume = (settings->volume == BUZZER_VOLUME_LOW ? 8 : 10);
  }
  SoundToneHz = 0;
  SoundBeepMS = 0;
  SoundBeeps = 0;
  SoundState = 0;
  SoundTimeMarker = 0;
}

void Sound_Notify(int8_t alarm_level)
{
  if (settings->volume == BUZZER_OFF)
      return;

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
  } else {    /* whether NONE or CLOSE */
    return;
  }

  if (settings->volume == BUZZER_EXT) {
    ext_buzzer(true);
  } else {
    int duration = 0;           // forever, until turned off
    bool background = true;    // return to main thread while sounding tone
    toneAC(SoundToneHz, volume, duration, background);
  }
  SoundState = 1;
  SoundTimeMarker = millis();

  if (settings->nmea_l || settings->nmea2_l) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSRAA,%d*"), alarm_level-1);
      NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer)-10);
      NMEA_Outs(settings->nmea_l, settings->nmea2_l, (byte *) NMEABuffer, strlen(NMEABuffer), false);
  }
}

void Sound_loop(void)
{
  if (SoundTimeMarker != 0 && millis() - SoundTimeMarker > SoundBeepMS) {

    if (SoundBeeps > 1) {

      if (SoundState == 1) {   /* a beep is ending */
        if (settings->volume == BUZZER_EXT)
          ext_buzzer(false);
        else
          noToneAC();
        SoundState = 0;
      } else {  /* sound is off, start another beep */
        if (settings->volume == BUZZER_EXT)
          ext_buzzer(true);
        else
          toneAC(SoundToneHz, volume, 0, true);
        SoundState = 1;
        --SoundBeeps;
      }
      SoundTimeMarker = millis();   /* reset timer for the next beep or gap */

    } else {   /* done beeping, turn it all off */

      if (settings->volume == BUZZER_EXT)
        ext_buzzer(false);
      else
        noToneAC();
      Sound_setup();
    }

    return;
  }

#if 0
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
#endif
}

void Sound_fini(void)
{
  if (settings->volume == BUZZER_EXT)
    ext_buzzer(false);
  else
    noToneAC();
  SoundTimeMarker = 0;
}

#endif  /* ESP32 */

#endif  /* EXCLUDE_SOUND */
