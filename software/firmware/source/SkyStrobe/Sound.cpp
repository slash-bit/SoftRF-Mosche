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

#include "SoCHelper.h"

#if defined(EXCLUDE_SOUND)
void  Sound_setup()       {}
bool  Sound_Notify(int level)  {}
void  Sound_loop()        {}
void  Sound_fini()        {}
#else

#include "Sound.h"
#include "EEPROM.h"
#include "NMEAHelper.h"

static unsigned long SoundTimeMarker = 0;
static int SoundBeeps = 0;     /* how many beeps */
static int SoundState = 0;     /* 1 = buzzing */
static int SoundToneHz = 0;    /* variable tone */
static int SoundBeepMS = 0;    /* how long each beep */

void Sound_setup(void)
{
  SoC->Sound_tone(0, BUZZER_VOLUME_FULL);
  SoundToneHz = 0;
  SoundBeepMS = 0;
  SoundBeeps = 0;
  SoundState = 0;
  SoundTimeMarker = 0;
}

void Sound_Notify(int level)
{
  if (SoundTimeMarker == 0) {

    if (level == ALARM_LEVEL_LOW) {
      SoundToneHz = ALARM_TONE_HZ_LOW;
      SoundBeepMS = ALARM_TONE_MS_LOW;
      SoundBeeps  = ALARM_BEEPS_LOW;
    } else if (level == ALARM_LEVEL_IMPORTANT) {
      SoundToneHz = ALARM_TONE_HZ_IMPORTANT;
      SoundBeepMS = ALARM_TONE_MS_IMPORTANT;
      SoundBeeps  = ALARM_BEEPS_IMPORTANT;
    } else if (level == ALARM_LEVEL_URGENT) {
      SoundToneHz = ALARM_TONE_HZ_URGENT;
      SoundBeepMS = ALARM_TONE_MS_URGENT;
      SoundBeeps  = ALARM_BEEPS_URGENT;
    } else {    /* whether NONE or CLOSE */
      Sound_setup();
      return;
    }

    SoC->Sound_tone(SoundToneHz, BUZZER_VOLUME_FULL);
    SoundState = 1;
    SoundTimeMarker = millis();

  }
}

void Sound_loop(void)
{
  if (SoundTimeMarker != 0 && millis() - SoundTimeMarker > SoundBeepMS) {
  
    if (SoundBeeps > 1) {
      if (SoundState == 1) {   /* a beep is ending */
         SoC->Sound_tone(0, BUZZER_VOLUME_FULL);
         SoundState = 0;
      } else {  /* sound is off, start another beep */
         SoC->Sound_tone(SoundToneHz, BUZZER_VOLUME_FULL);
         SoundState = 1;
         --SoundBeeps;
      }
      SoundTimeMarker = millis();   /* reset timer for the next beep or gap */

    } else {  /* done beeping, turn it all off */
      Sound_setup();
    }
  }
}

void Sound_fini(void)
{
  Sound_setup();
}

#endif /* EXCLUDE_SOUND */
