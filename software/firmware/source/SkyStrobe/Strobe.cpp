/*
 * StrobeHelper.cpp
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
#include "SkyStrobe.h"
#include "EEPROMHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "Sound.h"
#include "Strobe.h"
#include "EEPROM.h"
#include "TrafficHelper.h"

static int StrobePin = SOC_UNUSED_PIN;
uint32_t StrobeSetupMarker = 0;
static uint32_t StrobeTimeMarker = 0;
static uint32_t StrobePauseMarker = 0;
static int StrobeFlashes = 0;      /* how many flashes */
static bool StrobeState = false;   /* true = lit */
static int StrobeOnMS  = 0;        /* how long each flash */
static int StrobeOffMS = 0;        /* how long between flashes */
static int alarm_level = ALARM_LEVEL_NONE;
bool self_test_strobe = true;

void Strobe_setup(void)
{
  StrobePauseMarker = StrobeSetupMarker = millis();
  StrobePin = STROBEPIN;
  if (StrobePin == SOC_UNUSED_PIN)
      return;
  pinMode(StrobePin, OUTPUT);
  StrobeState = false;
  StrobeTimeMarker = 0;
  digitalWrite(StrobePin, LOW);
  if (BLUELEDPIN != SOC_UNUSED_PIN) {
      pinMode(BLUELEDPIN, OUTPUT);
      digitalWrite(BLUELEDPIN, HIGH);   /* LED connected to +Vcc */
  }
}

void Strobe_Start()
{
    if (StrobeTimeMarker != 0)   // flashing currently in progress
        return;

    if (StrobePin != SOC_UNUSED_PIN) {

        if (alarm_level >= ALARM_LEVEL_LOW) {
          StrobeFlashes = flashes_alarm;
          StrobeOnMS = ms_alarm;
          StrobeOffMS = gap_alarm;
        } else {
          StrobeFlashes = flashes_noalarm;
          StrobeOnMS = ms_noalarm;
          StrobeOffMS = gap_noalarm;
        }

        digitalWrite(StrobePin, HIGH);
        blue_LED(true);                  // defined in Sound.cpp
        StrobeState = true;
        StrobeTimeMarker = millis();

    } else {

        /* no hardware output, but pause anyway */
        StrobePauseMarker = millis();
    }

    if (settings->bridge != BRIDGE_SERIAL) {
        Serial.print("Strobe flash at alarm level: ");
        Serial.println(alarm_level);
    }
    if (settings->protocol == PROTOCOL_NMEA) {
        snprintf_P(NMEAbuf, sizeof(NMEAbuf)-4, PSTR("$PSKSF,%d*"), alarm_level);
        NMEA_Out(NMEAbuf);
    }
}

void Strobe_Continue()
{
    if (StrobePin == SOC_UNUSED_PIN)
        return;

    if (StrobeFlashes > 1) {
      if (StrobeState == 1) {   /* a flash is ending */
         digitalWrite(StrobePin, LOW);
         blue_LED(false);
         StrobeState = false;
      } else {  /* Strobe is off, start another flash */
         digitalWrite(StrobePin, HIGH);
         blue_LED(true);
         StrobeState = true;
         --StrobeFlashes;
      }
      StrobeTimeMarker = millis();   /* reset timer for the next flash or gap */

    } else {
      /* pause for now */
      digitalWrite(StrobePin, LOW);
      blue_LED(false);
      StrobeTimeMarker = 0;
      StrobePauseMarker = millis();   /* reset timer for the next flash burst */
    }
}

bool hasGNSS()
{
    if (settings->protocol == PROTOCOL_GDL90)
        return GDL90_hasOwnShip();
    return NMEA_hasGNSS();
}

void Strobe_loop(void)
{
  if (settings->strobe == STROBE_OFF)
      return;

  if (StrobeTimeMarker != 0) {

      /* currently flashing, complete the pattern */

      uint32_t duration = (StrobeState ? StrobeOnMS : StrobeOffMS);

      if ((millis() - StrobeTimeMarker) > duration)
          Strobe_Continue();

  } else if (StrobePauseMarker != 0) {

      /* not currently flashing, flash again after pause */

      if (self_test_strobe) {
          if (millis() > StrobeSetupMarker + 1000 * self_test_sec)
              self_test_strobe = false;
          alarm_level = ((millis() & 0x6000) == 0x6000) ? ALARM_LEVEL_LOW : ALARM_LEVEL_NONE;
      } else {
          alarm_level = max_alarm_level;
      }
      uint32_t pause_ms = 0;
      if (self_test_strobe) {
            pause_ms = alarm_level > ALARM_LEVEL_NONE ? pause_alarm : pause_noalarm;
      } else {
        if (alarm_level > ALARM_LEVEL_NONE) {
            pause_ms = pause_alarm;
        } else if ((settings->strobe == STROBE_AIRBORNE && ThisAircraft.airborne)
                 || (settings->strobe == STROBE_AIRBORNE && (! hasGNSS()))
                 || settings->strobe == STROBE_ALWAYS) {
            pause_ms = pause_noalarm;
        }
      }

      if (pause_ms != 0 && (millis() - StrobePauseMarker) > pause_ms)
            Strobe_Start();
  }
}

void Strobe_fini(void)
{
  Strobe_setup();         // turns strobe off
  StrobePauseMarker = 0;  // prevents new flashes
}

