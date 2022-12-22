/*
 * TrafficHelper.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
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

#include <TimeLib.h>

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "EEPROMHelper.h"
#include "Sound.h"

#include "SkyStrobe.h"

traffic_t ThisAircraft, Container[MAX_TRACKING_OBJECTS], fo, EmptyFO;
traffic_by_dist_t traffic[MAX_TRACKING_OBJECTS];

static unsigned long UpdateTrafficTimeMarker = 0;
static unsigned long Traffic_Sound_TimeMarker = 0;

int max_alarm_level = ALARM_LEVEL_NONE;       /* global, used for visual displays */


void Traffic_Add()
{
    if (! fo.ID)
        return;

    if (fo.alarm_level == ALARM_LEVEL_NONE) {
        if (fo.distance > ALARM_ZONE_CLOSE)
            return;
        if (fabs(fo.RelativeVertical) > VERTICAL_SEPARATION)
            return;
    }

    LED_notify();   /* blink green LED to signal traffic received */

    int i;

    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].ID == fo.ID) {             // updating known traffic
        uint8_t alert_bak = Container[i].alert;
        uint8_t alert_level = Container[i].alert_level;
        Container[i] = fo;
        Container[i].alert = alert_bak;
        Container[i].alert_level = alert_level;
        return;
      }
    }

    int max_dist_ndx = 0;
    int min_level_ndx = 0;
    float max_distance = 0;

    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

      if (! Container[i].ID) {
        Container[i] = fo;            // use an empty slot
        return;
      }

      if (ThisAircraft.timestamp > Container[i].timestamp + ENTRY_EXPIRATION_TIME) {
        Container[i] = fo;            // overwrite expired
        return;
      }

      float adj_dist = Container[i].adj_dist;
      if  (adj_dist > max_distance) {
        max_dist_ndx = i;
        max_distance = adj_dist;
      }
      if (Container[i].alarm_level < Container[min_level_ndx].alarm_level
       || ((Container[i].alarm_level == Container[min_level_ndx].alarm_level)
           && (adj_dist > Container[min_level_ndx].adj_dist)))
              min_level_ndx = i;
    }

    if (fo.alarm_level > Container[min_level_ndx].alarm_level) {
      Container[min_level_ndx] = fo;     // alarming traffic overrides
      return;
    }

    if (fo.adj_dist <  max_distance &&
        fo.alarm_level  >= Container[max_dist_ndx].alarm_level) {
      Container[max_dist_ndx] = fo;      // overwrite farthest traffic
      return;
    }
}

void Traffic_Update(traffic_t *fop)
{
  float distance, bearing;

  if (settings->protocol == PROTOCOL_GDL90) {

    distance = nmea.distanceBetween( ThisAircraft.latitude,
                                     ThisAircraft.longitude,
                                     fop->latitude,
                                     fop->longitude);

    bearing  = nmea.courseTo( ThisAircraft.latitude,
                              ThisAircraft.longitude,
                              fop->latitude,
                              fop->longitude);

    fop->RelativeNorth     = distance * cos(radians(bearing));
    fop->RelativeEast      = distance * sin(radians(bearing));
    fop->RelativeVertical  = fop->altitude - ThisAircraft.altitude;

    // fop->RelativeBearing = bearing;
    fop->distance = distance;
    fop->adj_dist = distance + VERTICAL_SLOPE * fabs(fop->RelativeVertical);

  } else if (fop->distance != 0) {     /* a PFLAU sentence: distance & bearing known */

    fop->adj_dist = fabs(fop->distance) + VERTICAL_SLOPE * fabs(fop->RelativeVertical);

  } else {           /* a PFLAA sentence */

    distance = sqrtf(fop->RelativeNorth * fop->RelativeNorth + fop->RelativeEast * fop->RelativeEast);
    fop->distance = distance;
    fop->adj_dist = fabs(distance) + VERTICAL_SLOPE * fabs(fop->RelativeVertical);

  }

  if (fop->alarm_level < fop->alert_level)     /* if gone farther then...   */
      fop->alert_level = fop->alarm_level;     /* ...alert if comes nearer again */
}

static void Traffic_Sound()
{
  if (settings->sound == SOUND_OFF)
      return;

  int i=0;
  int ntraffic=0;
//  int bearing;
//  char message[80];
  int sound_level_ndx = 0;
  max_alarm_level = ALARM_LEVEL_NONE;          /* global, used for strobe pattern */
  int sound_alarm_level = ALARM_LEVEL_NONE;    /* local, used for sound alerts */

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID) {

       if ((ThisAircraft.timestamp <= Container[i].timestamp + SOUND_EXPIRATION_TIME)) {

         /* find the maximum alarm level, whether to be alerted or not */
         if (Container[i].alarm_level > max_alarm_level) {
             max_alarm_level = Container[i].alarm_level;
         }

         /* figure out what is the highest alarm level needing a sound alert */
         if (Container[i].alarm_level > sound_alarm_level
                  && Container[i].alarm_level > Container[i].alert_level) {
             sound_alarm_level = Container[i].alarm_level;
             sound_level_ndx = i;
         }

         // traffic[ntraffic].fop = &Container[i];
         // traffic[ntraffic].distance = Container[i].distance;
         
         ntraffic++;
       }
    }
  }

  if (ntraffic == 0) { return; }

//  qsort(traffic, ntraffic, sizeof(traffic_by_dist_t), traffic_cmp_by_alarm);

  if (sound_alarm_level > ALARM_LEVEL_NONE) {
      //Container[sound_level_ndx].alert |= TRAFFIC_ALERT_SOUND;
      Container[sound_level_ndx].alert_level = sound_alarm_level;
      Container[sound_level_ndx].timestamp = now();
      Sound_Notify(sound_alarm_level);
  }

}

void Traffic_setup()
{
  UpdateTrafficTimeMarker = millis();
  Traffic_Sound_TimeMarker = millis();
}

void Traffic_loop()
{

  if (isTimeToUpdateTraffic()) {
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
      traffic_t *fop = &Container[i];
      if (fop->ID) {
        if (ThisAircraft.timestamp > fop->timestamp + ENTRY_EXPIRATION_TIME) {    // 5s
            *fop = EmptyFO;
        } else if (ThisAircraft.timestamp >= fop->timestamp + TRAFFIC_VECTOR_UPDATE_INTERVAL) {  // 2s
            Traffic_Update(fop);
        }
      }
    }
    UpdateTrafficTimeMarker = millis();
  }

  if (isTimeToSound()) {
      Traffic_Sound();
      Traffic_Sound_TimeMarker = millis();
  }

}

void Traffic_ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID &&
        (ThisAircraft.timestamp > Container[i].timestamp + ENTRY_EXPIRATION_TIME)) {
      Container[i] = EmptyFO;
    }
  }
}

int Traffic_Count()
{
  int count = 0;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID) {
      count++;
    }
  }

  return count;
}
