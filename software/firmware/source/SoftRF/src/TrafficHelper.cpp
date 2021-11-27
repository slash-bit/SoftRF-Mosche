/*
 * TrafficHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
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

#include "TrafficHelper.h"
#include "driver/EEPROM.h"
#include "driver/RF.h"
#include "driver/GNSS.h"
#include "driver/Sound.h"
#include "ui/Web.h"
#include "protocol/radio/Legacy.h"

unsigned long UpdateTrafficTimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
traffic_by_dist_t traffic_by_dist[MAX_TRACKING_OBJECTS];

static int8_t (*Alarm_Level)(ufo_t *, ufo_t *);

/*
 * No any alarms issued by the firmware.
 * Rely upon high-level flight management software.
 */
static int8_t Alarm_None(ufo_t *this_aircraft, ufo_t *fop)
{
  return ALARM_LEVEL_NONE;
}

/*
 * Simple, distance based alarm level assignment.
 */
static int8_t Alarm_Distance(ufo_t *this_aircraft, ufo_t *fop)
{
  int distance = (int) fop->distance;
  int8_t rval = ALARM_LEVEL_NONE;
  int abs_alt_diff = (int) fabs(fop->alt_diff);

  if (abs_alt_diff < VERTICAL_SEPARATION) { /* no warnings if too high or too low */

    /* take altitude difference into account */
    distance += VERTICAL_SLOPE * abs_alt_diff;

    if (distance < ALARM_ZONE_URGENT) {
      rval = ALARM_LEVEL_URGENT;
    } else if (distance < ALARM_ZONE_IMPORTANT) {
      rval = ALARM_LEVEL_IMPORTANT;
    } else if (distance < ALARM_ZONE_LOW) {
      rval = ALARM_LEVEL_LOW;
    } else if (distance < ALARM_ZONE_CLOSE) {
      rval = ALARM_LEVEL_CLOSE;
    }
  }

  return rval;
}

/*
 * EXPERIMENTAL
 *
 * Linear, CoG and GS based collision prediction.
 */
static int8_t Alarm_Vector(ufo_t *this_aircraft, ufo_t *fop)
{
  int8_t rval = ALARM_LEVEL_NONE;

  float abs_alt_diff = fabs(fop->alt_diff);

  if (abs_alt_diff < VERTICAL_SEPARATION) { /* no warnings if too high or too low */

    /* Subtract 2D velocity vector of traffic from 2D velocity vector of this aircraft */ 
    float V_rel_x = this_aircraft->speed * cosf(radians(90.0 - this_aircraft->course)) -
                    fop->speed * cosf(radians(90.0 - fop->course)) ;
    float V_rel_y = this_aircraft->speed * sinf(radians(90.0 - this_aircraft->course)) -
                    fop->speed * sinf(radians(90.0 - fop->course)) ;

    float V_rel_magnitude = sqrtf(V_rel_x * V_rel_x + V_rel_y * V_rel_y) * _GPS_MPS_PER_KNOT;
    float V_rel_direction = atan2f(V_rel_y, V_rel_x) * 180.0 / PI;  /* -180 ... 180 */

    /* convert from math angle into course relative to north */
    V_rel_direction = (V_rel_direction <= 90.0 ? 90.0 - V_rel_direction :
                                                450.0 - V_rel_direction);

    /* +- some degrees tolerance for collision course */
    if (V_rel_magnitude > ALARM_VECTOR_SPEED) {

      /* time is seconds prior to impact */
      /* take altitude difference into account */

      float t = (fop->distance + VERTICAL_SLOPE*abs_alt_diff) / V_rel_magnitude;

      float angle = fabs(V_rel_direction - fop->bearing);

      if (angle < ALARM_VECTOR_ANGLE) {

        /* time limit values are compliant with FLARM data port specs */

        if (t < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_URGENT;
        } else if (t < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else if (t < ALARM_TIME_LOW) {
          rval = ALARM_LEVEL_LOW;
        } else if (t < ALARM_TIME_CLOSE) {
          rval = ALARM_LEVEL_CLOSE;
        }    
      } else if (angle < ALARM_VECTOR_ANGLE * 2.0f) {

        /* reduce alarm level since direction is less direct */

        if (t < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else if (t < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_LOW;
        } else if (t < ALARM_TIME_LOW) {
          rval = ALARM_LEVEL_CLOSE;
        } else if (t < ALARM_TIME_CLOSE) {
          rval = ALARM_LEVEL_NONE;
        }    
      }
    }
  }
  return rval;
}

/*
 * "Legacy" method is based on short history of 2D velocity vectors (NS/EW)
 */
static int8_t Alarm_Legacy(ufo_t *this_aircraft, ufo_t *fop)
{

  int8_t rval = ALARM_LEVEL_NONE;

  /* TBD */

  return rval;
}

void Traffic_Update(ufo_t *fop)
{
  fop->distance = gnss.distanceBetween( ThisAircraft.latitude,
                                        ThisAircraft.longitude,
                                        fop->latitude,
                                        fop->longitude);

  fop->bearing  = gnss.courseTo( ThisAircraft.latitude,
                                 ThisAircraft.longitude,
                                 fop->latitude,
                                 fop->longitude);

  fop->alt_diff = fop->altitude - ThisAircraft.altitude;

  if (Alarm_Level) {

    fop->alarm_level = (*Alarm_Level)(&ThisAircraft, fop);

    /* if gone farther, reduce threshold for a new alert - with hysteresis. */
    /* E.g., if alarm was for LOW, alert_level was set to IMPORTANT.        */
    /* A new alarm alert will sound if close enough to now be URGENT.       */
    /* Or, if now gone to CLOSE (farther than LOW), set alert_level to LOW, */
    /* then next time reaches alarm_level IMPORTANT will give a new alert.  */
    /* Or, if now gone to NONE (farther than CLOSE), set alert_level to     */
    /* CLOSE, then next time returns to alarm_level LOW will give an alert. */

    if (fop->alarm_level < fop->alert_level)
         fop->alert_level = fop->alarm_level + 1;
  }
}

void ParseData()
{
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo.raw) ? sizeof(fo.raw) : rx_size;

#if DEBUG
    Hex2Bin(TxDataTemplate, RxBuffer);
#endif

    memset(fo.raw, 0, sizeof(fo.raw));
    memcpy(fo.raw, RxBuffer, rx_size);

    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now()); StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo.raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    if (memcmp(RxBuffer, TxBuffer, rx_size) == 0) {
      if (settings->nmea_p) {
        StdOut.println(F("$PSRFE,RF loopback is detected"));
      }
      return;
    }

    if (protocol_decode && (*protocol_decode)((void *) RxBuffer, &ThisAircraft, &fo)) {

      int i;

      fo.rssi = RF_last_rssi;

      Traffic_Update(&fo);

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr == fo.addr) {
          uint8_t alert_bak = Container[i].alert;
          uint8_t level_bak = Container[i].alert_level;
          Container[i] = fo;
          Container[i].alert = alert_bak;
          Container[i].alert_level = level_bak;
          return;
        }
      }

      int max_dist_ndx = 0;
      int min_level_ndx = 0;

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
          Container[i] = fo;
          return;
        }
#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
        if (Container[i].distance + VERTICAL_SLOPE*fabs(Container[i].alt_diff)
              > Container[max_dist_ndx].distance + 
                  VERTICAL_SLOPE*fabs(Container[max_dist_ndx].alt_diff))  {
          max_dist_ndx = i;
        }

        if (Container[i].alarm_level < Container[min_level_ndx].alarm_level)  {
          min_level_ndx = i;
        }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */
      }

#if !defined(EXCLUDE_TRAFFIC_FILTER_EXTENSION)
      if (fo.alarm_level > Container[min_level_ndx].alarm_level) {
        Container[min_level_ndx] = fo;
        return;
      }

      if ((fo.distance + VERTICAL_SLOPE*fabs(fo.alt_diff)
              <  Container[max_dist_ndx].distance
              + VERTICAL_SLOPE*fabs(Container[max_dist_ndx].alt_diff))
        &&
          fo.alarm_level >= Container[max_dist_ndx].alarm_level) {
        Container[max_dist_ndx] = fo;
        return;
      }
#endif /* EXCLUDE_TRAFFIC_FILTER_EXTENSION */

    }
}

void Traffic_setup()
{
  switch (settings->alarm)
  {
  case TRAFFIC_ALARM_NONE:
    Alarm_Level = &Alarm_None;
    break;
  case TRAFFIC_ALARM_VECTOR:
    Alarm_Level = &Alarm_Vector;
    break;
  case TRAFFIC_ALARM_LEGACY:
    Alarm_Level = &Alarm_Legacy;
    break;
  case TRAFFIC_ALARM_DISTANCE:
  default:
    Alarm_Level = &Alarm_Distance;
    break;
  }
}

void Traffic_loop()
{
  if (isTimeToUpdateTraffic()) {

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      ufo_t *fop = &Container[i];

      if (fop->addr &&
          (ThisAircraft.timestamp - fop->timestamp) <= ENTRY_EXPIRATION_TIME) {

        if ((ThisAircraft.timestamp - fop->timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL) {
          Traffic_Update(fop);
        }

        /* sound an alarm if new alert, or got two levels closer than previous  */
        /* alert, or hysteresis: got two levels farther, and then closer.       */
        /* E.g., if alarm was for LOW, alert_level was set to IMPORTANT.        */
        /* A new alarm alert will sound if close enough to now be URGENT.       */
        /* Or, if now gone to CLOSE (farther than LOW), set alert_level to LOW, */
        /* then next time reaches alarm_level IMPORTANT will give a new alert.  */
        /* Or, if now gone to NONE (farther than CLOSE), set alert_level to     */
        /* CLOSE, then next time returns to alarm_level LOW will give an alert. */

        if (fop->alarm_level > fop->alert_level
                && fop->alarm_level > ALARM_LEVEL_CLOSE) {
          Sound_Notify(fop->alarm_level);
          fop->alert_level = fop->alarm_level + 1;
          fop->alert |= TRAFFIC_ALERT_SOUND;  /* no longer actually used */
        }

      } else {
        *fop = EmptyFO;
        /* implied by emptyFO:
        fop->alert = 0;
        fop->alarm_level = 0;
        fop->alert_level = 0;
        fop->addr = 0;
        ... */
      }
    }

    UpdateTrafficTimeMarker = millis();
  }
}

void ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr &&
         (ThisAircraft.timestamp - Container[i].timestamp) > ENTRY_EXPIRATION_TIME) {
      Container[i] = EmptyFO;
    }
  }
}

int Traffic_Count()
{
  int count = 0;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr) {
      count++;
    }
  }

  return count;
}

/* this is used in Text_EPD.cpp for 'radar' display, */
/* so don't adjust for altitude difference.          */

int traffic_cmp_by_distance(const void *a, const void *b)
{
  traffic_by_dist_t *ta = (traffic_by_dist_t *)a;
  traffic_by_dist_t *tb = (traffic_by_dist_t *)b;

  if (ta->distance >  tb->distance) return  1;
/*  if (ta->distance == tb->distance) return  0; */
  if (ta->distance <  tb->distance) return -1;
  return  0;
}
