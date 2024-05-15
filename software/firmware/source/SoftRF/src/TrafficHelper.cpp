/*
 * TrafficHelper.cpp
 * Copyright (C) 2018-2021 Linar Yusupov
 *
 * Alarm_Legacy() code by Moshe Braner 2022
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
#include "driver/Buzzer.h"
#include "driver/Strobe.h"
#include "ui/Web.h"
#include "protocol/radio/Legacy.h"
#include "protocol/data/NMEA.h"
#include "ApproxMath.h"
#include "Wind.h"
#include "../SoftRF.h"

#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
#include "driver/Voice.h"
#endif
#endif

#if defined(ESP32)
#include "SPIFFS.h"
// #include <FS.h>
File AlarmLog;
bool AlarmLogOpen = false;
#endif

unsigned long UpdateTrafficTimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
uint8_t fo_raw[34];
traffic_by_dist_t traffic_by_dist[MAX_TRACKING_OBJECTS];
int max_alarm_level = ALARM_LEVEL_NONE;
bool alarm_ahead = false;                    /* global, used for visual displays */
bool relay_waiting = false;

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
 * Adjust relative altitude for relative vertical speed.
 */
float Adj_alt_diff(ufo_t *this_aircraft, ufo_t *fop)
{
  float alt_diff = fop->alt_diff;           /* positive means fop is higher than this_aircraft */
  float vsr = fop->vs - this_aircraft->vs;  /* positive means fop is rising relative to this_aircraft */
  if (abs(vsr) > 1000)  vsr = 0;            /* ignore implausible data */
  float alt_change = vsr * 0.05;  /* expected change in 10 seconds, converted to meters */

  /* only adjust towards higher alarm level: */
  if (alt_diff > 0 && alt_change < 0) {
    alt_diff += alt_change;   /* makes alt_diff smaller */
    if (alt_diff < 0)  return 0;  /* minimum abs_alt_diff */
  } else if (alt_diff < 0 && alt_change > 0) {
    alt_diff += alt_change;   /* makes alt_diff less negative */
    if (alt_diff > 0)  return 0;  /* minimum abs_alt_diff */
  }

  /* GPS altitude is fuzzy so ignore the first 60m difference */
  if (alt_diff > 0) {
    if (alt_diff < VERTICAL_SLACK)  return 0;
    return (alt_diff - VERTICAL_SLACK);
  }
  if (-alt_diff < VERTICAL_SLACK)  return 0;
  return (alt_diff + VERTICAL_SLACK);
}

/*
 * Simple, distance based alarm level assignment.
 */
static int8_t Alarm_Distance(ufo_t *this_aircraft, ufo_t *fop)
{
  int8_t rval = ALARM_LEVEL_NONE;

  float distance = fop->distance;
  if (distance > ALARM_ZONE_CLOSE
      || fabs(fop->adj_alt_diff) > VERTICAL_SEPARATION) {
    return ALARM_LEVEL_NONE;
  }

  float adj_distance;
  if (fop->adj_distance > distance)
         adj_distance = fop->adj_distance;
  else
         adj_distance = distance;

  if (adj_distance < ALARM_ZONE_EXTREME)
    --fop->alert_level;     /* may sound new alarm even if previous one was IMPORTANT */
  if (adj_distance < ALARM_ZONE_URGENT) {
    rval = ALARM_LEVEL_URGENT;
  } else if (adj_distance < ALARM_ZONE_IMPORTANT) {
    rval = ALARM_LEVEL_IMPORTANT;
  } else if (adj_distance < ALARM_ZONE_LOW) {
    rval = ALARM_LEVEL_LOW;
  } else if (adj_distance < ALARM_ZONE_CLOSE) {
    rval = ALARM_LEVEL_CLOSE;
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

  if (fop->gnsstime_ms - fop->prevtime_ms > 3000)   /* also catches prevtime_ms == 0 */
    return Alarm_Distance(this_aircraft, fop);

  float distance = fop->distance;
  if (distance > 2*ALARM_ZONE_CLOSE
      || fabs(fop->alt_diff) > 2*VERTICAL_SEPARATION) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  if (distance / (fop->speed + this_aircraft->speed)
         > ALARM_TIME_CLOSE * _GPS_MPS_PER_KNOT) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  /* if either aircraft is turning, vector method is not usable */
  if (fabs(this_aircraft->turnrate) > 3.0 || fabs(fop->turnrate) > 3.0)
        return Alarm_Distance(this_aircraft, fop);
//  if (fop->turnrate == 0.0) {   /* turnrate not available */
//    float angle = fabs(fop->course - fop->prevcourse);
//    if (angle > 270.0)  angle = 360.0 - angle;
//    if (angle > 6.0)
//      return Alarm_Distance(this_aircraft, fop);
//  }

  float V_rel_magnitude, V_rel_direction, t;

  if (fabs(fop->adj_alt_diff) < VERTICAL_SEPARATION) {  /* no alarms if too high or too low */

    float adj_distance;
    if (fop->adj_distance > distance)
           adj_distance = fop->adj_distance;
    else
           adj_distance = distance;

    /* Subtract 2D velocity vector of traffic from 2D velocity vector of this aircraft */ 
    float V_rel_y = this_aircraft->speed * cos_approx(this_aircraft->course) -
                    fop->speed * cos_approx(fop->course);                      /* N-S */
    float V_rel_x = this_aircraft->speed * sin_approx(this_aircraft->course) -
                    fop->speed * sin_approx(fop->course);                      /* E-W */

    V_rel_magnitude = approxHypotenuse(V_rel_x, V_rel_y) * _GPS_MPS_PER_KNOT;
    V_rel_direction = atan2_approx(V_rel_y, V_rel_x);     /* direction fop is coming from */

    /* +- some degrees tolerance for collision course */

    if (V_rel_magnitude > ALARM_VECTOR_SPEED) {

      /* time is seconds prior to impact */
      /* take altitude difference into account */

      t = adj_distance / V_rel_magnitude;

      if (t < ALARM_TIME_EXTREME)
          --fop->alert_level;     /* may sound new alarm even if previous one was IMPORTANT */

      float rel_angle = fabs(V_rel_direction - fop->bearing);

      if (rel_angle < ALARM_VECTOR_ANGLE && V_rel_magnitude > 3 * ALARM_VECTOR_SPEED) {

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

      } else if (rel_angle < 2 * ALARM_VECTOR_ANGLE) {

        /* reduce alarm level since direction is less direct */

        if (t < ALARM_TIME_EXTREME) {
          rval = ALARM_LEVEL_URGENT;
        } else if (t < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else if (t < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_LOW;
        } else if (t < ALARM_TIME_LOW) {
          rval = ALARM_LEVEL_CLOSE;
/*      } else if (t < ALARM_TIME_CLOSE) {
          rval = ALARM_LEVEL_NONE;               */
        }

      } else if (rel_angle < 3 * ALARM_VECTOR_ANGLE) {

        /* further reduce alarm level for larger angles */

        if (t < ALARM_TIME_EXTREME) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else if (t < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_LOW;
        } else if (t < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_CLOSE;
/*      } else if (t < ALARM_TIME_LOW) {
          rval = ALARM_LEVEL_NONE;               */
        }
      }
    }
  }

  /* send data out via NMEA for debugging */
  if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_ALARM)) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSALV,%06X,%ld,%d,%.1f,%.1f,%.1f,%.1f,%.5f,%.5f,%.1f,%.1f,%.1f,%.5f,%.5f,%.1f,%.1f,%.1f\r\n"),
      fop->addr, fop->gnsstime_ms, rval, V_rel_magnitude, V_rel_direction, fop->bearing, t,
      this_aircraft->latitude, this_aircraft->longitude, this_aircraft->altitude,
         this_aircraft->speed, this_aircraft->course,
      fop->latitude, fop->longitude, fop->altitude, fop->speed, fop->course);
    NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
  }

  return rval;
}


/*
 * VERY EXPERIMENTAL
 *
 * "Legacy" method is based on short history of (future) 2D velocity vectors (NS/EW).
 * The approach here tried to use the velocity components from the time points
 * that FLARM sends out, in the way it probably intended by sending the data
 * out in that format, and given the weak computing power of early hardware
 * - but turns out that's not what FLARM does...  See Wind.cpp project_that().
 */
static int8_t Alarm_Legacy(ufo_t *this_aircraft, ufo_t *fop)
{
  if (fop->distance > 2*ALARM_ZONE_CLOSE
      || fabs(fop->alt_diff) > 2*VERTICAL_SEPARATION) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  if (fop->distance / (fop->speed + this_aircraft->speed)
         > ALARM_TIME_LOW * _GPS_MPS_PER_KNOT) {
    return ALARM_LEVEL_NONE;
    /* save CPU cycles */
  }

  //if (fop->protocol == RF_PROTOCOL_LATEST) {
  //  return (Alarm_Vector(this_aircraft, fop));
    /* we don't know how to fully interpret the new protocol yet */
  //}

  /* here start the expensive calculations */

  /* calculate other aircraft's turn rate and project course into future time points */
  project_that(fop);

  /* Also compute such estimates for ThisAircraft  - both relative to air not ground */
  /* project_this(this_aircraft); */
  /* - was already called from Estimate_Wind() or Legacy_Encode() */

  if (fabs(this_aircraft->turnrate) < 2.0 && fabs(fop->turnrate) < 2.0) {
    /* neither aircraft is circling */
    return (Alarm_Vector(this_aircraft, fop));
    // hopefully this takes care of aerotows?
    // >>> or try and use this algorithm anyway?
  }

  /* Project relative position second by second into the future */
  /* Time points in the ns/ew array may be +2,4,6,8, and perhaps 2,6,10,14 for towplanes    */
  /*   - although that's in the FLARM transmissions, we compute what we want in Wind.cpp    */

  /* Use integer math for computational speed */

  /* also take altitude difference into account */
  int dz = (int) fop->alt_diff;
  int absdz = abs(dz);
  int adjdz = absdz;
  /* assume lower aircraft may zoom up */
  /* potential zoom altitude is about V^2/20 (m, m/s) */
  int vx, vy, vv, vv20;
  if (dz < 0) {             // other aircraft is lower
    vx = fop->air_ew[0];
    vy = fop->air_ns[0];    // quarter-meters per second airspeed
  } else {
    vx = this_aircraft->air_ew[0];
    vy = this_aircraft->air_ns[0];
  }
  vv = (vx*vx + vy*vy);
  vv20 = vv - (20*20*4*4);  // zoom to 20 m/s
  if (vv20 > 0)
    adjdz -= vv20 >> 9;     // about 2/3 of possible zoom
  adjdz -= 70;        // account for possible GPS altitude discrepancy
  if (adjdz < 0)
    adjdz = 0;
  if (adjdz > 100)
    return (ALARM_LEVEL_NONE);

  /* may want to adjust dz over time? */
  /* - but is relative vs representative of future? */
  /* - in same thermal average relative vs = 0 */

  /* prepare the second-by-second velocity vectors */
  static int thisvx[20];
  static int thisvy[20];
  static int thatvx[20];
  static int thatvy[20];
  //int vx, vy;
  int *px = thisvx;
  int *py = thisvy;
  /* considered interpolating between the 4 points, but does not seem useful */
  int i, j;
  /* if zooming to level of other aircraft, speed decreases */
  /* rough approximation: multiply speed by (1 - 5*dz/vv)   */
  /*    5 = 20, halved for average, halved again for sqrt() */
  if (vv20 > 8000 && dz > 15) {
    int32_t factor = (5*16*64) * (int32_t) absdz;
    factor = 64 - factor/vv;
    if (factor < 48)  factor = 48;
    for (i=0; i<6; i++) {
       int32_t v;
       v = this_aircraft->air_ew[i];
       v *= factor;
       vx = v >> 6;
       v = this_aircraft->air_ns[i];
       v *= factor;
       vy = v >> 6;
       for (j=0; j<3; j++) {
         *px++ = vx;
         *py++ = vy;    
       }
    }
  } else {
    for (i=0; i<6; i++) {
      vx = this_aircraft->air_ew[i];  /* quarter-meters per second */
      vy = this_aircraft->air_ns[i];
      for (j=0; j<3; j++) {
        *px++ = vx;
        *py++ = vy;    
      }
    }
  }
  *px = vx;  /* extrapolate one more second */
  *py = vy;

  /* same for the other aircraft */
  px = thatvx;
  py = thatvy;
  if (vv20 > 8000 && dz < -15) {
    int32_t factor = (5*16*64) * (int32_t) absdz;
    factor = 64 - factor/vv;
    if (factor < 48)  factor = 48;
    for (i=0; i<6; i++) {
       int32_t v;
       v = fop->air_ew[i];
       v *= factor;
       vx = v >> 6;
       v = fop->air_ns[i];
       v *= factor;
       vy = v >> 6;
       for (j=0; j<3; j++) {
         *px++ = vx;
         *py++ = vy;    
       }
    }
  } else {
    for (i=0; i<6; i++) {
      vx = fop->air_ew[i];
      vy = fop->air_ns[i];
      for (j=0; j<3; j++) {
        *px++ = vx;
        *py++ = vy;    
      }
    }
  }
  *px = vx;
  *py = vy;    

  /* 2D position of fop relative to this aircraft */
  /* - computed in Traffic_Update() */
  /* convert to quarter-meters */
  int dx = fop->dx << 2;
  int dy = fop->dy << 2;

  /* project paths over time and find minimum 3D distance */
  uint32_t minsqdist = 200*200*4*4;
  int mintime = ALARM_TIME_CLOSE;
  int vxmin = 0;
  int vymin = 0;
  /* if projections are from different times, offset the arrays */
  if (fop->projtime_ms > this_aircraft->projtime_ms + 500) {
    /* this_aircraft projection is older, shift by 1 second */
    i = 0;
    j = 1;
    dx -= thisvx[0];  /* this aircraft movement during the first second */
    dy -= thisvy[0];
  } else if (this_aircraft->projtime_ms > fop->projtime_ms + 500) {
    /* other aircraft projection is older */
    i = 1;
    j = 0;
    dx += thatvx[0];  /* other aircraft movement during the first second */
    dy += thatvy[0];
  } else {
    i = 0;
    j = 0;
  }
  int t;
  int sqdz = (adjdz*adjdz) << 4;
  for (t=0; t<18; t++) {  /* loop over the 1-second time points prepared */
    vx = thatvx[i] - thisvx[j];   /* relative velocity */
    vy = thatvy[i] - thisvy[j];
    dx += vx;   /* change in relative position over this second */
    dy += vy;
    /* dz += vz; */
    uint32_t sqdist = dx*dx + dy*dy + sqdz;
    if (sqdist < minsqdist) {
      minsqdist = sqdist;
      vxmin = vx;
      vymin = vy;
      mintime = t;
    }
    ++i;
    ++j;
  }

  int8_t rval = ALARM_LEVEL_NONE;

  /* try and set thresholds for alarms with gaggles - and tows - in mind */
  /* squeezed between size of thermal, length of tow rope, and accuracy of prediction */
  if (minsqdist < 60*60*4*4) {   /* 60 meters 3D separation */
        if (mintime < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_URGENT;
        } else if (mintime < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else {  /* min-dist time is at most 18 seconds */
          rval = ALARM_LEVEL_LOW;
        }
  } else if (minsqdist < 100*100*4*4) {   /* 100 meters */
        if (mintime < ALARM_TIME_EXTREME) {
          rval = ALARM_LEVEL_URGENT;
        } else if (mintime < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else if (mintime < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_LOW;
        } else {
          rval = ALARM_LEVEL_CLOSE;
        }
  } else if (minsqdist < 160*160*4*4 && (! this_aircraft->circling || ! fop->circling)) {
        if (mintime < ALARM_TIME_EXTREME) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else if (mintime < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_LOW;
        } else {
          rval = ALARM_LEVEL_CLOSE;
        }
  }
  uint32_t sqspeed = 0;
  if (rval > ALARM_LEVEL_NONE /* && mintime > ALARM_TIME_EXTREME */ ) {
    sqspeed = vxmin*vxmin + vymin*vymin;    /* relative speed at closest point, squared */
    if (sqspeed < 6*6*4*4) {     /* relative speed < 6 mps */
      --rval;       // <= IMPORTANT
      if (sqspeed < 4*4*4*4) {   /* relative speed < 4 mps */
        --rval;     // <= LOW
        if (sqspeed < 2*2*4*4)   /* relative speed < 2 mps */
          --rval;   // < LOW
      }
    }
  }
  if (rval < ALARM_LEVEL_NONE)
      rval = ALARM_LEVEL_NONE;

  if (rval >= ALARM_LEVEL_LOW && mintime < ALARM_TIME_EXTREME)
      --fop->alert_level;     /* may sound new alarm even if previous one was IMPORTANT */

  /* send data out via NMEA for debugging */
  if (rval > ALARM_LEVEL_CLOSE || fop->distance < ALARM_ZONE_IMPORTANT) {
    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_ALARM)) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSALL,%06X,%ld,%ld,%d,%d,%d,%d,%.1f,%.1f,%.1f,%ld,%ld,%.1f,%.1f,%.1f,%.1f\r\n"),
          fop->addr, fop->projtime_ms, this_aircraft->projtime_ms, rval, mintime, minsqdist, sqspeed,
          this_aircraft->speed, this_aircraft->heading, this_aircraft->turnrate,
          fop->dy, fop->dx, fop->alt_diff, fop->speed, fop->heading, fop->turnrate);
      NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
    }
  }

  return rval;
}

void Traffic_Update(ufo_t *fop)
{
  /* use an approximation for distance & bearing between 2 points */
  float x, y;
  y = 111300.0 * (fop->latitude  - ThisAircraft.latitude);         /* meters */
  x = 111300.0 * (fop->longitude - ThisAircraft.longitude) * CosLat(ThisAircraft.latitude);
  fop->distance = approxHypotenuse(x, y);      /* meters  */
  fop->bearing = atan2_approx(y, x);           /* degrees from ThisAircraft to fop */
  fop->dx = (int32_t) x;
  fop->dy = (int32_t) y;

  int rel_bearing = (int) (fop->bearing - ThisAircraft.course);
  rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));
  fop->RelativeBearing = rel_bearing;

  fop->alt_diff = fop->altitude - ThisAircraft.altitude;

  /* take altitude (and vert speed) differences into account as adjusted distance */
  float adj_alt_diff = Adj_alt_diff(&ThisAircraft, fop);
  fop->adj_alt_diff = adj_alt_diff;
  fop->adj_distance = fop->distance + VERTICAL_SLOPE * fabs(adj_alt_diff);

  /* follow FLARM docs: do not issue alarms about non-airborne traffic */
  /* - except in the first minute, for testing */
  if ((fop->protocol == RF_PROTOCOL_LEGACY || fop->protocol == RF_PROTOCOL_LATEST)
       && (fop->airborne == 0)
       && (millis() - SetupTimeMarker > 60000)) {
    fop->alarm_level = ALARM_LEVEL_NONE;
    return;
  }

  if (Alarm_Level) {
       fop->alarm_level = (*Alarm_Level)(&ThisAircraft, fop);

       /* if gone farther, reduce threshold for a new alert - with hysteresis. */
       /* E.g., if alarm was for LOW, alert_level was set to IMPORTANT.        */
       /* A new alarm alert will sound if close enough to now be URGENT.       */
       /* Or, if now gone to CLOSE (farther than LOW), set alert_level to LOW, */
       /* then next time reaches alarm_level IMPORTANT will give a new alert.  */
       /* Or, if now gone to NONE (farther than CLOSE), set alert_level to     */
       /* CLOSE, then next time returns to alarm_level LOW will give an alert. */

       if (fop->alarm_level < fop->alert_level)       /* if just less by 1...   */
            fop->alert_level = fop->alarm_level + 1;  /* ...then no change here */
  }
}

/* relay landed-traffic if we are airborne */
bool air_relay()
{
    static uint32_t lastrelay = 0;

    if (fo.airborne && (millis() > SetupTimeMarker + 60000)) {
      if (settings->relay < RELAY_ALL)
        return false;
      /* do not relay close-by traffic unless it is low (or landed) */
      if (fo.distance < 10000.0 && fo.alt_diff > -1000.0)
        return false;
    }

    /* only relay once in a while (5 seconds for any, 15 for same aircraft) */
    if (millis() < lastrelay + 1000*ANY_RELAY_TIME)
        return true;
    if (fo.timerelayed + ENTRY_RELAY_TIME > fo.timestamp)
        return true;

    relay_waiting = true;

    // only try and relay during first time slot,
    // to maximize chance that OGN ground stations will receive it
    if (RF_current_slot != 0 || !RF_Transmit_Ready())
        return true;

    // >>> re-encode new-protocol packets into old protocol for relaying
    bool relayed = false;
    size_t s = RF_Encode(&fo);
    if (s != 0)
        relayed = RF_Transmit(sizeof(legacy_packet_t), true);

    if (relayed) {
        fo.timerelayed = ThisAircraft.timestamp;
        lastrelay = millis();
        relay_waiting = false;
        // Serial.print("Relayed packet from ");
        // Serial.println(fo.addr, HEX);
        if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags)) {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSARL,1,%06X,%ld\r\n"),
            fo.addr, fo.timerelayed);
          NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
        }
    } else {
#if 0
        if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags)) {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSARL,0,%06X,%ld\r\n"),
            fo.addr, ThisAircraft.timestamp);
          NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
        }
#endif
    }
    return true;
}

void AddTraffic(ufo_t *fop)
{
    ufo_t *cip;
    time_t timenow = ThisAircraft.timestamp;

    bool do_relay = false;

    if (settings->rf_protocol == RF_PROTOCOL_LEGACY || settings->rf_protocol == RF_PROTOCOL_LATEST) {
      // relay some traffic if we are airborne (or in first-minute test)
      // do not relay GDL90 or ADS-B traffic
      if (settings->relay != RELAY_OFF
          && (fop->protocol == RF_PROTOCOL_LEGACY || fop->protocol == RF_PROTOCOL_LATEST)
          && fop->relayed == false         // not a packet already relayed one hop
          && (ThisAircraft.airborne || (millis() < SetupTimeMarker + 60000)))
      {
            do_relay = true;
      }
    }

    /* first check whether we are already tracking this object */
    int i;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

      cip = &Container[i];

      if (cip->addr == fop->addr) {

        // ignore external (GDL90) data about aircraft we also receive from directly
        if (fop->protocol == RF_PROTOCOL_GDL90 && cip->protocol != RF_PROTOCOL_GDL90
                      && timenow - cip->timestamp <= ENTRY_EXPIRATION_TIME)
            return;

        fop->distance = cip->distance;      // - data from previous packet or update
        fop->alt_diff = cip->alt_diff;
        fop->timerelayed = cip->timerelayed;
        if (do_relay)  do_relay = air_relay();
        // this updates fop->timerelayed, to be copied later into container[]

        /* ignore "new" GPS fixes that are exactly the same as before */
        if (fop->altitude == cip->altitude &&
            fop->latitude == cip->latitude &&
            fop->longitude == cip->longitude) {
                cip->timerelayed = fop->timerelayed;
                return;
        }

        /* overwrite old entry, but preserve fields that store history */
        /*   - they are copied into fo and then back into Container[i] */

        if ((fop->gnsstime_ms - cip->gnsstime_ms > 1200)
          /* packets spaced far enough apart, store new history */
        || (fop->gnsstime_ms - cip->prevtime_ms > 2600)) {
          /* previous history getting too old, drop it */
          /* this means using the past data from < 1200 ms ago */
          /* to avoid that would need to store data from yet another time point */
          fop->prevtime_ms  = cip->gnsstime_ms;
          fop->prevcourse   = cip->course;
          fop->prevheading  = cip->heading;
          /* fop->prevspeed = Container[i].speed; */
          fop->prevaltitude = cip->altitude;
        } else {
          /* retain the older history for now */
          fop->prevtime_ms  = cip->prevtime_ms;
          fop->prevcourse   = cip->prevcourse;
          fop->prevheading  = cip->prevheading;
          /* fop->prevspeed = cip->prevspeed; */
          fop->prevaltitude = cip->prevaltitude;
          /* >>> may want to also retain info needed to compute velocity vector at t=0 */
        }
        fop->turnrate    = cip->turnrate;   /* keep the old turn rate */
        fop->alert       = cip->alert;
        fop->alert_level = cip->alert_level;
     /* fop->callsign    = cip->callsign; */

        *cip = *fop;   /* copies the whole object/structure */

        /* Now old alert_level is in same structure, can update alarm_level:  */
        Traffic_Update(cip);    // also updates distance, alt_diff

        return;
      }
    }

    /* new object, try and find a slot for it */

    /* get distance, alt_diff, and alarm_level, to be copied later into container[] */
    Traffic_Update(fop);

    // timerelayed started out as 0 (from empty_fo)
    if (do_relay)  do_relay = air_relay();
    // this updates fop->timerelayed, to be copied later into container[]

    /* replace an empty or expired object if found */
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].addr == 0) {
        Container[i] = *fop;
        return;
      }
    }
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (timenow - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
        Container[i] = *fop;
        return;
      }
    }

    /* may need to replace a non-expired object:   */
    /* identify the least important current object */

    uint32_t follow_id = settings->follow_id;

    /* replace an object of lower alarm level if found */

    if (fop->alarm_level > ALARM_LEVEL_NONE) {
      int min_level_ndx = 0;
      int min_level = fop->alarm_level;
      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].alarm_level < min_level) {
          min_level_ndx = i;
          min_level = Container[i].alarm_level;
        } else if (Container[min_level_ndx].addr == follow_id
                     && min_level < fop->alarm_level
                     && Container[i].alarm_level == min_level) {
          min_level_ndx = i;
        }
      }
      if (min_level < fop->alarm_level) {
          Container[min_level_ndx] = *fop;
          return;
      }
    }

    /* identify the farthest-away non-"followed" object */
    /*    (distance adjusted for altitude difference)   */

    int max_dist_ndx = MAX_TRACKING_OBJECTS;
    float max_adj_dist = 0;
    float adj_distance;
    for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
      if (Container[i].alarm_level == ALARM_LEVEL_NONE
          && Container[i].addr != follow_id) {
        adj_distance = Container[i].distance;
        if (Container[i].adj_distance > adj_distance)
              adj_distance = Container[i].adj_distance;
        if (adj_distance > max_adj_dist)  {
          max_dist_ndx = i;
          max_adj_dist = adj_distance;
        }
      }
    }

    /* replace the farthest currently-tracked object, */
    /* but only if the new object is closer (or "followed", or relayed) */
    adj_distance = fop->adj_distance;
    if (max_dist_ndx < MAX_TRACKING_OBJECTS
      && (adj_distance < max_adj_dist
          || fop->addr == follow_id
          || (do_relay && fop->timerelayed > 0))) {
      Container[max_dist_ndx] = *fop;
      return;
    }

    /* otherwise, no slot found, ignore the new object */
}

void ParseData(void)
{
    uint8_t rf_protocol = settings->rf_protocol;
    size_t rx_size = RF_Payload_Size(rf_protocol);
    rx_size = rx_size > sizeof(fo_raw) ? sizeof(fo_raw) : rx_size;

    if (memcmp(RxBuffer, TxBuffer, rx_size) == 0) {
      if (settings->nmea_p) {
        StdOut.println(F("$PSRFE,RF loopback is detected"));
      }
      return;
    }

    memcpy(fo_raw, RxBuffer, rx_size);
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now()); StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo_raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    fo = EmptyFO;  /* to ensure no data from past packets remains in any field */

    if (protocol_decode == NULL)
        return;

    if (((*protocol_decode)((void *) fo_raw, &ThisAircraft, &fo)) == false)
        return;

    fo.rssi = RF_last_rssi;

    AddTraffic(&fo);
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
    if (! isTimeToUpdateTraffic())
        return;

    ufo_t *mfop = NULL;
    max_alarm_level = ALARM_LEVEL_NONE;          /* global, used for visual displays */
    alarm_ahead = false;                         /* global, used for strobe pattern */
    relay_waiting = false;
    int sound_alarm_level = ALARM_LEVEL_NONE;    /* local, used for sound alerts */
    int alarmcount = 0;

    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      ufo_t *fop = &Container[i];

      if (fop->addr) {  /* non-empty ufo */
      
        if (ThisAircraft.timestamp - fop->timestamp <= ENTRY_EXPIRATION_TIME) {

          if ((ThisAircraft.timestamp - fop->timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL)
              Traffic_Update(fop);
          /* else Traffic_Update(fop) was called last time a radio packet came in */

          /* determine the highest alarm level seen at the moment */
          if (fop->alarm_level > max_alarm_level)
              max_alarm_level = fop->alarm_level;

          /* determine if any traffic with alarm level low+ is "ahead" */
          /* - this is for the strobe, increase flashing if "ahead" */
          if (fop->alarm_level >= ALARM_LEVEL_LOW) {
              if (abs(fop->RelativeBearing) < 45)
                  alarm_ahead = true;
          }

          /* figure out what is the highest alarm level needing a sound alert */
          if (fop->alarm_level > fop->alert_level
                   && fop->alarm_level > ALARM_LEVEL_CLOSE) {
              ++alarmcount;
              if (fop->alarm_level > sound_alarm_level) {
                  sound_alarm_level = fop->alarm_level;
                  mfop = fop;
              }
          }

        } else {   /* expired ufo */

          fop->addr = 0;

          //*fop = EmptyFO;

          /* implied by emptyFO:
          fop->addr = 0;
          fop->alert = 0;
          fop->alarm_level = 0;
          fop->alert_level = 0;
          fop->prevtime_ms = 0;
          etc... */
        }
      }
    }

    /* sound an alarm if new alert, or got two levels closer than previous  */
    /* alert, or hysteresis: got two levels farther, and then closer.       */
    /* E.g., if alarm was for LOW, alert_level was set to IMPORTANT.        */
    /* A new alarm alert will sound if close enough to now be URGENT.       */
    /* Or, if now gone to CLOSE (farther than LOW), set alert_level to LOW, */
    /* then next time reaches alarm_level IMPORTANT will give a new alert.  */
    /* Or, if now gone to NONE (farther than CLOSE), set alert_level to     */
    /* CLOSE, then next time returns to alarm_level LOW will give an alert. */

    if (sound_alarm_level > ALARM_LEVEL_CLOSE) {
      // use alarmcount to modify the sounds
      bool notified = Buzzer_Notify(sound_alarm_level, (alarmcount > 1));
#if !defined(EXCLUDE_VOICE)
#if defined(ESP32)
      if (mfop != NULL)
        notified |= Voice_Notify(mfop, (alarmcount > 1));
#endif
#endif
      /* Doing this here means that alarms via $PSRAA follow the same
       * hysteresis algorithm as the sound alarms.
       * External devices (XCsoar) sounding alarms based on $PFLAU have
       * their own algorithms, are not affected and may be continuous */
      if (settings->nmea_l || settings->nmea2_l) {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRAA,%d*"), sound_alarm_level-1);
          NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer)-10);
          NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, strlen(NMEABuffer), false);
      }
      if (mfop != NULL) {
        // if (notified)
        {
          mfop->alert_level = mfop->alarm_level + 1;
          mfop->alert |= TRAFFIC_ALERT_SOUND;  /* not actually used for anything */
        }
#if defined(ESP32)
        if (settings->logalarms && AlarmLogOpen) {
          int year  = gnss.date.year();
          if( year > 99)  year = year - 2000;
          int month = gnss.date.month();
          int day   = gnss.date.day();
          // $GPGGA,235317.00,4003.90395,N,10512.57934,W,...
          char *cp = &GPGGA_Copy[7];   // after the "$GPGGA,", start of timestamp
          GPGGA_Copy[43] = '\0';       // overwrite the comma after the "E" or "W"
Serial.print("GGA timestamp: ");
Serial.println(cp);
          int rel_bearing = (int) (mfop->bearing - ThisAircraft.course);
          rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("%02d%02d%02d,%s,%d,%d,%06x,%d,%d,%d\r\n"),
              year, month, day, cp, mfop->alarm_level-1, alarmcount,
              mfop->addr, rel_bearing, (int)mfop->distance, (int)mfop->alt_diff);
Serial.println(NMEABuffer);
          int len = strlen(NMEABuffer);
          if (AlarmLog.write((const uint8_t *)NMEABuffer, len) == len) {
              AlarmLog.flush();
          } else {
              // perhaps out of space in SPIFFS
              AlarmLog.close();
              AlarmLogOpen = false;
          }
        }
#endif
      }
    }

    UpdateTrafficTimeMarker = millis();
}

// currently this function is not called from anywhere (in "normal" mode)
//   - instead expired entries are purged in Traffic_loop()
void ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr &&
         (ThisAircraft.timestamp - Container[i].timestamp) > ENTRY_EXPIRATION_TIME) {
      //Container[i] = EmptyFO;
      Container[i].addr = 0;
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

/* called (as needed) from softRF.ino normal(), or from ParseData() above, */
/*   or every few minutes from Estimate_Wind() if ADDR_TYPE_RANDOM         */
void generate_random_id()
{
    uint32_t id = millis();
    id = (id ^ (id<<5) ^ (id>>5)) & 0x000FFFFF;
    if (settings->id_method == ADDR_TYPE_RANDOM)
      id |= 0x00E00000;
    else
      id |= 0x00F00000;
    ThisAircraft.addr = id;
}
