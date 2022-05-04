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
#include "driver/Sound.h"
#include "ui/Web.h"
#include "protocol/radio/Legacy.h"
#include "protocol/data/NMEA.h"
#include "ApproxMath.h"
#include "Wind.h"

unsigned long UpdateTrafficTimeMarker = 0;

ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
uint8_t fo_raw[34];
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
  if (circling != 0 || fabs(this_aircraft->turnrate) > 3.0 || fabs(fop->turnrate) > 3.0)
        return Alarm_Distance(this_aircraft, fop);
  if (fop->turnrate == 0.0) {   /* fop->turnrate not available */
    float angle = fabs(fop->course - fop->prevcourse);
    if (angle > 270.0)  angle = 360.0 - angle;
    if (angle > 6.0)
      return Alarm_Distance(this_aircraft, fop);
  }


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

      if (rel_angle < ALARM_VECTOR_ANGLE) {

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
  if (settings->nmea_d && (settings->debug_flags & DEBUG_ALARM)) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSALV,%06X,%ld,%d,%.1f,%.1f,%.1f,%.1f,%.5f,%.5f,%.1f,%.1f,%.1f,%.5f,%.5f,%.1f,%.1f,%.1f\r\n"),
      fop->addr, fop->gnsstime_ms, rval, V_rel_magnitude, V_rel_direction, fop->bearing, t,
      this_aircraft->latitude, this_aircraft->longitude, this_aircraft->altitude,
         this_aircraft->speed, this_aircraft->course,
      fop->latitude, fop->longitude, fop->altitude, fop->speed, fop->course);
    NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);
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
  int8_t rval = ALARM_LEVEL_NONE;

// this is handled in project():
//  if (this_aircraft->prevtime_ms == 0 || fop->prevtime_ms == 0) {
//    /* no history to compute with */
//    return Alarm_Distance(this_aircraft, fop);
//  }

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

  /* here start the expensive calculations */

  /* calculate other aircraft's turn rate and project course into future time points */
  project_that(fop);

  /* Also compute such estimates for ThisAircraft  - both relative to air not ground */
  /* project_this(this_aircraft); */
  /* - was already called from Estimate_Wind() or Legacy_Encode() */

  if (fabs(this_aircraft->turnrate) < 2 && fabs(fop->turnrate) < 2) {
    /* neither aircraft is circling */
    return (Alarm_Vector(this_aircraft, fop));
  }

  /* Project relative position second by second into the future */
  /* Time points in the ns/ew array appear to be current time + 1.5, 4.5, 7.5, 10.5 seconds */
  /* Use integer math for computational speed */

  /* also take altitude difference into account */
  int dz = abs((int) fop->alt_diff);
  /* reduce altitude separation to account for possible 100m GPS altitude error */
  int adjdz = (dz > 100) ? (dz - 100) : 0;
  /* may want to adjust dz over time? */
  /* - but is relative vs representative of future? */
  /* - in same thermal average relative vs = 0 */
  if (adjdz > 100)
    return (ALARM_LEVEL_NONE);

  /* prepare the second-by-second velocity vectors */
  static int thisvx[20];
  static int thisvy[20];
  static int thatvx[20];
  static int thatvy[20];
  int i, j;
  int vx, vy;
  int *px = thisvx;
  int *py = thisvy;
  /* considered interpolating between the 4 points, but does not seem useful */
  for (i=0; i<6; i++) {
    vx = this_aircraft->air_ew[i];  /* quarter-meters per second */
    vy = this_aircraft->air_ns[i];
    for (j=0; j<3; j++) {
      *px++ = vx;
      *py++ = vy;    
    }
  }
#if 0
  /* could have extrapolated "points" past [3] without trig: */
  float speed = approxHypotenuse((float)this_aircraft->ns[0], (float)this_aircraft->ew[0]);
  float spdchg = approxHypotenuse((float)(this_aircraft->ns[1] - this_aircraft->ns[0]),
                                   (float)(this_aircraft->ew[1] - this_aircraft->ew[0]));
  float factor = spdchg / speed;
  int ifactor = (int) (256.0 * (2.0 - factor * factor));
  vx = this_aircraft->ew[4-4]
       + ((ifactor * (this_aircraft->ew[4-1] - this_aircraft->ew[4-3])) >> 8);
  vy = this_aircraft->ns[4-4]
       + ((ifactor * (this_aircraft->ns[4-1] - this_aircraft->ns[4-3])) >> 8);
  for (j=0; j<3; j++) {
    *px++ = vx;
    *py++ = vy;
  }
  vx = this_aircraft->ew[5-4]
       + ((ifactor * (vx - this_aircraft->ew[5-3])) >> 8);
  vy = this_aircraft->ns[5-4]
       + ((ifactor * (vy - this_aircraft->ns[5-3])) >> 8);
  for (j=0; j<3; j++) {
    *px++ = vx;
    *py++ = vy;
  }
#endif
  *px = vx;  /* extrapolate one more second */
  *py = vy;

  /* same for the other aircraft */
  px = thatvx;
  py = thatvy;
  for (i=0; i<6; i++) {
    vx = fop->air_ew[i];
    vy = fop->air_ns[i];
    for (j=0; j<3; j++) {
      *px++ = vx;
      *py++ = vy;    
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
  uint32_t minsqdist = 100*100*4*4;
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
      if (mintime < ALARM_TIME_EXTREME)
          --fop->alert_level;     /* may sound new alarm even if previous one was IMPORTANT */
    }
    ++i;
    ++j;
  }

  /* try and set thresholds for alarms with gaggles in mind */
  /* squeezed between size of thermal and accuracy of prediction */
  uint32_t sqspeed = 0;
  if (minsqdist < 100*100*4*4)
    sqspeed = vxmin*vxmin + vymin*vymin;    /* relative speed at closest point, squared */
  if (minsqdist < 60*60*4*4) {   /* 60 meters 3D separation */
        if (mintime < ALARM_TIME_URGENT) {
          rval = ALARM_LEVEL_URGENT;
        } else if (mintime < ALARM_TIME_IMPORTANT) {
          rval = ALARM_LEVEL_IMPORTANT;
        } else {  /* min-dist time is at most 18 seconds */
          rval = ALARM_LEVEL_LOW;
        }
        if (sqspeed < 8*8*4*4)  /* relative speed < 8 mps */
           --rval;
        if (sqspeed < 2*2*4*4)  /* relative speed < 2 mps */
           --rval;
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
        if (sqspeed < 4*4*4*4)  /* relative speed < 4 mps */
           --rval;
  }

  /* send data out via NMEA for debugging */
  if (rval > ALARM_LEVEL_CLOSE || fop->distance < ALARM_ZONE_IMPORTANT) {
  if (settings->nmea_d && (settings->debug_flags & DEBUG_ALARM)) {
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
      PSTR("$PSALL,%06X,%ld,%ld,%d,%d,%d,%d,%.1f,%.1f,%.1f,%ld,%ld,%.1f,%.1f,%.1f,%.1f\r\n"),
        fop->addr, fop->projtime_ms, this_aircraft->projtime_ms, rval, mintime, minsqdist, sqspeed,
        this_aircraft->speed, this_aircraft->heading, this_aircraft->turnrate,
        fop->dy, fop->dx, fop->alt_diff, fop->speed, fop->heading, fop->turnrate);
    NMEA_Out(settings->nmea_out, (byte *) NMEABuffer, strlen(NMEABuffer), false);
  }
  }

  return rval;
}

void Traffic_Update(ufo_t *fop)
{
  /* use an approximation for distance & bearing between 2 points */
  float x, y;
  y = 111300.0 * (fop->latitude - ThisAircraft.latitude);         /* meters */
  x = 111300.0 * (fop->longitude - ThisAircraft.longitude) * CosLat(ThisAircraft.latitude);
  fop->dx = (int32_t) x;
  fop->dy = (int32_t) y;
  fop->distance = approxHypotenuse(x, y);      /* meters  */
  fop->bearing = atan2_approx(y, x);           /* degrees from ThisAircraft to fop */

  fop->alt_diff = fop->altitude - ThisAircraft.altitude;

  /* take altitude (and vert speed) differences into account as adjusted distance */
  float adj_alt_diff = Adj_alt_diff(&ThisAircraft, fop);
  fop->adj_alt_diff = adj_alt_diff;
  fop->adj_distance = fop->distance + VERTICAL_SLOPE * fabs(adj_alt_diff);

  /* follow FLARM docs: do not issue alarms about non-airborne traffic */
  if (fop->protocol == RF_PROTOCOL_LEGACY && fop->airborne == 0) {
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

void ParseData(void)
{
    size_t rx_size = RF_Payload_Size(settings->rf_protocol);
    rx_size = rx_size > sizeof(fo_raw) ? sizeof(fo_raw) : rx_size;

    if (memcmp(RxBuffer, TxBuffer, rx_size) == 0) {
      if (settings->nmea_p) {
        StdOut.println(F("$PSRFE,RF loopback is detected"));
      }
      return;
    }

    /* memset(fo_raw, 0, sizeof(fo_raw)); */
    memcpy(fo_raw, RxBuffer, rx_size);
    if (settings->nmea_p) {
      StdOut.print(F("$PSRFI,"));
      StdOut.print((unsigned long) now()); StdOut.print(F(","));
      StdOut.print(Bin2Hex(fo_raw, rx_size)); StdOut.print(F(","));
      StdOut.println(RF_last_rssi);
    }

    fo = EmptyFO;  /* to ensure no data from past packets remains in any field */

    if (protocol_decode && (*protocol_decode)((void *) fo_raw, &ThisAircraft, &fo)) {

      if (fo.addr == settings->ignore_id) {        /* ID told in settings to ignore */
             return;
      } else if (fo.addr == ThisAircraft.addr) {
             /* received same ID as this aircraft, and not told to ignore it */
             /* then replace this aircraft ID with a random one */
             settings->id_method = ADDR_TYPE_ANONYMOUS;
             generate_random_id();
             return;
      }

      fo.rssi = RF_last_rssi;

      int i;
      ufo_t *cip;

      /* first check whether we are already tracking this object */
      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {

        cip = &Container[i];

        if (cip->addr == fo.addr) {

          /* ignore "new" GPS fixes that are exactly the same as before */
          if ( /* fo.timestamp == Container[i].timestamp && */
              fo.altitude == cip->altitude &&
              fo.latitude == cip->latitude &&
              fo.longitude == cip->longitude)
                        return;

          /* overwrite old entry, but preserve fields that store history */
          /*   - they are copied into fo and then back into Container[i] */

          if ((fo.gnsstime_ms - cip->gnsstime_ms > 1200)
            /* packets spaced far enough apart, store new history */
          || (fo.gnsstime_ms - cip->prevtime_ms > 2600)) {
            /* previous history getting too old, drop it */
            /* this means using the past data from < 1200 ms ago */
            /* to avoid that would need to store data from yet another time point */
            fo.prevtime_ms  = cip->gnsstime_ms;
            fo.prevcourse   = cip->course;
            fo.prevheading  = cip->heading;
            /* fo.prevspeed = Container[i].speed; */
            fo.prevaltitude = cip->altitude;
          } else {
            /* retain the older history for now */
            fo.prevtime_ms  = cip->prevtime_ms;
            fo.prevcourse   = cip->prevcourse;
            fo.prevheading  = cip->prevheading;
            /* fo.prevspeed = cip->prevspeed; */
            fo.prevaltitude = cip->prevaltitude;
            /* >>> may want to also retain info needed to compute velocity vector at t=0 */
          }
          fo.turnrate    = cip->turnrate;   /* keep the old turn rate */
          fo.alert       = cip->alert;
          fo.alert_level = cip->alert_level;
       /* fo.callsign    = cip->callsign; */

          *cip = fo;   /* copies the whole object/structure */

          /* Now old alert_level is in same structure, can update alarm_level:  */
          Traffic_Update(cip);

          return;
        }
      }

      /* new object, try and find a slot for it */

      /* first get distance and alarm_level */
      Traffic_Update(&fo);

      /* replace an empty or expired object if found */
      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].addr == 0) {
          Container[i] = fo;
          return;
        }
      }
      time_t timenow = now();
      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (timenow - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
          Container[i] = fo;
          return;
        }
      }

      /* may need to replace a non-expired object:   */
      /* identify the least important current object */

      uint32_t follow_id = settings->follow_id;

      /* replace an object of lower alarm level if found */

      if (fo.alarm_level > ALARM_LEVEL_NONE) {
        int min_level_ndx = 0;
        int min_level = fo.alarm_level;
        for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
          if (Container[i].alarm_level < min_level) {
            min_level_ndx = i;
            min_level = Container[i].alarm_level;
          } else if (Container[min_level_ndx].addr == follow_id
                       && min_level < fo.alarm_level
                       && Container[i].alarm_level == min_level) {
            min_level_ndx = i;
          }
        }
        if (min_level < fo.alarm_level) {
            Container[min_level_ndx] = fo;
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
      /* but only if the new object is closer (or "followed") */
      adj_distance = fo.adj_distance;
      if (max_dist_ndx < MAX_TRACKING_OBJECTS
        && (adj_distance < max_adj_dist || fo.addr == follow_id)) {
        Container[max_dist_ndx] = fo;
        return;
      }

       /* otherwise, no slot found, ignore the new object */
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

    ufo_t *mfop = NULL;
    int max_alarm_level = ALARM_LEVEL_NONE;
        
    for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

      ufo_t *fop = &Container[i];

      if (fop->addr) {  /* non-empty ufo */
      
        if (ThisAircraft.timestamp - fop->timestamp <= ENTRY_EXPIRATION_TIME) {

          if ((ThisAircraft.timestamp - fop->timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL)
              Traffic_Update(fop);
          /* else Traffic_Update(fop) was called last time a radio packet came in */

          /* figure out what is the highest alarm level needing a sound alert */
          if (fop->alarm_level > fop->alert_level
                   && fop->alarm_level > ALARM_LEVEL_CLOSE) {
              if (fop->alarm_level > max_alarm_level) {
                  max_alarm_level = fop->alarm_level;
                  mfop = fop;
              }
          }

        } else {   /* expired ufo */

          *fop = EmptyFO;
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

    if (max_alarm_level > ALARM_LEVEL_CLOSE) {
      Sound_Notify(max_alarm_level);
      if (mfop != NULL) {
        mfop->alert_level = mfop->alarm_level + 1;
        mfop->alert |= TRAFFIC_ALERT_SOUND;  /* not actually used for anything */
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
