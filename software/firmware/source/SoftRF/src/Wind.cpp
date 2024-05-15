/*
 * Wind.cpp
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

#include "TrafficHelper.h"
#include "protocol/radio/Legacy.h"
#include "protocol/data/NMEA.h"
#include "ApproxMath.h"
#include "Wind.h"
#include "driver/EEPROM.h"
#include "driver/GNSS.h"
#include "driver/RF.h"


float wind_best_ns = 0.0;  /* mps */
float wind_best_ew = 0.0;
float airspeed = 0.0;
float wind_speed = 0.0;
float wind_direction = 0.0;
float avg_turnrate = 0.0;
float avg_speed = 0.0;     /* average around the circle */
float avg_climbrate = 0.0; /* fpm, based on GNSS data */


void Estimate_Wind()
{
  static float old_time = 0.0;
  static float old_lat = 0.0;     /* where a circle started, on its Northern edge */
  static float old_lon = 0.0;     /* where a circle started, on its Eastern edge */
  static uint32_t start_time = 0.0;
  static uint32_t old_lat_time = 0;    /* when those circles started */
  static uint32_t old_lon_time = 0;
  static float old_turnrate = 0.0;
  static float old_course = 0.0;
  static float cumul_turn = 0.0;  /* cumulative change of direction around circle */
  static int oldquadrant = 0;
  static float min_gs = 999.0;    /* min and max ground speed around circle */
  static float max_gs = 0.0;
  static float min_gs_course = 0.0;   /* directions of those min and max ground speeds */
  static float max_gs_course = 0.0;
  static float prev_gs_ns, prev_gs_ew, prev_cd_ns, prev_cd_ew;
  static float weight_gs, weight_cd;
  static uint32_t decaytime = 0;

  bool ok = false;
  bool ns = false;
  bool ew = false;

  int32_t new_time = ThisAircraft.gnsstime_ms;

  /* ignore repeats of same GPS reading */
  if (new_time == old_time)
    return;

  /* recover from GPS outage */
  if (new_time - old_time > 3000) {
     old_time = new_time;
     ThisAircraft.circling = 0;
     old_turnrate = 0.0;
     avg_turnrate = 0.0;
     return;
  }

  old_time = new_time;

  project_this(&ThisAircraft);
  float turnrate = ThisAircraft.turnrate;  /* was computed in project_this() */
  float course_change, interval;

  if (ThisAircraft.circling == 0) { /* not considered in a stable circle */

    /* slowly decay wind towards zero over time */
    /* with a half-life of about 20 minutes     */
    /* - but new circling will refresh it       */
    /* piggy-back random time for new random ID */
    if (new_time > decaytime) {
        decaytime = new_time + 100000;
        wind_best_ns *= 0.95;
        wind_best_ew *= 0.95;
        wind_speed  *= 0.95;
        if (settings->id_method == ADDR_TYPE_RANDOM)
             generate_random_id();
    }

    if (old_turnrate == 0.0) {  /* was not turning */
      if (fabs(turnrate) > 5.0 && fabs(turnrate) < 40.0) {  /* now turning */
        /* start watching whether stable */
        old_turnrate = turnrate;
        old_course = ThisAircraft.course;
        cumul_turn = 0.0;
      }
      return;
    }

    if ((old_turnrate > 0.0 && turnrate < 0.0)
     || (old_turnrate < 0.0 && turnrate > 0.0)) {
      /* not a consistent turn */ 
      old_turnrate = 0.0;
      return;
    }

    course_change = ThisAircraft.course - old_course;
    if (course_change > 300.0)   course_change -= 360.0;   /* passed through North */
    if (course_change < -300.0)  course_change += 360.0;
    cumul_turn += course_change;
    old_course = ThisAircraft.course;
    if (fabs(cumul_turn) > 180.0) {  /* completed a half-turn */
       /* declare state to be "circling", and set up to measure wind */
       ThisAircraft.circling = (turnrate > 0.0? 1 : -1);
       start_time = new_time;
       cumul_turn = 0.0;
       oldquadrant = 0;
       old_lat_time = 0;
       old_lon_time = 0;
       prev_gs_ns = prev_cd_ns = wind_best_ns;
       prev_gs_ew = prev_cd_ew = wind_best_ew;
       weight_gs = 0.04;
       weight_cd = 0.03;  /* weights will increase or decrease later */
    }

    old_turnrate = turnrate;
    return;

  }   /* end if (circling == 0) */

  /* got here if circling != 0, check whether still circling */

  if ((ThisAircraft.circling > 0 && turnrate < -2.0)
   || (ThisAircraft.circling < 0 && turnrate >  2.0)) {
     /* no longer a consistent turn */ 
     ThisAircraft.circling = 0;
     old_turnrate = 0.0;
     avg_turnrate = 0.0;
     decaytime = new_time + 100000;
     return;
  }

  /* watch for changes in ground speed */

  if (ThisAircraft.speed > max_gs) {
    max_gs = ThisAircraft.speed;
    max_gs_course = ThisAircraft.course;
  }
  if (ThisAircraft.speed < min_gs) {
    min_gs = ThisAircraft.speed;
    min_gs_course = ThisAircraft.course;
  }

  /* note when a whole circle is done */

  course_change = ThisAircraft.course - old_course;
  if (course_change > 300.0)   course_change -= 360.0;   /* passed through North */
  if (course_change < -300.0)  course_change += 360.0;
  cumul_turn += course_change;
  old_course = ThisAircraft.course;

  float wind_ns, wind_ew;

  if (fabs(cumul_turn) > 360.0) {  /* completed a circle */

       float direction, windspeed;

       turnrate = 360000.0 * (float) ThisAircraft.circling / (float) (new_time - start_time);
       if (fabs(turnrate) > 50.0)  turnrate = avg_turnrate;   /* ignore implausible data */
       if (fabs(turnrate) <  2.0)  turnrate = 0.0;            /* ignore inaccurate data */
       if (avg_turnrate == 0.0)
           avg_turnrate = turnrate;
       else
           avg_turnrate = 0.8 * avg_turnrate + 0.2 * turnrate;

      /* use ground speed observations to estimate wind */

       min_gs_course += 180.0;  /* point downwind */
       if (min_gs_course > 360.0)  min_gs_course -= 360.0;
       if (fabs(min_gs_course-max_gs_course) < 180.0) {       /* they do not straddle North */
         if (fabs(min_gs_course - max_gs_course) < 30.0) {    /* they are roughly the same */
           direction = 0.5 * (min_gs_course + max_gs_course);
           ok = true;
         }
       } else {                                               /* they do straddle North */
         if (min_gs_course > 270.0) {                         /* min_gs_course is W of N */
           if (fabs(360.0 - min_gs_course + max_gs_course) < 30.0)
             ok = true;
         } else if (max_gs_course > 270.0) {                  /* max_gs_course is W of N */
           if (fabs(360.0 - max_gs_course + min_gs_course) < 30.0)
             ok = true;
         }
         direction = 0.5 * (min_gs_course + max_gs_course - 360.0);
         if (direction < 0.0)  direction += 360.0;
       }
       if (ok) {
         windspeed = 0.5 * (max_gs - min_gs) * _GPS_MPS_PER_KNOT;
         if (airspeed == 0.0)
             airspeed = 0.5 * (max_gs + min_gs) * _GPS_MPS_PER_KNOT;
         else
             airspeed = (1.0 - weight_gs) * airspeed
                         + weight_gs * 0.5 * (max_gs + min_gs) * _GPS_MPS_PER_KNOT;
       }
       if (ok && windspeed > 1.0) {   /* ignore wind estimate < 1 mps */
         wind_ns = windspeed * cos_approx(direction);
         wind_ew = windspeed * sin_approx(direction);
         if (windspeed < 40.0
         &&  fabs(wind_ns-wind_best_ns) < 20.0
         &&  fabs(wind_ew-wind_best_ew) < 20.0) {            /* ignore implausible values */
           if (wind_best_ns == 0.0)  /* not initialized yet */
             wind_best_ns = wind_ns;
           else
             wind_best_ns = (1.0 - weight_gs) * wind_best_ns + weight_gs * wind_ns;
             /* only gradually change "best" estimate */
           if (wind_best_ew == 0.0)
             wind_best_ew = wind_ew;
           else
             wind_best_ew = (1.0 - weight_gs) * wind_best_ew + weight_gs * wind_ew;
           if (avg_speed == 0.0)
             avg_speed = 0.5 * (min_gs + max_gs);            /* = average AIRspeed */
           else
             avg_speed = 0.7 * avg_speed + 0.3 * 0.5*(min_gs+max_gs);
             /* this is retained over time and changed gradually */
           // wind_speed = approxHypotenuse(wind_best_ns, wind_best_ew);
         }
         /* give more weight to subsequent circles in same thermal */
         /* conversely if estimates are noisy reduce the weight    */
         if ((fabs(wind_best_ns) > 2.5 && fabs(wind_ns-prev_gs_ns) > 0.5*fabs(wind_best_ns))
          || (fabs(wind_best_ew) > 2.5 && fabs(wind_ew-prev_gs_ew) > 0.5*fabs(wind_best_ew))) {
            if (weight_gs > 0.03)  weight_gs -= 0.02;
         } else {
            if (weight_gs < 0.09)  weight_gs += 0.02;
         }
         prev_gs_ns = wind_ns;
         prev_gs_ew = wind_ew;
       }

       cumul_turn = 0.0;     /* set up to observe the next circle */
       start_time = new_time;
       min_gs = 999.0;
       max_gs = 0.0;       

  }   /* done with GS around circle */


  /* also use drift while circling to estimate wind */

  /* classify direction into 4 quadrants */

  int icourse = (int) (ThisAircraft.course + 0.5);
  if (icourse < 0)  icourse += 360;
  int quadrant = 1;
  if (icourse > 90) {
    ++quadrant;
    if (icourse > 180) {
      ++quadrant;
      if (icourse > 270)
        ++quadrant;
    }
  }

  /* detect completion of NS/EW-oriented circles */

  if (ThisAircraft.circling > 0) {          /* clockwise */
    if (quadrant == 2 && oldquadrant == 1)  /* traveling East, on the North side of the circle */
      ns = true;
    if (quadrant == 3 && oldquadrant == 2)  /* traveling South, on the East side of the circle */
      ew = true;
  } else if (ThisAircraft.circling < 0) {   /* counter-clockwise */
    if (quadrant == 3 && oldquadrant == 4)  /* traveling West, on the North side of the circle */
      ns = true;
    if (quadrant == 4 && oldquadrant == 1)  /* traveling North, on the East side of the circle */
      ew = true;
  }

  oldquadrant = quadrant;  /* set up to detect the next quadrant crossing */

  /* measure drift in completed circles */

  float drift_ns;
  if (ns) {  /* started and perhaps completed a circle */
    float new_lat = ThisAircraft.latitude;
    if (old_lat_time != 0) {  /* there is history to use */
      drift_ns = 111300.0 * (new_lat - old_lat);   /* how far further North, in meters */
      if (abs(drift_ns) > 300)  drift_ns = 0;      /* ignore implausible values */
      interval = 0.001 * (new_time - old_lat_time);
      wind_ns = drift_ns / interval;               /* m/s */
      if (fabs(wind_ns-wind_best_ns) < 20.0 && fabs(wind_ns-wind_best_ns) > 1.0) {
        if (wind_best_ns == 0.0)  /* not initialized yet */
          wind_best_ns = wind_ns;
        else if (wind_ns != 0.0)
          wind_best_ns = (1.0 - weight_cd) * wind_best_ns + weight_cd * wind_ns;
      }
      if (fabs(wind_best_ns) > 2.5 && fabs(wind_ns-prev_cd_ns) > 0.5*fabs(wind_best_ns)) {
        if (weight_cd > 0.02)  weight_cd -= 0.015;
      } else {
        if (weight_cd < 0.08)  weight_cd += 0.015;  /* it may get bumped up in EW section too */
      }
      prev_cd_ns = wind_ns;
    }
    old_lat = new_lat;  /* start observing a new circle */
    old_lat_time = new_time;
  }

  float drift_ew;
  if (ew) {
    float new_lon = ThisAircraft.longitude;
    if (old_lon_time != 0) {
      drift_ew = 111300.0 * (new_lon - old_lon) * CosLat(ThisAircraft.latitude); /* how far further East */
      if (abs(drift_ew) > 300)  drift_ew = 0;
      interval = 0.001 * (new_time - old_lon_time);
      wind_ew = drift_ew / interval;
      if (fabs(wind_ew-wind_best_ew) < 20.0 && fabs(wind_ew-wind_best_ew) > 1.0) {
        if (wind_best_ew == 0.0)
          wind_best_ew = wind_ew;
        else if (wind_ew != 0.0)
          wind_best_ew = (1.0 - weight_cd) * wind_best_ew + weight_cd * wind_ew;
      }
      if (fabs(wind_best_ew) > 2.5 && fabs(wind_ew-prev_cd_ew) > 0.5*fabs(wind_best_ew)) {
        if (weight_cd > 0.02)  weight_cd -= 0.015;
      } else {
        if (weight_cd < 0.08)  weight_cd += 0.015;
      }
      prev_cd_ew = wind_ew;
    }
    old_lon = new_lon;  /* start observing a new circle */
    old_lon_time = new_time;
  }

  if (ns || ew) {
    wind_speed = approxHypotenuse(wind_best_ns, wind_best_ew);
    wind_direction = atan2_approx(-wind_best_ns, -wind_best_ew);  /* direction coming FROM */
    turnrate = 360.0 * (float) ThisAircraft.circling / interval;
    if (fabs(turnrate) > 50.0)  turnrate = avg_turnrate;   /* ignore implausible data */
    if (fabs(turnrate) <  2.0)  turnrate = 0.0;            /* ignore inaccurate data */
    if (avg_turnrate == 0.0)
        avg_turnrate = turnrate;
    else
        avg_turnrate = 0.7 * avg_turnrate + 0.3 * turnrate;
  }

  /* send data out via NMEA for debugging */
  if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_WIND)) {
    if (ok) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWGS,%ld,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n"),
        new_time, ThisAircraft.speed, avg_speed, ThisAircraft.course,
        ThisAircraft.turnrate, avg_turnrate, min_gs_course, max_gs_course,
        weight_gs, wind_ns, wind_ew, wind_best_ns, wind_best_ew);
      NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
    }
    if (ns) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWNS,%ld,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n"),
        new_time, ThisAircraft.latitude, ThisAircraft.longitude,
        ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        interval, drift_ns, weight_cd, wind_ns, wind_best_ns, wind_best_ew);
      NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
    }
    if (ew) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWEW,%ld,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n"),
        new_time, ThisAircraft.latitude, ThisAircraft.longitude,
        ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        interval, drift_ew, weight_cd, wind_ew, wind_best_ns, wind_best_ew);
      NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSWSD,%.1f,%.0f\r\n"),
        wind_speed * (1.0 / _GPS_MPS_PER_KNOT), wind_direction);
      NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
    }
  }

}


/* keep track of whether this aircraft is airborne */
void this_airborne()
{
    /* static vars to keep track of 'airborne' status: */
    static int airborne = -4;
    static float prevspeed = 0;
    static float initial_latitude = 0;
    static float initial_longitude = 0;
    static float initial_altitude = 0;

    int was_airborne = airborne;
    float speed = ThisAircraft.speed;

    if (initial_latitude == 0) {
      /* wait for stable fix */
      if (! GNSSTimeMarker)
        return;
      /* set initial location */
      initial_latitude  = ThisAircraft.latitude;
      initial_longitude = ThisAircraft.longitude;
      initial_altitude  = ThisAircraft.altitude;
    }

    if (speed < 1.0) {

      if (airborne > 0) {
        --airborne;
        /* after 50 calls (~20 sec if consecutive)
           with speed < 1 knot consider it a landing */
      } else /* not airborne */ {
        /* if had some speed and then stopped - reset to -4 again */
        airborne = -4;
        initial_latitude = 0;
      }

    } else if (airborne < 0) {    /* not airborne but moving with speed > 1 knot */

      if (GNSSTimeMarker > 0 && ThisAircraft.prevtime_ms > 0) {  /* had fix for a while */

        if ( speed > 20.0                                               /* 20 knots  */
          || fabs(ThisAircraft.latitude - initial_latitude) > 0.0018f   /* about 200 meters */
          || fabs(ThisAircraft.longitude - initial_longitude) > 0.0027f
          || fabs(ThisAircraft.altitude - initial_altitude) > 120.0f) {
            /* movement larger than typical GNSS noise */
            float interval = 0.001 * fabs(ThisAircraft.gnsstime_ms - ThisAircraft.prevtime_ms);
            if (fabs(ThisAircraft.altitude - ThisAircraft.prevaltitude) > 20.0 * interval
             || fabs(ThisAircraft.course - ThisAircraft.prevcourse) > 50.0 * interval
             || speed > 4.0 * prevspeed || prevspeed > 4.0 * speed) {
               /* supposed initial movement is too jerky - wait for smoother changes */
            } else {
                ++airborne;
                /* require additional movements to call it airborne */
                initial_latitude  = ThisAircraft.latitude;
                initial_longitude = ThisAircraft.longitude;
                initial_altitude  = ThisAircraft.altitude;
            }
            prevspeed = speed;
            if (airborne > 0)    /* consistently good indications */
                 airborne = 50;  /* now really airborne */
        }
      }

    }

#if defined(ESP32)
    // restart alarm log on first takeoff after boot
    if (AlarmLogOpen==false) {
      if (settings->logalarms && ThisAircraft.airborne==0 && airborne>0) {
        if (SPIFFS.begin(true)) {
          bool append = false;
          if (SPIFFS.exists("/alarmlog.txt") && SPIFFS.totalBytes() - SPIFFS.usedBytes() > 10000)
              append = true;
          AlarmLog = SPIFFS.open("/alarmlog.txt", (append? FILE_APPEND : FILE_WRITE));
          if (AlarmLog) {
              AlarmLogOpen = true;
              if (append == false) {
                const char *p = "date,time,lat,lon,level,count,ID,relbrg,hdist,vdist\r\n";
                AlarmLog.write((const uint8_t *)p, strlen(p));
              }
          } else {
              Serial.println(F("Failed to open alarmlog.txt"));
          }
        } else {
            Serial.println(F("Failed to start SPIFFS"));
        }
      }

    // close the alarm log after landing
    } else if (ThisAircraft.airborne==1 && airborne<=0) {
        AlarmLog.close();
        AlarmLogOpen = false;
    }
#endif

    ThisAircraft.airborne = (airborne > 0)? 1 : 0;

    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
      if (airborne != was_airborne) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSTAA,this_airborne: %d, %.1f, %.5f, %.5f, %.1f\r\n"),
            airborne, speed, ThisAircraft.latitude, ThisAircraft.longitude, ThisAircraft.altitude);
        NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
      }
    }
}


void report_this_projection(ufo_t *this_aircraft, int proj_type)
{
#if 0
    if (proj_type==1)
      Serial.printf("this_proj: no history  %.1f %.1f %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0]);
    else if (proj_type==2)
      Serial.printf("this_proj: not turning %.1f %.1f %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0]);
    else if (proj_type==3)
      Serial.printf("this_proj: circling    %.1f %.1f %d %d %d %d %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->air_ns[1], this_aircraft->air_ew[1],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0],
        this_aircraft->fla_ns[1], this_aircraft->fla_ew[1]);
    else if (proj_type==4)
      Serial.printf("this_proj: history     %.1f %.1f %d %d %d %d %d %d %d %d\r\n",
        this_aircraft->course, this_aircraft->heading,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->air_ns[1], this_aircraft->air_ew[1],
        this_aircraft->fla_ns[0], this_aircraft->fla_ew[0],
        this_aircraft->fla_ns[1], this_aircraft->fla_ew[1]);
    else
      Serial.printf("this_proj: eh?\r\n");
#endif
    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("$PSPTA,%d,%d,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d\r\n"),
        this_aircraft->airborne,
        proj_type, this_aircraft->course, this_aircraft->heading,
        this_aircraft->prevheading, this_aircraft->turnrate, this_aircraft->circling,
        this_aircraft->air_ns[0], this_aircraft->air_ew[0],
        this_aircraft->air_ns[1], this_aircraft->air_ew[1]);
      NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
    }
}

void report_that_projection(ufo_t *fop, int proj_type)
{
#if 0
    Serial.printf("that_proj: %d %.1f %.1f %d %d %d %d %d %d\r\n",
          proj_type, fop->course, fop->heading,
          fop->air_ns[0], fop->air_ew[0],
          fop->air_ns[1], fop->air_ew[1],
          fop->air_ns[2], fop->air_ew[2]);
#endif
    if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_PROJECTION)) {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSPOA,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d\r\n"),
          proj_type, fop->course, fop->heading,
          fop->air_ns[0], fop->air_ew[0],
          fop->air_ns[1], fop->air_ew[1],
          fop->air_ns[2], fop->air_ew[2]);
        NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
    }
}


/*
 * Project the future path of this_aircraft into some future time points.
 */
void project_this(ufo_t *this_aircraft)
{
    int i, endturn;
    int16_t ns, ew;
    float gspeed, aspeed;
    float course, heading, turnrate, gs_ns, gs_ew, as_ns, as_ew;
    uint32_t interval;
    float c, s;
    bool report = false;
    int proj_type = 0;

    /* don't project this aircraft more often than every 400 ms */
    static uint32_t time_to_project_course = 0;
    if (this_aircraft->gnsstime_ms < time_to_project_course)
          return;
    time_to_project_course = this_aircraft->gnsstime_ms + 400;

    static uint32_t time_to_report = 0;
    if (this_aircraft->gnsstime_ms > time_to_report) {
        time_to_report = this_aircraft->gnsstime_ms + 2300;
        report = true;
    }

    this_airborne();      /* determine the "airborne" flag */

    /* first get heading and turn rate */

    /* ground reference speed and course from last GNSS fix */
    gspeed = this_aircraft->speed * _GPS_MPS_PER_KNOT;
    course = this_aircraft->course;
    gs_ns = gspeed * cos_approx(course);
    gs_ew = gspeed * sin_approx(course);

    /* compute airspeed and heading from course and speed and wind */
    as_ns = gs_ns - wind_best_ns;
    as_ew = gs_ew - wind_best_ew;
    heading = atan2_approx(as_ns, as_ew);
    this_aircraft->heading = heading;

    /* also compute ground-reference turn rate */
    interval = this_aircraft->gnsstime_ms - this_aircraft->prevtime_ms;
    this_aircraft->projtime_ms = this_aircraft->gnsstime_ms - (interval >> 1);
          /* midway between the 2 time points */
    turnrate = (course - this_aircraft->prevcourse) / (0.001 * (float) interval);
    if (fabs(turnrate) <  2.0)  turnrate = 0.0;
    if (fabs(turnrate) > 50.0)  turnrate = 0.0;
    if (interval < 1200) {
       /* short interval between packets, average with previously known turn rate */
       this_aircraft->turnrate = 0.5 * (turnrate + this_aircraft->turnrate);
    } else {
       this_aircraft->turnrate = turnrate;
    }

    /* if this aircraft is circling then use the average turn rate       */
    /* around the circle as measured within the wind estimation function */
    /* - this differs from the momentary turn rate if there is wind      */

    if (this_aircraft->circling != 0 && avg_turnrate != 0.0) {

      proj_type = 3;
      this_aircraft->projtime_ms = this_aircraft->gnsstime_ms;
      turnrate = avg_turnrate;
      //this_aircraft->aturnrate = turnrate;

      aspeed = airspeed;   /* average airspeed measured while circling */

      // fall through to computation of ground-reference turn rate

    } else if (this_aircraft->gnsstime_ms - this_aircraft->prevtime_ms > 3000) {

      /* if no usable history, assume a straight path */

      this_aircraft->projtime_ms = this_aircraft->gnsstime_ms;
      this_aircraft->prevcourse  = this_aircraft->course;
      this_aircraft->prevheading = this_aircraft->heading;
      //this_aircraft->aturnrate = 0.0;
      this_aircraft->turnrate = 0.0;

      //aspeed = approxHypotenuse(as_ns, as_ew);
      //this_aircraft->airspeed = aspeed;
      ns = (int16_t) roundf(4.0 * as_ns);
      ew = (int16_t) roundf(4.0 * as_ew);
      for (i=0; i<6; i++) {
        this_aircraft->air_ns[i] = ns;
        this_aircraft->air_ew[i] = ew;
      }

      if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
        /* also need to compute fla_ns[] & fla_ew[] for transmissions */
        ns = (int16_t) roundf(4.0 * gs_ns);
        ew = (int16_t) roundf(4.0 * gs_ew);
        for (i=0; i<4; i++) {
          this_aircraft->fla_ns[i] = ns;
          this_aircraft->fla_ew[i] = ew;
        }
      }

      if (report) report_this_projection(this_aircraft, 1);

      return;

    } else {  /* this aircraft not "circling" but has history */

      proj_type = 4;

      aspeed = approxHypotenuse(as_ns, as_ew);

      /* compute turn rate - degrees per second */
      /* uses current and previous course and time_ms */

      /* previous heading from past course and speed and wind */
      float prevheading = this_aircraft->prevheading;

      /* turn rate in the air reference frame (drifting with the wind) */

      float heading_change = heading - prevheading;
      if (fabs(heading_change) > 270.0) {
        /* roll-over through 360 */
        if (heading > 0.0)  heading_change -= 360.0;
        else heading_change += 360.0;
      }
      //interval = this_aircraft->gnsstime_ms - this_aircraft->prevtime_ms;
      turnrate = heading_change / (0.001 * (float) interval);
      if (fabs(turnrate) <  2.0)  turnrate = 0.0;        /* ignore inaccurate data */
      if (fabs(turnrate) > 50.0)  turnrate = 0.0;        /* ignore implausible data */
      //if (interval < 1200) {
         /* short interval between packets, average with previously known turn rate */
         //this_aircraft->aturnrate = 0.5 * (turnrate + this_aircraft->aturnrate);
      //} else {
         //this_aircraft->aturnrate = turnrate;
      //}
    }

    /* compute NS & EW speed components for future time points */

    if (fabs(turnrate) < 2.0) {

      /* treat it as not turning at all */
      ns = (int16_t) roundf(4.0 * as_ns);
      ew = (int16_t) roundf(4.0 * as_ew);
      for (i=0; i<6; i++) {
        this_aircraft->air_ns[i] = ns;
        this_aircraft->air_ew[i] = ew;
      }
      //this_aircraft->aturnrate = 0.0;
      this_aircraft->turnrate = 0.0;

      if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {
        /* also need to compute fla_ns[] & fla_ew[] for transmissions */
        ns = (int16_t) roundf(4.0 * gs_ns);
        ew = (int16_t) roundf(4.0 * gs_ew);
        for (i=0; i<4; i++) {
          this_aircraft->fla_ns[i] = ns;
          this_aircraft->fla_ew[i] = ew;
        }
      }
      //this_aircraft->airspeed = aspeed;

      if (report) report_this_projection(this_aircraft, 2);

      return;

    }
    
    /* turning */

    if (this_aircraft->projtime_ms > this_aircraft->gnsstime_ms)
      heading += turnrate * (float) (this_aircraft->projtime_ms - this_aircraft->gnsstime_ms);
    else
      heading -= turnrate * (float) (this_aircraft->gnsstime_ms - this_aircraft->projtime_ms);

    /* our internal intervals are 3 sec, even though transmissions may use 2 or 4 */

    if (fabs(turnrate) > 6.0) {
      /* since the projection is in straight segments rather than a circle, */
      /* correct the speed for the polygon shortcut relative to the circumference */
      /* so that the projected trajectory will reach the points at the right time */
      /* factor = 360/PI/turnrate/interval * sin_approx(turnrate*interval/2) */
      /* - 1/2 slice angle in degrees over interval=3seconds is turnrate * 1.5 */
      float factor = (360.0/3.0/3.1416)/turnrate * sin_approx(turnrate*1.5);
      if (factor < 0.86)  factor = 0.86;
      if (factor < 0.99)  aspeed *= factor;
    }
    //this_aircraft->airspeed = aspeed;

    float dir_chg = 1.5 * turnrate;  // average heading between now and the first point
    heading += dir_chg;              //   which will be 3 seconds into future
    dir_chg *= 2.0;                  // 3-second intervals after that
    if (this_aircraft->circling) {
        endturn = 6;
    } else if (fabs(dir_chg) > 15.0) {
        endturn = (int) (90.0 / fabs(dir_chg));    // limit to a 90-degree turn
        if (endturn == 0)  endturn = 1;
    } else {
        endturn = 6;
    }
    for (i=0; i<6; i++) {
       if (i < endturn) {
          if (heading >  360.0)  heading -= 360.0;
          if (heading < -360.0)  heading += 360.0;
          ns = (int16_t) roundf(4.0 * aspeed * cos_approx(heading));
          ew = (int16_t) roundf(4.0 * aspeed * sin_approx(heading));
          heading += dir_chg;
       }  // else stop turning, keep same velocity vector
       this_aircraft->air_ns[i] = ns;
       this_aircraft->air_ew[i] = ew;
    }

    if (settings->rf_protocol != RF_PROTOCOL_LEGACY) {
        if (report) report_this_projection(this_aircraft, proj_type);
        return;
    }

    // if (settings->rf_protocol == RF_PROTOCOL_LEGACY) {

    /* also need to compute fla_ns[] & fla_ew[] for transmissions */

    /* first need to compute initial ground-reference turn rate */
    // compute this (approximately) for now+2s instead of now:
    //float windangle = course + 2.0*turnrate - wind_direction;
    //if (windangle > 0)  windangle -= 180.0;
    //else                windangle += 180.0;     /* angle from DOWNwind */
    //turnrate = turnrate / (1.0 + cos_approx(windangle) * wind_speed / aspeed);
    /* this increases the turnrate when heading upwind, and vice versa */

    // alternative: use turnrate computed above from course and prevcourse

    /* now imitate FLARM and keep speed and turn rate constant while circling */
    float delta_t;
    if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_TOWPLANE)         // known 4-second intervals
        delta_t = 4.0;
    else if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_DROPPLANE)   // unverified assumptions
        delta_t = 4.0;
    else if (this_aircraft->aircraft_type == AIRCRAFT_TYPE_POWERED)
        delta_t = 4.0;
    else if (this_aircraft->circling)                  // typically gliders
        delta_t = 3.0;
    else
        delta_t = 2.0;
    dir_chg = delta_t * turnrate;
    if (this_aircraft->circling) {
        endturn = 4;
    } else if (fabs(dir_chg) > 22.5) {
        // >>> limit to a 90-degree turn
        //     - FLARM may or may not want this in the projection?
        endturn = (int) (90.0 / fabs(dir_chg));
        if (endturn == 0)  endturn = 1;
    } else {
        endturn = 4;
    }
    for (i=0; i<4; i++) {
      if (i < endturn) {
        // first velocity direction will be "delta_t" seconds into future
        //   - because that is what FLARM seems to send
        course += dir_chg;
        if (course >  360.0)  course -= 360.0;
        if (course < -360.0)  course += 360.0;
        ns = (int16_t) roundf(4.0 * gspeed * cos_approx(course));
        ew = (int16_t) roundf(4.0 * gspeed * sin_approx(course));
      } // else stop turning, keep same velocity vector
      this_aircraft->fla_ns[i] = ns;
      this_aircraft->fla_ew[i] = ew;
    }

    //}

    if (report) report_this_projection(this_aircraft, proj_type);
}


/*
 * Project the future path of other aircraft into some future time points.
 */
void project_that(ufo_t *fop)
{
    int i, endturn;
    float aspeed, gspeed, course, heading, aturnrate;
    float gs_ns, gs_ew, as_ns, as_ew, windangle;
    int16_t ns, ew;
    bool report = false;

    static uint32_t time_to_report = 0;
    if (millis() > time_to_report) {
        time_to_report = millis() + 2300;
        report = true;
    }

    if (fop->protocol == RF_PROTOCOL_LEGACY || fop->protocol == RF_PROTOCOL_LATEST) {

      /* Turn rate was obtained in Legacy_decode() */
      /* For vector alarm method don't need more than that */
      if (settings->alarm != TRAFFIC_ALARM_LEGACY)
            return;

      /* Incoming Legacy packets provide us with the velocity data.       */
      /* But it is not in a good format, neither air nor ground reference */
      /* The data in the packets ignores the wind!                        */
      /* It is Ground Speed at some point in time (presumably now)        */
      /* and does not change later in the projection (points [1]-[3]).    */
      /* The turn rate is given as constant but likely also ground-ref.   */
      /*            https://pastebin.com/raw/nmi71bTG                     */
      /*            https://pastebin.com/raw/Pi014La8                     */

/*
To compute the correct air-reference circling path:
* Get speed vector from the first ns/ew pair.
* Compute ground-ref course C from atan2() of the first ns/ew pair.
* Compute ground-ref course from atan2() of the second ns/ew pair.
* Compute the (ground reference) turn rate R from the difference.
* Subtract the wind vector from the first pair to get the airspeed vector.
* Rewind it to time "now", or about 1 or 1.5 seconds before the first projection point.
   - that gives the heading.
   - also compute the course "now", by adding the wind back in.
* Compute scalar magnitude of airspeed A and of wind W.
* Compute the angle D of the ground-ref course C relative to the wind vector
   - 0 degrees means directly downwind.
* Compute a correction E to the turn rate:  E = cos(D)*R*W/A
* Add E to the turn rate R to get the air-reference turn rate.
   - Downwind turn rates get bigger (in air reference), upwind smaller.
   - or do it as R *= (1+cos(D)*W/A)
* Use the initial heading and the corrected turn rate to project the 4 future points.
* this is a total of 13 trig calls!
* might as well continue and compute 2 more points? (can use the no-trig method)
*
      float direction0 = atan2_approx((float) fop->fla_ns[0], (float) fop->fla_ew[0]);
      float direction1 = atan2_approx((float) fop->fla_ns[1], (float) fop->fla_ew[1]);
      float dir_chg = direction1 - direction0;
      if (dir_chg >  270.0) dir_chg -= 360.0;
      if (dir_chg < -270.0) dir_chg += 360.0;
      // FLARMs send lat/lon of position 2, 3, or 4 seconds into future
      // compute present ground-reference direction
      float dir_now;
      if (fop->interval == 4)   // AIRCRAFT_TYPE_TOWPLANE, 4-second intervals
          dir_now = direction0 - 0.5 * dir_chg;
      else if (fop->interval == 3)
          dir_now = direction0 - 0.667 * dir_chg;  // 3-second intervals
      else
          dir_now = direction0 - dir_chg;          // 2-second intervals
      gspeed = approxHypotenuse((float) fop->fla_ns[0], (float) fop->fla_ew[0]);
*/
      float dir_now = fop->course;            // already computed in legacy_decode()
      float dir_chg = 3.0 * fop->turnrate;    // internally we use 3-second intervals
      gspeed = fop->speed * (4.0 * _GPS_MPS_PER_KNOT);   /* quarter-meters per sec */
      gs_ns = gspeed * cos_approx(dir_now);
      gs_ew = gspeed * sin_approx(dir_now);     /* present ground-speed vector */
      as_ns = gs_ns - 4*wind_best_ns;
      as_ew = gs_ew - 4*wind_best_ew;           /* present air-speed vector */
      heading = atan2_approx(as_ns, as_ew);
      aspeed = approxHypotenuse(as_ns, as_ew);
      windangle = dir_now - wind_direction;
      if (windangle > 0)  windangle -= 180.0;         /* angle from DOWNwind */
      else                windangle += 180.0;
      dir_chg *= (1.0 + cos_approx(windangle) * wind_speed / aspeed);
      /* this makes air-ref turnrate smaller than the ground-ref turnrate
         when heading upwind, and vice versa */

      if (fabs(dir_chg) > 18.0) {
        /* since the projection is in straight segments rather than a circle, */
        /* correct the speed for the polygon shortcut relative to the circumference */
        /* so that the projected trajectory will reach the points at the right time */
        /* factor = 360/PI/turnrate/interval * sin_approx(turnrate*interval/2) */
       /* - 1/2 slice angle in degrees over interval=3seconds is turnrate * 1.5 */
        float factor = (360.0/3.1416)/dir_chg * sin_approx(0.5*dir_chg);
        if (factor < 0.86)  factor = 0.86;
        if (factor < 0.99)  aspeed *= factor;
      }

      /* whew! now can compute air-ref direction for any future time, simple trig: */
      heading += 0.5 * dir_chg;    // 1.5 sec into future
      if (fop->circling) {
          endturn = 6;
      } else if (fabs(dir_chg) > 15.0) {
          endturn = (int) (90.0 / fabs(dir_chg));    // limit to a 90-degree turn
          if (endturn == 0)  endturn = 1;
      } else {
          endturn = 6;
      }
      for (int i=0; i<6; i++) {
         if (i < endturn) {
             if (heading >  360.0) heading -= 360.0;
             if (heading < -360.0) heading += 360.0;
             ns = (int16_t) roundf(aspeed * cos_approx(heading));
             ew = (int16_t) roundf(aspeed * sin_approx(heading));
             heading += dir_chg;
         }  // else stop turning, keep same velocity vector
         fop->air_ns[i] = ns;
         fop->air_ew[i] = ew;
      }

      if (report) report_that_projection(fop, 1);

      return;
    }

    /* for other protocols, turn rate was NOT obtained in decode() */
    /* rely on history to compute turn rate */

    /* if no usable history, assume a straight path */

    if (fop->gnsstime_ms - fop->prevtime_ms > 3000) {

      fop->turnrate = 0.0;

      if (settings->alarm != TRAFFIC_ALARM_LEGACY)  // don't need more than the turn rate
          return;

      gspeed = fop->speed * _GPS_MPS_PER_KNOT;

      /* compute heading from course and speed and last wind estimate */
      int16_t ns, ew;
      ns = (int16_t) roundf(4.0 * (gspeed * cos_approx(fop->course) - wind_best_ns));
      ew = (int16_t) roundf(4.0 * (gspeed * sin_approx(fop->course) - wind_best_ew));

      /* project a straight line */
      for (i=0; i<6; i++) {
        fop->air_ns[i] = ns;
        fop->air_ew[i] = ew;
      }

      if (report) report_that_projection(fop, 2);

      return;

    }

    /* have history - compute turn rate - degrees per second */
    /* uses current and previous course and time_ms */

    gspeed = fop->speed * _GPS_MPS_PER_KNOT;   /* ground speed */
    course = fop->course;

    /* previous heading from past course and speed and wind */
    float prevheading = fop->prevheading;

    /* same for current time point */
    as_ns = gspeed * cos_approx(fop->course) - wind_best_ns;
    as_ew = gspeed * sin_approx(fop->course) - wind_best_ew;
    heading = atan2_approx(as_ns, as_ew);
    if (heading >  360.0) heading -= 360.0;
    if (heading < -360.0) heading += 360.0;
    fop->heading = heading;                      /* will be carried over into prevheading */
    aspeed = approxHypotenuse(as_ns, as_ew);     /* air speed */

    /* turn rate in the air reference frame (drifting with the wind) */
    float heading_change = heading - prevheading;

    if (fabs(heading_change) > 270.0) {
      /* roll-over through 360 */
      if (heading > 270.0)  heading_change -= 360.0;
      else heading_change += 360.0;
    }
    uint32_t interval = fop->gnsstime_ms - fop->prevtime_ms;
    fop->projtime_ms = fop->gnsstime_ms - (interval >> 1);  /* midway between the 2 time points */
    aturnrate = heading_change / (0.001 * (float) interval);
    if (fabs(aturnrate) > 50.0)  aturnrate = 0.0;        /* ignore implausible data */
    if (fabs(aturnrate) <  2.0)  aturnrate = 0.0;        /* ignore inaccurate data */
    if (interval < 1200 && fop->turnrate != 0) {
       /* short interval between packets, average with previously known turn rate */
       fop->turnrate = 0.5 * (aturnrate + fop->turnrate);
    } else {
       fop->turnrate = aturnrate;   // (stores air ref turnrate in the gnd ref field)
    }

    if (settings->alarm != TRAFFIC_ALARM_LEGACY)  // don't need more than the turn rate
          return;

    /*  compute air-reference NS & EW speed components for future time points */

    if (fabs(aturnrate) < 2.0) {   /* hardly turning - treat it as not turning at all */

      ns = (int16_t) roundf(4.0 * as_ns);
      ew = (int16_t) roundf(4.0 * as_ew);
      for (i=0; i<6; i++) {  /* loop over the 4 time points stored in arrays */
        fop->air_ns[i] = ns;
        fop->air_ew[i] = ew;
      }

      if (report) report_that_projection(fop, 3);

      return;
    }

    /* else, if turning */

    if (fop->projtime_ms > fop->gnsstime_ms)
      heading += aturnrate * (float) (fop->projtime_ms - fop->gnsstime_ms);
    else
      heading -= aturnrate * (float) (fop->gnsstime_ms - fop->projtime_ms);

    if (fabs(aturnrate) > 6.0) {
      /* since the projection is in straight segments rather than a circle, */
      /* correct the speed for the polygon shortcut relative to the circumference */
      /* so that the projected trajectory will reach the points at the right time */
      /* factor = 360/PI/turnrate/interval * sin_approx(turnrate*interval/2) */
     /* - 1/2 slice angle in degrees over interval=3seconds is turnrate * 1.5 */
      float factor = (360.0/3.0/3.1416)/aturnrate * sin_approx(aturnrate*1.5);
      if (factor < 0.86)  factor = 0.86;
      if (factor < 0.99)  aspeed *= factor;
    }

    heading += 1.5 * aturnrate;         // first point will be 1.5 seconds into future
    float dir_chg = 3.0 * aturnrate;   // 3-second intervals after that
    endturn = 6;
    if (fabs(dir_chg) > 15.0) {
        endturn = (int) (90.0 / fabs(dir_chg));    // limit to a 90-degree turn
        if (endturn == 0)  endturn = 1;
    }
    for (i=0; i<6; i++) {
      if (i < endturn) {
        if (heading >  360.0) heading -= 360.0;
        if (heading < -360.0) heading += 360.0;
        ns = (int16_t) roundf(4.0 * aspeed * cos_approx(heading));
        ew = (int16_t) roundf(4.0 * aspeed * sin_approx(heading));
        heading += dir_chg;
      }
      fop->air_ns[i] = ns;
      fop->air_ew[i] = ew;
    }

    if (report) report_that_projection(fop, 4);
}


/* use GPS data to estimate this aircraft's climb rate, used if no baro sensor */
float Estimate_Climbrate(void)
{
    float alt_change = ThisAircraft.altitude - ThisAircraft.prevaltitude;
    float interval = 0.001 * (ThisAircraft.gnsstime_ms - ThisAircraft.prevtime_ms);
    float climbrate = alt_change / interval;
    climbrate *= (_GPS_FEET_PER_METER * 60.0);               /* feet per minute */
    if (fabs(climbrate) > 4000.0)  climbrate = avg_climbrate;  /* ignore implausible data */
    if (fabs(climbrate) <  100.0)  climbrate = 0.0;            /* ignore inaccurate data */
    avg_climbrate = 0.7 * avg_climbrate + 0.3 * climbrate;

    static uint32_t time_to_report = 0;
    if (millis() > time_to_report) {
        time_to_report = millis() + 6300;
#if 0
        Serial.printf("climbrate fpm: %.0f  %.0f,%.0f,%d,%d\r\n", avg_climbrate,
                 ThisAircraft.altitude, ThisAircraft.prevaltitude, ThisAircraft.gnsstime_ms, ThisAircraft.prevtime_ms);
#endif
        if ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_WIND)) {
            snprintf_P(NMEABuffer, sizeof(NMEABuffer),
               PSTR("$PSWCR,%.0f,%.0f,%.0f,%d,%d\r\n"), avg_climbrate,
                 ThisAircraft.altitude, ThisAircraft.prevaltitude, ThisAircraft.gnsstime_ms, ThisAircraft.prevtime_ms);
            NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
        }
    }

    return avg_climbrate;
}
