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
#include "ApproxMath.h"
#include "Wind.h"
#include "driver/EEPROM.h"
#include "driver/GNSS.h"
#include "driver/WiFi.h"


float wind_best_ns = 0.0;  /* mps */
float wind_best_ew = 0.0;
int circling = 0;          /* 1 = clockwise, -1 = counterclockwise */
float avg_turnrate = 0.0;
float avg_speed = 0.0;     /* average around the circle */
float avg_climbrate = 0.0; /* based on GNSS data */


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

  bool ok = false;
  bool ns = false;
  bool ew = false;

  int32_t new_time = ThisAircraft.gnsstime_ms;

  /* recover from GPS outage */
  if (new_time - old_time > 3000) {
     old_time = new_time;
     circling = 0;
     old_turnrate = 0.0;
     avg_turnrate = 0.0;
     return;
  }
  old_time = new_time;

  /* calc_turnrate(&ThisAircraft); - converted to inline here: */
  /* compute (ground-reference) turn rate - degrees per second */
  /* uses current and previous course and time_ms */
  float course = ThisAircraft.course;
  float course_change = course - ThisAircraft.prevcourse;
  if (fabs(course_change) > 270.0) {
      /* roll-over through 360 */
      if (course > 270.0)  course_change -= 360.0;
      else course_change += 360.0;
  }
  float interval = 0.001 * (ThisAircraft.gnsstime_ms - ThisAircraft.prevtime_ms);
  float turnrate = course_change / interval;
  if (fabs(turnrate) > 50.0)  turnrate = 0.0;        /* ignore implausible data */
  if (fabs(turnrate) <  2.0)  turnrate = 0.0;        /* ignore inaccurate data */
  ThisAircraft.turnrate = turnrate;

  static uint32_t decaytime = 0;

  if (circling == 0) { /* not considered in a stable circle */

    /* slowly decay wind towards zero over time */
    /* with a half-life of about 20 minutes     */
    /* - but new circling will refresh it       */
    /* piggy-back random time for new random ID */
    if (new_time > decaytime) {
        decaytime = new_time + 100000;
        wind_best_ns *= 0.95;
        wind_best_ew *= 0.95;
        if (settings->id_method == ADDR_TYPE_RANDOM)
             generate_random_id();
    }

    if (old_turnrate == 0.0) {  /* was not turning */
      if (fabs(turnrate) > 5.0 && fabs(turnrate) < 40.0) {  /* now turning */
        /* start watching whether stable */
        old_turnrate = turnrate;
        old_course = ThisAircraft.course;
        return;
      }
    }

    if ((old_turnrate > 0.0 && turnrate < 2.0)
     || (old_turnrate < 0.0 && turnrate > -2.0)) {
      /* not a consistent turn */ 
      old_turnrate = 0.0;
      return;
    }

    course_change = fabs(ThisAircraft.course - old_course);
    if (course_change > 300.0)   /* passed through North */
      course_change = 360.0 - course_change;
    if (course_change > 180.0) {  /* completed a half-turn */
       if (turnrate > 0.0)  circling = 1;
       if (turnrate < 0.0)  circling = -1;
       start_time = new_time;    
       old_course = ThisAircraft.course;
       cumul_turn = 0.0;
       oldquadrant = 0;
       old_lat_time = 0;
       old_lon_time = 0;
    }

    old_turnrate = turnrate;
    return;
  }

  /* got here if circling != 0, check whether still circling */

  if ((circling > 0 && turnrate < 2.0)
   || (circling < 0 && turnrate > -2.0)) {
     /* no longer a consistent turn */ 
     circling = 0;
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

  float wind_ns;
  float wind_ew;
  float direction;
  float windspeed;

  if (fabs(cumul_turn) > 360.0) {  /* completed a circle */

       turnrate = 360000.0 * (float) circling / (float) (new_time - start_time);
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
       if (ok && (max_gs - min_gs > 4.0)) {        /* ignore wind estimate < 2 knots */
         windspeed = 0.5 * (max_gs - min_gs) * _GPS_MPS_PER_KNOT;
         wind_ns = windspeed * cos_approx(direction);
         wind_ew = windspeed * sin_approx(direction);
         if (fabs(wind_ns-wind_best_ns) < 20.0
         &&  fabs(wind_ew-wind_best_ew) < 20.0) {            /* ignore implausible values */
           if (wind_best_ns == 0.0)  /* not initialized yet */
             wind_best_ns = wind_ns;
           else
             wind_best_ns = 0.9*wind_best_ns + 0.1*wind_ns;  /* only gradually change "best" estimate */
           if (wind_best_ew == 0.0)
             wind_best_ew = wind_ew;
           else
             wind_best_ew = 0.9 * wind_best_ew + 0.1 * wind_ns;
           if (avg_speed == 0.0)
             avg_speed = 0.5 * (min_gs + max_gs);            /* = average AIRspeed */
           else
             avg_speed = 0.7 * avg_speed + 0.3 * 0.5*(min_gs+max_gs);
             /* this is retained over time and changed gradually */
         }
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
  int8_t quadrant = 1;
  if (icourse > 90) {
    ++quadrant;
    if (icourse > 180) {
      ++quadrant;
      if (icourse > 270)
        ++quadrant;
    }
  }

  /* detect completion of NS/EW-oriented circles */

  if (circling > 0) {  /* clockwise */
    if (quadrant == 2 && oldquadrant == 1)  /* traveling East, on the North side of the circle */
      ns = true;
    if (quadrant == 3 && oldquadrant == 2)  /* traveling South, on the East side of the circle */
      ew = true;
  } else if (circling < 0) { /* counter-clockwise */
    if (quadrant == 3 && oldquadrant == 4)  /* traveling West, on the North side of the circle */
      ns = true;
    if (quadrant == 4 && oldquadrant == 1)  /* traveling North, on the East side of the circle */
      ew = true;
  }

  oldquadrant = quadrant;  /* set up to detect the next quadrant crossing */

  /* measure drift in completed circles */

  float new_lat;
  float new_lon;
  float drift_ns;
  float drift_ew;

  if (ns) {  /* started and perhaps completed a circle */
    if (old_lat_time != 0) {  /* there is history to use */
      new_lat = ThisAircraft.latitude;
      drift_ns = 111300.0 * (new_lat - old_lat);   /* how far further North, in meters */
      interval = 0.001 * (new_time - old_lat_time);
      wind_ns = drift_ns / interval;               /* m/s */
      if (fabs(wind_ns-wind_best_ns) < 20.0 && fabs(wind_ns-wind_best_ns) > 1.0) {
        if (wind_best_ns == 0.0)  /* not initialized yet */
          wind_best_ns = wind_ns;
        else
          wind_best_ns = 0.9*wind_best_ns + 0.1*wind_ns;  /* only gradually change "best" estimate */
      }
    }
    old_lat = new_lat;  /* start observing a new circle */
    old_lat_time = new_time;
  }
  
  if (ew) {
    if (old_lon_time != 0) {
      new_lon = ThisAircraft.longitude;
      drift_ew = 111300.0 * (new_lon - old_lon) * CosLat(ThisAircraft.latitude); /* how far further East */
      interval = 0.001 * (new_time - old_lon_time);
      wind_ew = drift_ew / interval;
      if (fabs(wind_ew-wind_best_ew) < 20.0 && fabs(wind_ew-wind_best_ew) > 1.0) {
        if (wind_best_ew == 0.0)
          wind_best_ew = wind_ns;
        else
          wind_best_ew = 0.9*wind_best_ew + 0.1*wind_ew;
      }
    }
    old_lon = new_lon;  /* start observing a new circle */
    old_lon_time = new_time;
  }

  if (ns || ew) {
    turnrate = 360.0 * (float) circling / interval;
    if (fabs(turnrate) > 50.0)  turnrate = avg_turnrate;   /* ignore implausible data */
    if (fabs(turnrate) <  2.0)  turnrate = 0.0;            /* ignore inaccurate data */
    if (avg_turnrate == 0.0)
        avg_turnrate = turnrate;
    else
        avg_turnrate = 0.7 * avg_turnrate + 0.3 * turnrate;
  }

  /* send radio packet data out via UDP for debugging */
  if ((settings->debug_flags & DEBUG_WIND) && udp_is_ready) {
    if (ok) {
      snprintf_P(UDPpacketBuffer, UDP_PACKET_BUFSIZE,
        PSTR("$PFLAS,W,GS,%ld,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f\n\0"),
        new_time, ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        min_gs, max_gs, min_gs_course, max_gs_course, windspeed, direction,
        wind_ns, wind_ew, wind_best_ns, wind_best_ew, avg_speed);
      SoC->WiFi_transmit_UDP(NMEA_UDP_PORT, (byte *) UDPpacketBuffer,
                       strlen(UDPpacketBuffer));
    }
    if (ns) {
      snprintf_P(UDPpacketBuffer, UDP_PACKET_BUFSIZE,
        PSTR("$PFLAS,W,NS,%ld,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n\0"),
        new_time, ThisAircraft.latitude, ThisAircraft.longitude,
        ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        interval, drift_ns, wind_ns, wind_best_ns, wind_best_ew);
      SoC->WiFi_transmit_UDP(NMEA_UDP_PORT, (byte *) UDPpacketBuffer,
                       strlen(UDPpacketBuffer));
    }
    if (ew) {
      snprintf_P(UDPpacketBuffer, UDP_PACKET_BUFSIZE,
        PSTR("$PFLAS,W,EW,%ld,%.5f,%.5f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n\0"),
        new_time, ThisAircraft.latitude, ThisAircraft.longitude,
        ThisAircraft.speed, ThisAircraft.course, ThisAircraft.turnrate,
        interval, drift_ew, wind_ew, wind_best_ns, wind_best_ew);
      SoC->WiFi_transmit_UDP(NMEA_UDP_PORT, (byte *) UDPpacketBuffer,
                       strlen(UDPpacketBuffer));
    }
  }

}


void project_course(ufo_t *this_aircraft, ufo_t *fop)
{
    int i, ns, ew;

    /* For this aircraft only recompute once every 400 ms */
    static uint32_t lasttime_ms = 0;
    if (fop == this_aircraft) {
      if (this_aircraft->gnsstime_ms - lasttime_ms < 400)  return;
      lasttime_ms = this_aircraft->gnsstime_ms;
    }

      fop->turnrate = 0.0;
      float speed4mps = fop->speed * _GPS_MPS_PER_KNOT * 4.0;
      ns = (int) (speed4mps * cos_approx(fop->course));
      ew = (int) (speed4mps * sin_approx(fop->course));
      for (i=0; i<4; i++) {
        fop->ns[i] = ns;
        fop->ew[i] = ew;
      }
}


float Estimate_Climbrate(void)
{
    float alt_change = ThisAircraft.altitude - ThisAircraft.prevaltitude;
    float interval = 0.001 * (ThisAircraft.gnsstime_ms - ThisAircraft.prevtime_ms);
    float climbrate = alt_change / interval;
    if (fabs(climbrate) > 20.0)  climbrate = avg_climbrate;  /* ignore implausible data */
    if (fabs(climbrate) <  0.5)  climbrate = 0.0;            /* ignore inaccurate data */
    avg_climbrate = 0.7 * avg_climbrate + 0.3 * climbrate;
    return avg_climbrate * (_GPS_FEET_PER_METER * 60.0); /* feet per minute */
}
