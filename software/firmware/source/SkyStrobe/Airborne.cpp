/*
 * Airborne.cpp
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
#include "Strobe.h"

/* keep track of whether this aircraft is airborne */

void airborne_loop()
{
    /* static vars to keep track of 'airborne' status: */
    static int airborne = -5;
    static float initial_latitude = 0;
    static float initial_longitude = 0;
    static float initial_altitude = 0;
    static float prevspeed = 0;
    static float prevaltitude = 0;
    static float prevcourse = 0;

    static uint32_t time_to_check_airborne = 0;
    if (millis() < time_to_check_airborne + 4300)
          return;
    uint32_t last_time = time_to_check_airborne;
    time_to_check_airborne = millis();

    if (! hasGNSS())
        return;

    if (initial_latitude == 0) {
      /* set initial location */
      initial_latitude = ThisAircraft.latitude;
      initial_longitude = ThisAircraft.longitude;
      initial_altitude = ThisAircraft.altitude;
    }

    float speed = ThisAircraft.GroundSpeed;

    if (airborne > 0) {

      if (speed < 1.0)  --airborne;
      /* after 50 calls (~20 sec if consecutive)
         with speed < 1kt consider it a landing */
      if (airborne <= 0) {
        airborne = -5;
        initial_latitude = 0;
      }

    } else if (speed > 1.0) {

      /* > 1 knot but airborne still <= 0 */

      if (speed > 10.0                                                /* 10 knots  */
        || fabs(ThisAircraft.latitude - initial_latitude) > 0.0018f   /* about 200 meters */
        || fabs(ThisAircraft.longitude - initial_longitude) > 0.0027f
        || fabs(ThisAircraft.altitude - initial_altitude) > 120.0f) {
          /* movement larger than typical GNSS noise */
          ++airborne;

          float interval = 0.001 * (float)(time_to_check_airborne - last_time);
          if (fabs(ThisAircraft.altitude - prevaltitude) > 20.0 * interval
           || fabs(ThisAircraft.Track - prevcourse) > 50.0 * interval
           || speed > 4.0 * prevspeed || prevspeed > 4.0 * speed) {
             /* supposed initial movement is too jerky */
             if (airborne > -5)  airborne -= 2;
          }

          prevspeed = speed;
          prevcourse = ThisAircraft.Track;
          prevaltitude = ThisAircraft.altitude;

          if (airborne > 0)    /* consistently good indications */
               airborne = 50;  /* now really airborne */
      }

    }

    ThisAircraft.airborne = (airborne > 0)? 1 : 0;
}
