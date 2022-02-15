/*
 * ApproxMath.cpp
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
#include "ApproxMath.h"

/* For the purposes used here, trig functions don't need much precision */
/* - on the other hand can save CPU time by using faster approximations */

/* helper function, only valid for positive arguments      */
/* quadratic fit on 0-45 range, results within +-0.25 deg  */
float atan_positive(float ns, float ew)
{
  float t;
  if (ew < ns) {
    t = ew / ns;
    return (45.0*t + 15.5*(t*(1.0-t)));
  } else {
    t = ns / ew;
    return (90.0 - 45.0*t - 15.5*(t*(1.0-t)));
  }
}

/* An approximation to atan2()                         */
/* - returned value is in degrees clockwise-from-North */
/* - arguments are in reverse order from library atan2 */
float atan2_approx(float ns, float ew)
{
  if (ew > 0.0) {
    if (ns > 0.0) return atan_positive(ns,ew);
    if (ns < 0.0) return 180.0 - atan_positive(-ns,ew);
    /* if (ns==0) */ return 90.0;
  } else if (ew < 0.0) {
    if (ns > 0.0) return 360.0 - atan_positive(ns,-ew);
    if (ns < 0.0) return 180.0 + atan_positive(-ns,-ew);
    /* if (ns==0) */ return 270.0;
  } else {  /* if (ew==0) */
    if (ns >= 0.0) return 0.0;
    return 180.0;
  }
}

/* Bhaskara's (7th century) approximation for the Sine and Cosine, expanded range: */
/*   https://scholarworks.umt.edu/cgi/viewcontent.cgi?article=1313&context=tme     */

/* approximate sin(), argument in degrees, meant for +-360deg range */
float sin_approx(float degs)
{
  bool neg;
  float prathama, sine;

  neg = (degs < 0.0);
  if (neg)  degs = -degs;
  if (degs > 360.0)  degs = degs - 360.0;
  if (degs > 360.0)  degs = degs - 360.0;
  if (degs > 180.0) {
    prathama = (degs - 180.0) * (360.0 - degs);
    neg = !neg;
  } else {
    prathama = degs * (180.0 - degs);
  }
  sine = 4.0*prathama / (40500.0-prathama);
  if (neg)  return -sine;
  return sine;
}

float cos_approx(float degs)
{
  return sin_approx(degs+90.0);
}

/* cos(latitude) is used to convert longitude difference into linear distance. */
/* Only compute once, that is accurate enough for use through the whole flight */

float CosLat(float latitude)
{
  static float cos_lat = 0.0;
  if (cos_lat == 0.0)
    cos_lat = cos_approx(latitude);
  return cos_lat;
}
