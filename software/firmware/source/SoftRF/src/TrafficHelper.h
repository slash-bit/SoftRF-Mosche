/*
 * TrafficHelper.h
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

#ifndef TRAFFICHELPER_H
#define TRAFFICHELPER_H

#include "system/SoC.h"

/* for DISTANCE method: traffic beyond ALARM_ZONE_NONE is invisible */
#define ALARM_ZONE_NONE       15000 /* zone range is 1500m <-> 15000m */
#define ALARM_ZONE_CLOSE      1500  /* zone range is 1000m <->  1500m */
#define ALARM_ZONE_LOW        1000  /* zone range is  700m <->  1000m */
#define ALARM_ZONE_IMPORTANT  700   /* zone range is  400m <->   700m */
#define ALARM_ZONE_URGENT     400   /* zone range is    0m <->   400m */

/* for VECTOR method: */
#define ALARM_VECTOR_ANGLE    10.1
#define ALARM_VECTOR_SPEED    0.3
#define ALARM_TIME_CLOSE      30.1
#define ALARM_TIME_LOW        19.1
#define ALARM_TIME_IMPORTANT  13.1
#define ALARM_TIME_URGENT     9.1

#define VERTICAL_SLOPE                5  /* slope effect for alerts */
#define VERTICAL_SLACK               60  /* meters  - allow for GPS alt error */
#define VERTICAL_SEPARATION         300  /* meters  - for alerts */
#define VERTICAL_VISIBILITY_RANGE   900  /* this value higher than FLARM specs */

#define TRAFFIC_VECTOR_UPDATE_INTERVAL 2 /* seconds */
#define TRAFFIC_UPDATE_INTERVAL_MS (TRAFFIC_VECTOR_UPDATE_INTERVAL * 1000)
#define isTimeToUpdateTraffic() (millis() - UpdateTrafficTimeMarker > \
                                  TRAFFIC_UPDATE_INTERVAL_MS)

typedef struct traffic_by_dist_struct {
  ufo_t *fop;
  float distance;
} traffic_by_dist_t;

enum
{
	TRAFFIC_ALARM_NONE,
	TRAFFIC_ALARM_DISTANCE,
	TRAFFIC_ALARM_VECTOR,
	TRAFFIC_ALARM_LEGACY
};

#define TRAFFIC_ALERT_SOUND   1

void ParseData(void);
void Traffic_setup(void);
void Traffic_loop(void);
void ClearExpired(void);
void Traffic_Update(ufo_t *);
int  Traffic_Count(void);

int  traffic_cmp_by_distance(const void *, const void *);

extern ufo_t fo, Container[MAX_TRACKING_OBJECTS], EmptyFO;
extern traffic_by_dist_t traffic_by_dist[MAX_TRACKING_OBJECTS];

#endif /* TRAFFICHELPER_H */
