/*
 * Protocol_Legacy, decoder for legacy radio protocol
 * Copyright (C) 2014-2015 Stanislaw Pusep
 *
 * Protocol_Legacy, encoder for legacy radio protocol
 * Copyright (C) 2016-2021 Linar Yusupov
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

#include <math.h>
#include <stdint.h>

#include <protocol.h>

#include "../../../SoftRF.h"
#include "../../TrafficHelper.h"
#include "../../Wind.h"
#include "../../ApproxMath.h"
#include "../../driver/RF.h"
#include "../../driver/EEPROM.h"
#include "../data/NMEA.h"

const rf_proto_desc_t legacy_proto_desc = {
  "Legacy",
  .type            = RF_PROTOCOL_LEGACY,
  .modulation_type = RF_MODULATION_TYPE_2FSK,
  .preamble_type   = LEGACY_PREAMBLE_TYPE,
  .preamble_size   = LEGACY_PREAMBLE_SIZE,
  .syncword        = LEGACY_SYNCWORD,
  .syncword_size   = LEGACY_SYNCWORD_SIZE,
  .net_id          = 0x0000, /* not in use */
  .payload_type    = RF_PAYLOAD_INVERTED,
  .payload_size    = LEGACY_PAYLOAD_SIZE,
  .payload_offset  = 0,
  .crc_type        = LEGACY_CRC_TYPE,
  .crc_size        = LEGACY_CRC_SIZE,

  .bitrate         = RF_BITRATE_100KBPS,
  .deviation       = RF_FREQUENCY_DEVIATION_50KHZ,
  .whitening       = RF_WHITENING_MANCHESTER,
  .bandwidth       = RF_RX_BANDWIDTH_SS_125KHZ,

  .air_time        = LEGACY_AIR_TIME,

#if defined(USE_TIME_SLOTS)
  .tm_type         = RF_TIMING_2SLOTS_PPS_SYNC,
#else
  .tm_type         = RF_TIMING_INTERVAL,
#endif
  .tx_interval_min = LEGACY_TX_INTERVAL_MIN,
  .tx_interval_max = LEGACY_TX_INTERVAL_MAX,
  .slot0           = {400,  800},
  .slot1           = {800, 1200}
};

/* http://en.wikipedia.org/wiki/XXTEA */
void btea(uint32_t *v, int8_t n, const uint32_t key[4]) {
    uint32_t y, z, sum;
    uint32_t p, rounds, e;

    #define DELTA 0x9e3779b9
    // #define ROUNDS (6 + 52 / n)
    #define ROUNDS 6
    #define MX (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^ ((sum ^ y) + (key[(p & 3) ^ e] ^ z)))

    if (n > 1) {
        /* Coding Part */
        rounds = ROUNDS;
        sum = 0;
        z = v[n - 1];
        do {
            sum += DELTA;
            e = (sum >> 2) & 3;
            for (p = 0; p < n - 1; p++) {
                y = v[p + 1];
                z = v[p] += MX;
            }
            y = v[0];
            z = v[n - 1] += MX;
        } while (--rounds);
    } else if (n < -1) {
        /* Decoding Part */
        n = -n;
        rounds = ROUNDS;
        sum = rounds * DELTA;
        y = v[0];
        do {
            e = (sum >> 2) & 3;
            for (p = n - 1; p > 0; p--) {
                z = v[p - 1];
                y = v[p] -= MX;
            }
            z = v[n - 1];
            y = v[0] -= MX;
            sum -= DELTA;
        } while (--rounds);
    }
}

/* http://pastebin.com/YK2f8bfm */
long obscure(uint32_t key, uint32_t seed) {
    uint32_t m1 = seed * (key ^ (key >> 16));
    uint32_t m2 = (seed * (m1 ^ (m1 >> 16)));
    return m2 ^ (m2 >> 16);
}

static const uint32_t table[8] = LEGACY_KEY1;

void make_key(uint32_t key[4], uint32_t timestamp, uint32_t address) {
    int8_t i, ndx;
    for (i = 0; i < 4; i++) {
        ndx = ((timestamp >> 23) & 1) ? i+4 : i ;
        key[i] = obscure(table[ndx] ^ ((timestamp >> 6) ^ address), LEGACY_KEY2) ^ LEGACY_KEY3;
    }
}

bool legacy_decode(void *legacy_pkt, ufo_t *this_aircraft, ufo_t *fop) {

    legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;

    // FLARM seems to send some other type of data packet occasionally, ignore it
    if (pkt->_unk0 != 0)
        return false;

    float ref_lat = this_aircraft->latitude;
    float ref_lon = this_aircraft->longitude;
    float geo_separ = this_aircraft->geoid_separation;
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    uint32_t key[4];
    int ndx;
    uint8_t pkt_parity=0;

    make_key(key, timestamp, (pkt->addr << 8) & 0xffffff);
    btea((uint32_t *) pkt + 1, -5, key);

    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }
    if (pkt_parity % 2) {
        if (settings->nmea_p) {
          StdOut.print(F("$PSRFE,bad parity of decoded packet: "));
          StdOut.println(pkt_parity % 2, HEX);
        }
        return false;
    }

    fop->addr = pkt->addr;
    
    if (fop->addr == settings->ignore_id)
         return true;                 /* ID told in settings to ignore */
    if (fop->addr == ThisAircraft.addr)
         return true;                 /* same ID as this aircraft - ignore */
    /* return true so that the packet will reach ParseData() */

    fop->protocol = RF_PROTOCOL_LEGACY;
    fop->addr_type = pkt->addr_type;
    fop->timestamp = timestamp;
    fop->gnsstime_ms = millis();

    // this section revised by MB on 220526
    int32_t round_lat, round_lon;
    if (ref_lat < 0.0)
        round_lat = -(((int32_t) (-ref_lat * 1e7) + 0x40) >> 7);
    else
        round_lat = ((int32_t) (ref_lat * 1e7) + 0x40) >> 7;
    int32_t ilat = ((int32_t)pkt->lat - round_lat) & 0x07FFFF;
    if (ilat >= 0x040000) ilat -= 0x080000;
    float lat = (float)((ilat + round_lat) << 7) * 1e-7;
    if (ref_lon < 0.0)
        round_lon = -(((int32_t) (-ref_lon * 1e7) + 0x40) >> 7);
    else
        round_lon = ((int32_t) (ref_lon * 1e7) + 0x40) >> 7;
    int32_t ilon = ((int32_t)pkt->lon - round_lon) & 0x0FFFFF;
    if (ilon >= 0x080000) ilon -= 0x0100000;
    float lon = (float)((ilon + round_lon) << 7) * 1e-7;

    uint8_t smult = pkt->smult;
    float nsf = (float) (pkt->ns[0] << smult);      /* quarter-meters per sec */
    float ewf = (float) (pkt->ew[0] << smult);
//    float heading = atan2_approx(nsf, ewf);
//    /* if those are airspeeds, adjust for wind */   // they are not airspeeds.
//    nsf += 4.0 * wind_best_ns;
//    ewf += 4.0 * wind_best_ew;
    float course = atan2_approx(nsf, ewf);
    float speed4 = approxHypotenuse(nsf, ewf);
    float turnrate = 0;
    if (speed4 > 0) {
      float nextcourse = atan2_approx((float) pkt->ns[1], (float) pkt->ew[1]);
      float turnangle = (nextcourse - course);
      if (turnangle >  270.0) turnangle -= 360.0;
      if (turnangle < -270.0) turnangle += 360.0;
      //turnrate = 0.333 * turnangle;  /* assuming 3 seconds interval */
      turnrate = 0.5 * turnangle;  /* >>> assuming 2 seconds interval */
      /* adjust direction for turning during time between "now" and [0] */
      // >>> assumes that interval is 2s and that [0] is 2s after "now"
      course -= turnangle;
      if (course >  360.0) course -= 360.0;
      if (course < -360.0) course += 360.0;
    }

    uint16_t vs_u16 = pkt->vs;
    int16_t vs_i16 = (int16_t) (vs_u16 | (vs_u16 & (1<<9) ? 0xFC00U : 0));
    int16_t vs10 = vs_i16 << smult;

    int16_t alt = pkt->alt ; /* relative to WGS84 ellipsoid */

    fop->airborne = pkt->airborne;
    fop->latitude = lat;
    fop->longitude = lon;
    fop->altitude = (float) alt - geo_separ;
    fop->speed = (1.0 / (4.0 * _GPS_MPS_PER_KNOT)) * speed4;
    fop->course = course;
//    fop->heading = heading;
    fop->turnrate = turnrate;
         /* this is as reported by FLARM, which is ground-reference - at time [0]? */
    fop->vs = ((float) vs10) * (_GPS_FEET_PER_METER * 6.0);
    fop->aircraft_type = pkt->aircraft_type;
    fop->stealth = pkt->stealth;
    fop->no_track = pkt->no_track;
    /* Keep the data given for the first 2 time points  */
    /* The other 2 time points are not useful in wind   */
    /* due to the data being in neither reference frame */
    fop->fla_ns[0] = ((int16_t) pkt->ns[0]) << smult;
    fop->fla_ns[1] = ((int16_t) pkt->ns[1]) << smult;
    fop->fla_ew[0] = ((int16_t) pkt->ew[0]) << smult;
    fop->fla_ew[1] = ((int16_t) pkt->ew[1]) << smult;
    fop->projtime_ms = fop->gnsstime_ms;

#if 1
    /* send received radio packet data out via NMEA for debugging */
    if (settings->nmea_d || settings->nmea2_d) {
      if (settings->debug_flags & DEBUG_LEGACY) {
        int16_t ns2 = ((int16_t) pkt->ns[2]) << smult;
        int16_t ns3 = ((int16_t) pkt->ns[3]) << smult;
        int16_t ew2 = ((int16_t) pkt->ew[2]) << smult;
        int16_t ew3 = ((int16_t) pkt->ew[3]) << smult;
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFL,%06X,%ld,%.6f,%.6f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n"),
          fop->addr, (int32_t)fop->gnsstime_ms - (int32_t)ThisAircraft.gnsstime_ms,
          fop->latitude, fop->longitude, fop->altitude,
          course, turnrate, vs10, smult, fop->airborne, pkt->_unk2,
          fop->fla_ns[0], fop->fla_ns[1], ns2, ns3,
          fop->fla_ew[0], fop->fla_ew[1], ew2, ew3);
        NMEA_Outs(settings->nmea_d, settings->nmea2_d, (byte *) NMEABuffer, strlen(NMEABuffer), false);
        if (settings->debug_flags & DEBUG_RESVD1) {
          /* also output the raw (but decrypted) packet as a whole, in hex */
          snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PSRFB,%06X,%ld,%s\r\n"),
            fop->addr, fop->gnsstime_ms,
            bytes2Hex((byte *)pkt, sizeof (legacy_packet_t)));
          NMEA_Outs(settings->nmea_d, settings->nmea2_d, (byte *) NMEABuffer, strlen(NMEABuffer), false);
        }
      }
    }
#endif

    return true;
}

size_t legacy_encode(void *legacy_pkt, ufo_t *this_aircraft) {

    legacy_packet_t *pkt = (legacy_packet_t *) legacy_pkt;

    int ndx;
    uint8_t pkt_parity;
    uint32_t key[4];

    float lat = this_aircraft->latitude;
    float lon = this_aircraft->longitude;
    int16_t alt = (int16_t) (this_aircraft->altitude + this_aircraft->geoid_separation);
    uint32_t timestamp = (uint32_t) this_aircraft->timestamp;

    float course = this_aircraft->course;
    float speedf = this_aircraft->speed * _GPS_MPS_PER_KNOT; /* m/s */
    float vsf = this_aircraft->vs / (_GPS_FEET_PER_METER * 60.0); /* m/s */

    uint16_t speed4 = (uint16_t) roundf(speedf * 4.0f);
    if (speed4 > 0x3FF) {
      speed4 = 0x3FF;
    }

    uint8_t smult;
    if        (speed4 & 0x200) {
      smult = 3;
    } else if (speed4 & 0x100) {
      smult = 2;
    } else if (speed4 & 0x080) {
      smult = 1;
    } else {
      smult = 0;
    }
    pkt->smult = smult;

//    if (this_aircraft->prevtime_ms != 0) {
      /* Compute NS & EW speed components for future time points. */
      project_this(this_aircraft);       /* which also calls airborne() */
      if (millis() - SetupTimeMarker < 60000) {
          pkt->airborne = 1;    /* post-boot testing */
      } else {
          pkt->airborne = this_aircraft->airborne;
      }
      for (int i=0; i<4; i++) {
         pkt->ns[i] = (int8_t) (this_aircraft->fla_ns[i] >> smult);
         pkt->ew[i] = (int8_t) (this_aircraft->fla_ew[i] >> smult);
      }
      /* quarter-meters per sec if smult==0 */

    int16_t vs10 = (int16_t) roundf(vsf * 10.0f);
/*  pkt->vs = this_aircraft->stealth ? 0 : vs10 >> pkt->smult; */
/*  - that degrades collision avoidance - should only mask vs in NMEA */
    pkt->vs = vs10 >> smult;

    uint32_t id = this_aircraft->addr;

    pkt->addr_type = settings->id_method;

    pkt->addr = id & 0x00FFFFFF;

    pkt->stealth = this_aircraft->stealth;
    pkt->no_track = this_aircraft->no_track;

    pkt->aircraft_type = this_aircraft->aircraft_type;

    pkt->gps = 323;

    float turnRate = this_aircraft->turnrate;

    /* project position 2 seconds into future, as it seems that FLARM does that */
    course += turnRate;           // 1 second into future
    float offset = speedf * (2.0 / 111300.0);   // degslat/sec * 2 sec = degs moved
    lat += (offset * cos_approx(course));
    lon += (offset * sin_approx(course) * InvCosLat());

    // this section revised by MB on 220526
    if (lat < 0.0)
        pkt->lat = (uint32_t) (-(((int32_t) (-lat * 1e7) + 0x40) >> 7)) & 0x07FFFF;
    else
        pkt->lat = (((uint32_t) (lat * 1e7) + 0x40) >> 7) & 0x07FFFF;
    if (lon < 0.0)
        pkt->lon = (uint32_t) (-(((int32_t) (-lon * 1e7) + 0x40) >> 7)) & 0x0FFFFF;
    else
        pkt->lon = (((uint32_t) (lon * 1e7) + 0x40) >> 7) & 0x0FFFFF;

    if (alt < 0) {
        alt = 0;    // cannot be negative
    } else {
      if (alt >= (1<<13))
          alt = (1<<13)-1; // clamp to Max
    }
    pkt->alt = alt;

    /* FLARM seems to send this in _unk2 from a glider */
    static int turning = 1;   // not turning sharply
    if (turnRate > 18)
        turning = 0;           // turning sharply right
    else if (turnRate < -18)
        turning = 3;           // turning sharply left
    else if (turnRate > -10 && turnRate < 10)
        turning = 1;           // return to not-turning with hysteresis
    // else
    //   retain previous value of turning
    pkt->_unk2 = turning;

    pkt->_unk0 = 0;
    pkt->_unk1 = 0;
    pkt->_unk3 = 0;
//    pkt->_unk4 = 0;

    pkt->parity = 0;
    for (ndx = 0; ndx < sizeof (legacy_packet_t); ndx++) {
      pkt_parity += parity(*(((unsigned char *) pkt) + ndx));
    }

    pkt->parity = (pkt_parity % 2);

    make_key(key, timestamp , (pkt->addr << 8) & 0xffffff);

#if 0
    Serial.print(key[0]);   Serial.print(", ");
    Serial.print(key[1]);   Serial.print(", ");
    Serial.print(key[2]);   Serial.print(", ");
    Serial.println(key[3]);
#endif
    btea((uint32_t *) pkt + 1, 5, key);

    return (sizeof(legacy_packet_t));
}
