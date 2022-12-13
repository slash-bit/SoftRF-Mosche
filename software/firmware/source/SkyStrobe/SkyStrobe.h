/*
 * SkyStrobe.h
 * Based on SkyView - Copyright (C) 2019-2022 Linar Yusupov
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

#ifndef SKYSTROBE_H
#define SKYSTROBE_H

#define SKYSTROBE

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#define SKYSTROBE_FIRMWARE_VERSION  "0.2"
#define SKYSTROBE_IDENT     "SkyStrobe-"

#define EXCLUDE_BUTTONS

#define DEFAULT_AP_SSID   "SoftRF-abc123"
#define DEFAULT_AP_PSK    "12345678"

#define RELAY_DST_PORT    12390
#define RELAY_SRC_PORT    (RELAY_DST_PORT - 1)

//#define BRIDGE_UDP_PORT   10110

/* SoftRF serial output defaults */
#define SERIAL_OUT_BR     38400
#define SERIAL_OUT_BITS   SERIAL_8N1

#define DATA_TIMEOUT      2000 /* 2.0 seconds */


typedef struct hardware_info {
    byte  model;
    byte  revision;
    byte  soc;
    byte  display;

} hardware_info_t;

enum
{
	SOFTRF_MODEL_UNKNOWN,
	SOFTRF_MODEL_STANDALONE,
	SOFTRF_MODEL_PRIME,
	SOFTRF_MODEL_UAV,
	SOFTRF_MODEL_PRIME_MK2,
	SOFTRF_MODEL_RASPBERRY,
	SOFTRF_MODEL_UAT,
	SOFTRF_MODEL_SKYVIEW,
	SOFTRF_MODEL_RETRO,
	SOFTRF_MODEL_SKYWATCH,
	SOFTRF_MODEL_DONGLE,
	SOFTRF_MODEL_OCTAVE,
	SOFTRF_MODEL_UNI,
	SOFTRF_MODEL_WEBTOP_SERIAL,
	SOFTRF_MODEL_MINI,
	SOFTRF_MODEL_BADGE,
	SOFTRF_MODEL_ES,
	SOFTRF_MODEL_BRACELET,
	SOFTRF_MODEL_ACADEMY,
	SOFTRF_MODEL_LEGO,
	SOFTRF_MODEL_WEBTOP_USB,
	SOFTRF_MODEL_PRIME_MK3,
	SOFTRF_MODEL_BALKAN,
	SOFTRF_MODEL_RSVD1,
	SOFTRF_MODEL_RSVD2,
	SOFTRF_MODEL_RSVD3,
	SOFTRF_MODEL_RSVD4,
	SOFTRF_MODEL_RSVD5,
	SOFTRF_MODEL_RSVD6,
	SOFTRF_MODEL_RSVD7,
	SOFTRF_MODEL_RSVD8,
	SOFTRF_MODEL_RSVD9,
	SOFTRF_MODEL_SKYSTROBE,
};


enum
{
	CON_NONE,
	CON_SERIAL,
	CON_WIFI_UDP,
	CON_WIFI_TCP,
	CON_BLUETOOTH_SPP,
	CON_BLUETOOTH_LE
};

enum
{
	BRIDGE_NONE,
	BRIDGE_UDP,
	BRIDGE_TCP,
	BRIDGE_BT_SPP,
	BRIDGE_BT_LE,
	BRIDGE_SERIAL
};

enum
{
	B4800,
	B9600,
	B19200,
	B38400,
	B57600,
	B115200,
	B2000000
};

enum
{
	PROTOCOL_NONE,
	PROTOCOL_NMEA, /* FTD-12 */
	PROTOCOL_GDL90,
	PROTOCOL_MAVLINK_1,
	PROTOCOL_MAVLINK_2,
	PROTOCOL_D1090,
	PROTOCOL_UATRADIO
};

enum
{
	POWER_SAVE_NONE = 0,
	POWER_SAVE_WIFI = 1,
	POWER_SAVE_GNSS = 2
};

extern hardware_info_t hw_info;
extern uint32_t when_to_reboot;

void shutdown(const char *);
void Input_loop(void);

#endif /* SKYSTROBE_H */
