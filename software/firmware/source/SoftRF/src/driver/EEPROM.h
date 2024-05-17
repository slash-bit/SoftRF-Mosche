/*
 * EEPROMHelper.h
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

#ifndef EEPROMHELPER_H
#define EEPROMHELPER_H

#include "../../SoftRF.h"

//one of the following needs to be defined in SoftRF.h:  (not used any more)
//#define DEFAULT_REGION_EU
//#define DEFAULT_REGION_US

//#if !defined(DEFAULT_REGION_EU) && !defined(DEFAULT_REGION_US)
//#error No default region defined
//#endif

//#if defined(DEFAULT_REGION_EU) && defined(DEFAULT_REGION_US)
//#error Multiple default regions defined
//#endif

#include "../system/SoC.h"

#if !defined(EXCLUDE_EEPROM)
#if defined(ENERGIA_ARCH_CC13XX) || defined(ENERGIA_ARCH_CC13X2)
#include <EEPROM_CC13XX.h>
#else
#include <EEPROM.h>
#endif /* CC13XX or CC13X2 */
#endif /* EXCLUDE_EEPROM */

#define SOFTRF_EEPROM_MAGIC   0xBABADEDA
#define SOFTRF_EEPROM_VERSION 0xACAC0B0C

enum
{
	EEPROM_EXT_LOAD,
	EEPROM_EXT_DEFAULTS,
	EEPROM_EXT_STORE
};

enum
{
	STROBE_OFF = 0,
	STROBE_ALARM,
	STROBE_AIRBORNE,
	STROBE_ALWAYS
};

//#if !defined(EXCLUDE_VOICE)
//#if defined(ESP32)
enum
{
	VOICE_OFF = 0,
	VOICE_INT,
	VOICE_EXT
};
//#endif
//#endif

enum
{
	DEST_NONE,
	DEST_UART,
	DEST_UDP,
	DEST_TCP,
	DEST_USB,
	DEST_BLUETOOTH,
	DEST_UART2
};

enum
{
	BAUD_DEFAULT = 0,
	BAUD_4800 = 1,
	BAUD_9600 = 2,
	BAUD_19200 = 3,
	BAUD_38400 = 4,
	BAUD_57600 = 5,
	BAUD_115200 = 6,
	BAUD_2000000 = 7
};

enum
{
	TCP_MODE_SERVER=0,
	TCP_MODE_CLIENT
};

enum
{
	RELAY_OFF=0,
	RELAY_LANDED,
	RELAY_ALL,
	RELAY_ONLY
};

typedef struct Settings {
    uint8_t  mode:4;
    uint8_t  rf_protocol:4;
    uint8_t  band:4;
    uint8_t  txpower:2;
    uint8_t  volume:2;
    uint8_t  aircraft_type;
    //uint8_t  led_num;    // not used
    uint8_t  resvd1;

    bool     nmea_g:1;
    bool     nmea_p:1;
    bool     nmea_l:1;
    bool     nmea_s:1;
    bool     nmea_d:1;
    uint8_t  nmea_out:3;

    uint8_t  bluetooth:3; /* ESP32 built-in Bluetooth */
    uint8_t  alarm:3;
    bool     stealth:1;
    bool     no_track:1;

    uint8_t  gdl90:3;    // output destination
    uint8_t  d1090:3;
    uint8_t  json:2;

    uint8_t  pointer:2;
    uint8_t  power_save:2;
    uint8_t  power_external:1;  /* if nonzero, shuts down if battery is not full */
    uint8_t  resvd2:3;

    uint8_t  resvd3;

    int8_t   freq_corr; /* +/-, kHz */
    uint8_t  relay:2;
    uint8_t  gdl90_in:3;    // data from this port will be interpreted as GDL90
    uint8_t  alt_udp:1;     // if 1 then use 10111 instead of 10110
    bool     nmea_e:1;
    bool     nmea2_e:1;     // whether to send bridged data
    uint8_t  baud_rate:3;   /* for serial UART0 */
    uint8_t  baudrate2:3;   /* for aux UART2 */
    bool     invert2:1;     // whether to invert the logic levels on UART2
    bool     altpin0:1;     // whether to use a different pin for UART0 RX

    /* Use a key provided by (local) gliding contest organizer */
    uint32_t igc_key[4];

    /* added to allow setting aircraft ID and also an ID to ignore */
    uint32_t aircraft_id:24;
    uint8_t  id_method:2;     /* whether to use device ID, ICAO ID, or random */
    uint8_t  debug_flags:6;   /* each bit activates output of some debug info */
    uint32_t ignore_id:24;
    uint8_t  strobe:2;
    bool    logalarms:1;
    uint8_t  voice:2;
    uint8_t  tcpport:1;       /* 0=2000, 1=8880 */
    uint8_t  tcpmode:1;
    bool     ppswire:1;       /* whether T-Beam v0.7 has wire added from PPS to GPIO37 */
    uint32_t follow_id:24;

    bool     nmea2_g:1;
    bool     nmea2_p:1;
    bool     nmea2_l:1;
    bool     nmea2_s:1;
    bool     nmea2_d:1;
    uint8_t  nmea_out2:3;     /* second NMEA output route */

    char    ssid[19];
    char    psk[17];
    char    host_ip[16];

} __attribute__((packed)) settings_t;

typedef struct EEPROM_S {
    uint32_t  magic;
    uint32_t  version;
    settings_t settings;
    uint32_t  version2;    // guard from both ends
} eeprom_struct_t;

typedef union EEPROM_U {
   eeprom_struct_t field;
   uint8_t raw[sizeof(eeprom_struct_t)];
} eeprom_t;

#define DEBUG_WIND 0x01
#define DEBUG_PROJECTION 0x02
#define DEBUG_ALARM 0x04
#define DEBUG_LEGACY 0x08
#define DEBUG_RESVD1 0x10
#define DEBUG_FAKEFIX 0x20

void EEPROM_setup(void);
void EEPROM_defaults(void);
void EEPROM_store(void);
void show_settings_serial(void);
extern settings_t *settings;
extern uint32_t baudrates[];

#endif /* EEPROMHELPER_H */
