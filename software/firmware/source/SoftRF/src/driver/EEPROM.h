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
#define SOFTRF_EEPROM_VERSION 0xACAC0B0D

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
    ADSB_RX_NONE = 0,
    ADSB_RX_GNS5892 = 1,
    ADSB_RX_2,
    ADSB_RX_3
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

enum
{
	EXT_GNSS_NONE=0,
	EXT_GNSS_39_4,
	EXT_GNSS_13_2,
	EXT_GNSS_15_14
};

enum
{
	SD_CARD_NONE=0,   // SCK, MISO, MOSI, SS
	SD_CARD_13_25,    // SD card on 13,25,2,0
	SD_CARD_13_VP,    // SD card on 13,VP,2,0
	SD_CARD_LORA      // SD card on 5,19,27,0
};

enum
{
	FLIGHT_LOG_NONE=0,
	FLIGHT_LOG_ALWAYS,
	FLIGHT_LOG_AIRBORNE,
	FLIGHT_LOG_TRAFFIC
};

enum
{
	LOG_INTERVAL_1S = 0,
	LOG_INTERVAL_2S = 1,
	LOG_INTERVAL_4S = 2,
	LOG_INTERVAL_8S = 3
};

typedef struct __attribute__((packed)) Settings {

    uint8_t  mode:4;            // do not move
    uint8_t  rf_protocol:4;     // do not move
    uint8_t  band:4;            // do not move
    uint8_t  txpower:2;         // do not move
    uint8_t  volume:2;
    uint8_t  aircraft_type;     // can be reduced to 5 bits (not 4!)

    uint8_t  geoid:7;
    bool     resvd1:1;

    bool     nmea_g:1;       // do not move
    bool     nmea_p:1;
    bool     nmea_l:1;
    bool     nmea_s:1;
    bool     nmea_d:1;
    uint8_t  nmea_out:3;     // do not move

    uint8_t  bluetooth:3; // ESP32 built-in Bluetooth  // do not move
    uint8_t  alarm:3;        // do not move
    bool     stealth:1;      // do not move
    bool     no_track:1;     // do not move

    uint8_t  gdl90:3;      // output destination
    uint8_t  d1090:3;
    uint8_t  json:1;       // was 2 bits
    bool     resvd2:1;

    uint8_t  pointer:2;
    uint8_t  power_save:2;
    uint8_t  power_external:1;  /* if nonzero, shuts down if battery is not full */
    uint8_t  rx1090:2;       // attached ADS-B receiver module    // do not move
    bool     mode_s:1;

    uint8_t  gnss_pins:2;    // external GNSS added to T-Beam     // do not move
    uint8_t  sd_card:2;      // gpio pins for SD card adapter
    uint8_t  logflight:2;
    uint8_t  loginterval:2;

    int8_t   freq_corr; /* +/-, kHz */   // <<< limited to +-30, so can liberate two bits
    uint8_t  relay:2;
    uint8_t  gdl90_in:3;    // data from this port will be interpreted as GDL90
    uint8_t  alt_udp:1;     // if 1 then use 10111 instead of 10110
    bool     nmea_e:1;
    bool     nmea2_e:1;     // whether to send bridged data
    uint8_t  baud_rate:3;   // for serial UART0    // do not move
    uint8_t  baudrate2:3;   // for aux UART2       // do not move
    bool     invert2:1;     // whether to invert the logic levels on UART2
    bool     altpin0:1;     // whether to use a different pin for UART0 RX

    /* Use a key provided by (local) gliding contest organizer */
    uint32_t igc_key[4];

    /* added to allow setting aircraft ID and also an ID to ignore */
    uint32_t aircraft_id:24;  // do not move
    uint8_t  id_method:2;     // device ID, ICAO ID, or random    // do not move
    uint8_t  debug_flags:6;   /* each bit activates output of some debug info */
    uint32_t ignore_id:24;    // do not move
    uint8_t  strobe:2;
    bool    logalarms:1;
    uint8_t  voice:2;
    uint8_t  tcpport:1;       // 0=2000, 1=8880   // do not move
    uint8_t  tcpmode:1;       // do not move
    bool     ppswire:1;       // whether PPS wire added  // do not move
    uint32_t follow_id:24;    // do not move

    bool     nmea2_g:1;       // do not move
    bool     nmea2_p:1;
    bool     nmea2_l:1;
    bool     nmea2_s:1;
    bool     nmea2_d:1;
    uint8_t  nmea_out2:3;     // second NMEA output route    // do not move

    char    ssid[19];         // do not move
    char    psk[17];
    char    host_ip[16];

} settings_t;

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
#define DEBUG_DEEPER 0x10
#define DEBUG_SIMULATE 0x20

void EEPROM_setup(void);
void EEPROM_defaults(void);
void EEPROM_store(void);
void show_settings_serial(void);
void do_test_mode(void);

extern bool default_settings_used;
extern settings_t *settings;
extern uint32_t baudrates[];
extern bool do_alarm_demo;
extern bool test_mode;
extern bool landed_out_mode;
extern int8_t geoid_from_setting;

#endif /* EEPROMHELPER_H */
