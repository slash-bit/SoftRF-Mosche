/*
 * EEPROMHelper.h
 * Copyright (C) 2019-2022 Linar Yusupov
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

#if defined(ARDUINO) && !defined(ENERGIA_ARCH_CC13XX) && !defined(RASPBERRY_PI)
#include <EEPROM.h>
#endif /* ARDUINO */


#define SKYSTROBE_EEPROM_MAGIC   0xABBAFACE
#define SKYSTROBE_EEPROM_VERSION 0x00000BB2

typedef struct Settings {

    uint8_t  connection;
    uint8_t  bridge;
    uint8_t  protocol;
    uint8_t  baudrate;

    char     server  [21+1];
    char     key     [17+1];

    uint8_t  strobe;
    uint8_t  sound;

    uint8_t  sw1:2;
    uint8_t  sw2:2;
    uint8_t  swlogic:2;
    uint8_t  resvd2:2;
    uint8_t  resvd3;

    uint8_t  gap_alarm;
    uint8_t  flashes_alarm;
    uint8_t  ms_alarm;
    uint8_t  period_alarm;    // in units of 8 ms

    uint8_t  gap_noalarm;
    uint8_t  flashes_noalarm;
    uint8_t  ms_noalarm;
    uint8_t  period_noalarm;  // in units of 32 ms

    uint8_t  self_test_sec;   // sound self-test runs twice as long

    uint8_t  hz_low;          // in units of 32 hz
    uint8_t  hz_important;
    uint8_t  hz_urgent;

    uint8_t  beeps_low;
    uint8_t  beeps_important;
    uint8_t  beeps_urgent;

    uint8_t  tone_ms_low;     // in units of 8 ms
    uint8_t  tone_ms_important;
    uint8_t  tone_ms_urgent;

    uint8_t  buzz_period;     // in units of 4 ms

    uint8_t  resvd5;

    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;

} __attribute__((packed)) settings_t;

typedef struct EEPROM_S {
    uint32_t  magic;
    uint32_t  version;
    settings_t settings;
} eeprom_struct_t;


typedef union EEPROM_U {
   eeprom_struct_t field;
   uint8_t raw[sizeof(eeprom_struct_t)];
} eeprom_t;

enum
{
	STROBE_OFF = 0,
	STROBE_ALARM,
	STROBE_AIRBORNE,
	STROBE_ALWAYS
};

enum
{
	SOUND_OFF = 0,
	SOUND_ON
};

enum
{
	NO_SWITCH = 0,
	NORMALLY_OPEN,
	NORMALLY_CLOSED,
};

enum
{
	SWITCH_AND = 0,
	SWITCH_OR,
	SWITCH_XOR
};

void EEPROM_setup(void);
void EEPROM_defaults(void);
void EEPROM_store(void);
extern settings_t *settings;

extern uint8_t temp_connection;
extern uint8_t temp_bridge;

extern uint16_t  gap_alarm;
extern uint16_t  flashes_alarm;
extern uint16_t  ms_alarm;
extern uint16_t  pause_alarm;  // computed from period & the 3 above
extern uint16_t  gap_noalarm;
extern uint16_t  flashes_noalarm;
extern uint16_t  ms_noalarm;
extern uint16_t  pause_noalarm;
extern uint16_t  self_test_sec;
extern uint16_t  hz_low;
extern uint16_t  hz_important;
extern uint16_t  hz_urgent;
extern uint16_t  beeps_low;
extern uint16_t  beeps_important;
extern uint16_t  beeps_urgent;
extern uint16_t  tone_ms_low;
extern uint16_t  tone_ms_important;
extern uint16_t  tone_ms_urgent;
extern uint16_t  buzz_period;

void backdoor(char *, int);

#endif /* EEPROMHELPER_H */
