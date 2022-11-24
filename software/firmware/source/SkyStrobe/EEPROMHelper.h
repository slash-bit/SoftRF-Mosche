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
#define SKYSTROBE_EEPROM_VERSION 0x00000BB1

typedef struct Settings {

    uint8_t  connection;
    uint8_t  protocol;
    uint8_t  baudrate;
    uint8_t  resvd1;

    char     server  [18];
    char     key     [18];

    uint8_t  strobe;
    uint8_t  sound;

    uint8_t  resvd3;
    uint8_t  resvd4;
    uint8_t  resvd5;
    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
} settings_t;

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

void EEPROM_setup(void);
void EEPROM_defaults(void);
void EEPROM_store(void);
extern settings_t *settings;

#endif /* EEPROMHELPER_H */
