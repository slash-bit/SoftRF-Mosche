/*
 * EEPROMHelper.cpp
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

#include "../system/SoC.h"

#if defined(EXCLUDE_EEPROM)
void EEPROM_setup()    {}
void EEPROM_store()    {}
#else

#include "EEPROM.h"
#include "RF.h"
#include "LED.h"
#include "Sound.h"
#include "Bluetooth.h"
#include "../TrafficHelper.h"
#include "../protocol/radio/Legacy.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"
#include "Battery.h"

// start reading from the first byte (address 0) of the EEPROM

eeprom_t eeprom_block;
settings_t *settings;

void EEPROM_setup()
{
  int cmd = EEPROM_EXT_LOAD;

  if (!SoC->EEPROM_begin(sizeof(eeprom_t)))
  {
    Serial.print(F("ERROR: Failed to initialize "));
    Serial.print(sizeof(eeprom_t));
    Serial.println(F(" bytes of EEPROM!"));
    Serial.flush();
    delay(1000000);
  }

  for (int i=0; i<sizeof(eeprom_t); i++) {
    eeprom_block.raw[i] = EEPROM.read(i);
  }

  if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC) {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();
    cmd = EEPROM_EXT_DEFAULTS;
  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version != SOFTRF_EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

      EEPROM_defaults();
      cmd = EEPROM_EXT_DEFAULTS;
    }
  }
  settings = &eeprom_block.field.settings;

//#if defined(DEFAULT_REGION_US)
//      settings->band = RF_BAND_US;
//#endif

  SoC->EEPROM_extension(cmd);
}

void EEPROM_defaults()
{
  eeprom_block.field.magic                  = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version                = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.settings.mode          = SOFTRF_MODE_NORMAL;
  eeprom_block.field.settings.rf_protocol   = hw_info.model == SOFTRF_MODEL_BRACELET ?
                                              RF_PROTOCOL_FANET : RF_PROTOCOL_LEGACY;
#if defined(DEFAULT_REGION_US)
  eeprom_block.field.settings.band          = RF_BAND_US;
#else
  eeprom_block.field.settings.band          = RF_BAND_EU;
#endif
  eeprom_block.field.settings.aircraft_type = hw_info.model == SOFTRF_MODEL_BRACELET ?
                                              AIRCRAFT_TYPE_STATIC :
                                              AIRCRAFT_TYPE_GLIDER;
  eeprom_block.field.settings.txpower       = RF_TX_POWER_FULL;
  eeprom_block.field.settings.bluetooth     = BLUETOOTH_OFF;
  eeprom_block.field.settings.alarm         = TRAFFIC_ALARM_LEGACY;

  /* This will speed up 'factory' boot sequence on Editions other than Standalone */
  if (hw_info.model == SOFTRF_MODEL_STANDALONE
   || hw_info.model == SOFTRF_MODEL_PRIME) {
    eeprom_block.field.settings.volume      = BUZZER_VOLUME_FULL;
    eeprom_block.field.settings.pointer     = DIRECTION_NORTH_UP;
  } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    eeprom_block.field.settings.volume      = BUZZER_VOLUME_FULL;
    eeprom_block.field.settings.pointer     = LED_OFF;
  } else {
    eeprom_block.field.settings.volume      = BUZZER_OFF;
    eeprom_block.field.settings.pointer     = LED_OFF;
  }

  eeprom_block.field.settings.nmea_g     = true;
  eeprom_block.field.settings.nmea_p     = false;
  eeprom_block.field.settings.nmea_l     = true;
  eeprom_block.field.settings.nmea_s     = true;
  eeprom_block.field.settings.nmea_d     = false;

  eeprom_block.field.settings.nmea2_g     = true;
  eeprom_block.field.settings.nmea2_p     = false;
  eeprom_block.field.settings.nmea2_l     = true;
  eeprom_block.field.settings.nmea2_s     = true;
  eeprom_block.field.settings.nmea2_d     = false;

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
  eeprom_block.field.settings.nmea_out   = NMEA_USB;
  eeprom_block.field.settings.nmea_out2  = NMEA_OFF;
#else
  eeprom_block.field.settings.nmea_out   = hw_info.model == SOFTRF_MODEL_BADGE ?
                                             NMEA_BLUETOOTH :
                                          (hw_info.model == SOFTRF_MODEL_PRIME ?
                                             NMEA_UDP :
                                          (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?
                                             NMEA_UDP :
                                           NMEA_UART));
  eeprom_block.field.settings.nmea_out2  = hw_info.model == SOFTRF_MODEL_BADGE ?
                                             NMEA_USB :
                                          (hw_info.model == SOFTRF_MODEL_PRIME ?
                                             NMEA_UART :
                                          (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?
                                             NMEA_UART :
                                           NMEA_OFF));
#endif

  eeprom_block.field.settings.gdl90      = GDL90_OFF;
  eeprom_block.field.settings.d1090      = D1090_OFF;
  eeprom_block.field.settings.json       = JSON_OFF;
  eeprom_block.field.settings.stealth    = false;
  eeprom_block.field.settings.no_track   = false;
  eeprom_block.field.settings.power_save = hw_info.model == SOFTRF_MODEL_BRACELET ?
                                           POWER_SAVE_NORECEIVE : POWER_SAVE_NONE;
  eeprom_block.field.settings.power_external = 0;
  eeprom_block.field.settings.freq_corr  = 0;
  eeprom_block.field.settings.baud_rate  = BAUD_DEFAULT;
  eeprom_block.field.settings.igc_key[0] = 0;
  eeprom_block.field.settings.igc_key[1] = 0;
  eeprom_block.field.settings.igc_key[2] = 0;
  eeprom_block.field.settings.igc_key[3] = 0;

  /* added to allow setting aircraft ID and also an ID to ignore */
  eeprom_block.field.settings.aircraft_id = 0;
  eeprom_block.field.settings.ignore_id = 0;
  eeprom_block.field.settings.follow_id = 0;
  eeprom_block.field.settings.id_method = ADDR_TYPE_FLARM;
  eeprom_block.field.settings.debug_flags = 0;
}

void EEPROM_store()
{
  Serial.println("Writing EEPROM...");

  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);
  }

  SoC->EEPROM_extension(EEPROM_EXT_STORE);

  EEPROM_commit();
}

#endif /* EXCLUDE_EEPROM */
