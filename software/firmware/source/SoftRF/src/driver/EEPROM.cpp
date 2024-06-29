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
#include "Buzzer.h"
#include "Bluetooth.h"
#include "../TrafficHelper.h"
#include "../protocol/radio/Legacy.h"
#include "../protocol/data/NMEA.h"
#include "../protocol/data/GDL90.h"
#include "../protocol/data/D1090.h"
#include "../protocol/data/JSON.h"
#include "Battery.h"

// start reading from the first byte (address 0) of the EEPROM

bool default_settings_used = false;
eeprom_t eeprom_block;
settings_t *settings;

bool do_alarm_demo = false;  // besides settings->alarm_demo, activated by middle button on T-Beam

uint32_t baudrates[8] = 
{
    0,
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    0
};

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

  settings = &eeprom_block.field.settings;

  if (eeprom_block.field.magic != SOFTRF_EEPROM_MAGIC) {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();
    cmd = EEPROM_EXT_DEFAULTS;
  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version  != SOFTRF_EEPROM_VERSION
    ||  settings->ssid[sizeof(settings->ssid)-1] != '\0'
    ||  settings->psk[sizeof(settings->psk)-1] != '\0'
    ||  eeprom_block.field.version2 != SOFTRF_EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));

      EEPROM_defaults();
      cmd = EEPROM_EXT_DEFAULTS;
    }
    else
      Serial.println(F("Loaded existing user settings"));
  }

  SoC->EEPROM_extension(cmd);

  settings->alarm_demo = false;   // since it is commented out in Web.cpp

  Serial.println(F("Settings:"));
  show_settings_serial();
}

void EEPROM_defaults()
{
  default_settings_used = true;

  eeprom_block.field.magic    = SOFTRF_EEPROM_MAGIC;
  eeprom_block.field.version  = SOFTRF_EEPROM_VERSION;
  eeprom_block.field.version2 = SOFTRF_EEPROM_VERSION;

  settings->mode          = SOFTRF_MODE_NORMAL;
  settings->rf_protocol   = hw_info.model == SOFTRF_MODEL_BRACELET ?
                              RF_PROTOCOL_FANET : RF_PROTOCOL_LATEST;
#if defined(DEFAULT_REGION_US)
  settings->band          = RF_BAND_US;
#else
  settings->band          = RF_BAND_EU;
#endif
  settings->aircraft_type = hw_info.model == SOFTRF_MODEL_BRACELET ?
                                              AIRCRAFT_TYPE_STATIC :
                                              AIRCRAFT_TYPE_GLIDER;
  settings->txpower       = RF_TX_POWER_FULL;
  settings->bluetooth     = BLUETOOTH_OFF;
  settings->alarm         = TRAFFIC_ALARM_LEGACY;

  /* This will speed up 'factory' boot sequence on Editions other than Standalone */
  if (hw_info.model == SOFTRF_MODEL_STANDALONE
   || hw_info.model == SOFTRF_MODEL_PRIME) {
    settings->volume  = BUZZER_OFF;
    settings->strobe  = STROBE_OFF;
    settings->pointer = DIRECTION_NORTH_UP;
  } else if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
    settings->volume  = BUZZER_VOLUME_FULL;
    settings->strobe  = STROBE_OFF;    // was STROBE_ALARM
    settings->pointer = LED_OFF;
  } else {
    settings->volume  = BUZZER_OFF;
    settings->strobe  = STROBE_OFF;
    settings->pointer = LED_OFF;
  }

  settings->voice = VOICE_OFF;

  settings->relay = RELAY_OFF;   // >>> revert to RELAY_LANDED as default eventually

  settings->nmea_g  = true;
  settings->nmea_p  = false;
  settings->nmea_l  = true;
  settings->nmea_s  = true;
  settings->nmea_d  = false;
  settings->nmea_e  = false;

  settings->nmea2_g = true;
  settings->nmea2_p = false;
  settings->nmea2_l = true;
  settings->nmea2_s = true;
  settings->nmea2_d = false;
  settings->nmea2_e = false;

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
  settings->nmea_out   = DEST_USB;
  settings->nmea_out2  = DEST_NONE;
#else
  settings->nmea_out   = hw_info.model == SOFTRF_MODEL_BADGE ?
                                             DEST_BLUETOOTH :
                                          (hw_info.model == SOFTRF_MODEL_PRIME ?
                                             DEST_UDP :
                                          (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?
                                             DEST_UDP :
                                           DEST_UART));
  settings->nmea_out2  = hw_info.model == SOFTRF_MODEL_BADGE ?
                                             DEST_USB :
                                          (hw_info.model == SOFTRF_MODEL_PRIME ?
                                             DEST_UART :
                                          (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ?
                                             DEST_UART :
                                           DEST_NONE));
#endif

  settings->gdl90_in   = DEST_NONE;
  settings->gdl90      = DEST_NONE;
#if !defined(EXCLUDE_D1090)
  settings->d1090      = DEST_NONE;
#endif
  settings->json       = JSON_OFF;
  settings->stealth    = false;
  settings->no_track   = false;
  settings->power_save = hw_info.model == SOFTRF_MODEL_BRACELET ?
                                           POWER_SAVE_NORECEIVE : POWER_SAVE_NONE;
  settings->power_external = 0;
  settings->freq_corr  = 0;
  settings->baud_rate  = BAUD_DEFAULT;      // Serial  - meaning 38400
  settings->altpin0    = false;
  settings->alt_udp    = false;
  settings->baudrate2  = BAUD_DEFAULT;      // Serial2 - meaning disabled
  settings->invert2    = false;
  settings->rx1090     = ADSB_RX_NONE;
  settings->alarm_demo = false;
  settings->igc_key[0] = 0;
  settings->igc_key[1] = 0;
  settings->igc_key[2] = 0;
  settings->igc_key[3] = 0;

  /* added to allow setting aircraft ID and also an ID to ignore */
  settings->aircraft_id = 0;
  settings->ignore_id = 0;
  settings->follow_id = 0;
  settings->id_method = ADDR_TYPE_FLARM;
  settings->logalarms = false;
  settings->debug_flags = 0;      // if and when debug output will be turned on - 0x3F for all

  strncpy(settings->ssid, MY_ACCESSPOINT_SSID, sizeof(settings->ssid)-1);
  settings->ssid[sizeof(settings->ssid)-1] = '\0';
  strncpy(settings->psk, MY_ACCESSPOINT_PSK, sizeof(settings->psk)-1);
  settings->psk[sizeof(settings->psk)-1] = '\0';
  settings->tcpmode = TCP_MODE_SERVER;
  strncpy(settings->host_ip, NMEA_TCP_IP, sizeof(settings->host_ip)-1);
  settings->host_ip[sizeof(settings->host_ip)-1] = '\0';
  settings->tcpport = 0;   // 2000
  settings->ppswire = false;   /* whether T-Beam v0.7 has wire added from PPS to GPIO37 */
}

void show_settings_serial()
{
    Serial.print(F(" Mode "));Serial.println(settings->mode);
    Serial.print(F(" Aircraft ID "));Serial.printf("%06X\r\n", settings->aircraft_id);
    Serial.print(F(" ID method "));Serial.println(settings->id_method);
    Serial.print(F(" Ignore ID "));Serial.printf("%06X\r\n", settings->ignore_id);
    Serial.print(F(" Follow ID "));Serial.printf("%06X\r\n", settings->follow_id);
    Serial.print(F(" Protocol "));Serial.println(settings->rf_protocol);
    Serial.print(F(" Band "));Serial.println(settings->band);
    Serial.print(F(" Aircraft type "));Serial.println(settings->aircraft_type);
    Serial.print(F(" Alarm trigger "));Serial.println(settings->alarm);
    Serial.print(F(" Tx Power "));Serial.println(settings->txpower);
    Serial.print(F(" Volume "));Serial.println(settings->volume);
    Serial.print(F(" Strobe "));Serial.println(settings->strobe);
    Serial.print(F(" Alarm Demo "));Serial.println(settings->alarm_demo);
    Serial.print(F(" LED pointer "));Serial.println(settings->pointer);
    Serial.print(F(" Voice "));Serial.println(settings->voice);
    Serial.print(F(" Baud 1 "));Serial.println(settings->baud_rate);
    Serial.print(F(" Alt RX pin "));Serial.println(settings->altpin0);
    Serial.print(F(" Baud 2 "));Serial.println(settings->baudrate2);
    Serial.print(F(" Invert 2 "));Serial.println(settings->invert2);
    Serial.print(F(" Alt UDP "));Serial.println(settings->alt_udp);
    Serial.print(F(" Bluetooth "));Serial.println(settings->bluetooth);
    Serial.print(F(" TCP mode "));Serial.println(settings->tcpmode);
    Serial.print(F(" TCP port "));Serial.println(settings->tcpport);
    Serial.print(F(" SSID "));Serial.println(settings->ssid);
    Serial.print(F(" PSK "));Serial.println(settings->psk);
    Serial.print(F(" Host IP "));Serial.println(settings->host_ip);
    Serial.print(F(" NMEA Out 1 "));Serial.println(settings->nmea_out);
    Serial.print(F(" NMEA GNSS "));Serial.println(settings->nmea_g);
    Serial.print(F(" NMEA Private "));Serial.println(settings->nmea_p);
    Serial.print(F(" NMEA Legacy "));Serial.println(settings->nmea_l);
    Serial.print(F(" NMEA Sensors "));Serial.println(settings->nmea_s);
    Serial.print(F(" NMEA Debug "));Serial.println(settings->nmea_d);
    Serial.print(F(" NMEA External "));Serial.println(settings->nmea_e);
    Serial.print(F(" NMEA Out 2 "));Serial.println(settings->nmea_out2);
    Serial.print(F(" NMEA2 GNSS "));Serial.println(settings->nmea2_g);
    Serial.print(F(" NMEA2 Private "));Serial.println(settings->nmea2_p);
    Serial.print(F(" NMEA2 Legacy "));Serial.println(settings->nmea2_l);
    Serial.print(F(" NMEA2 Sensors "));Serial.println(settings->nmea2_s);
    Serial.print(F(" NMEA2 Debug "));Serial.println(settings->nmea2_d);
    Serial.print(F(" NMEA2 External "));Serial.println(settings->nmea2_e);
    Serial.print(F(" ADS-B Receiver "));Serial.println(settings->rx1090);
    Serial.print(F(" GDL90 in "));Serial.println(settings->gdl90_in);
    Serial.print(F(" GDL90 out "));Serial.println(settings->gdl90);
    Serial.print(F(" DUMP1090 "));Serial.println(settings->d1090);
    Serial.print(F(" Air-Relay "));Serial.println(settings->relay);
    Serial.print(F(" Stealth "));Serial.println(settings->stealth);
    Serial.print(F(" No track "));Serial.println(settings->no_track);
    Serial.print(F(" Power save "));Serial.println(settings->power_save);
    Serial.print(F(" Power external "));Serial.println(settings->power_external);
    Serial.print(F(" Freq. correction "));Serial.println(settings->freq_corr);
    Serial.print(F(" Alarm Log "));Serial.println(settings->logalarms);
    Serial.print(F(" PPS wire "));Serial.println(settings->ppswire);
    Serial.print(F(" debug_flags "));Serial.printf("%02X\r\n", settings->debug_flags);
#if defined(USE_OGN_ENCRYPTION)
    if (settings->rf_protocol == RF_PROTOCOL_OGNTP) {
        Serial.print(" IGC key");
        Serial.printf(" %08X", (settings->igc_key[0]? 0x88888888 : 0));
        Serial.printf(" %08X", (settings->igc_key[1]? 0x88888888 : 0));
        Serial.printf(" %08X", (settings->igc_key[2]? 0x88888888 : 0));
        Serial.printf(" %08X\r\n", (settings->igc_key[3]? 0x88888888 : 0));
    }
#endif
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
