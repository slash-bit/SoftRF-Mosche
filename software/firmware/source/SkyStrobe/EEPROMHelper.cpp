/*
 * EEPROMHelper.cpp
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

#include <stdlib.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "SoCHelper.h"
#include "NMEAHelper.h"

#include "Strobe.h"
#include "Sound.h"

#include "SkyStrobe.h"

// start reading from the first byte (address 0) of the EEPROM

eeprom_t eeprom_block;
settings_t *settings;

/* variables copied from settings */

uint8_t temp_connection;
uint8_t temp_bridge;

uint16_t  gap_alarm;
uint16_t  flashes_alarm;
uint16_t  ms_alarm;
uint16_t  period_alarm;
uint16_t  pause_alarm;

uint16_t  gap_noalarm;
uint16_t  flashes_noalarm;
uint16_t  ms_noalarm;
uint16_t  period_noalarm;
uint16_t  pause_noalarm;

uint16_t  self_test_sec;

uint16_t  hz_low;
uint16_t  hz_important;
uint16_t  hz_urgent;

uint16_t  beeps_low;
uint16_t  beeps_important;
uint16_t  beeps_urgent;

uint16_t  tone_ms_low;
uint16_t  tone_ms_important;
uint16_t  tone_ms_urgent;

uint16_t  buzz_period;     // in units of 2 ms

void compute_pauses()
{
    pause_noalarm = (period_noalarm-flashes_noalarm*(ms_noalarm+gap_noalarm)+gap_noalarm);
    pause_alarm = (period_alarm-flashes_alarm*(ms_alarm+gap_alarm)+gap_alarm);
}

void read_settings()
{
    gap_alarm = settings->gap_alarm;
    flashes_alarm = settings->flashes_alarm;
    ms_alarm = settings->ms_alarm;
    period_alarm = ((uint16_t) settings->period_alarm) << 3;    // stored in units of 8 ms

    gap_noalarm = settings->gap_noalarm;
    flashes_noalarm = settings->flashes_noalarm;
    ms_noalarm = settings->ms_noalarm;
    period_noalarm = ((uint16_t) settings->period_noalarm) << 5;  // stored in units of 32 ms

    compute_pauses();  // based on the 4 parameters above

    self_test_sec = settings->self_test_sec;

    hz_low = settings->hz_low << 5;                            // stored in units of 32 Hz
    hz_important = settings->hz_important << 5;
    hz_urgent = settings->hz_urgent << 5;

    beeps_low = settings->beeps_low;
    beeps_important = settings->beeps_important;
    beeps_urgent = settings->beeps_urgent;

    tone_ms_low = settings->tone_ms_low << 3;                  // stored in units of 8 ms
    tone_ms_important = settings->tone_ms_important << 3;
    tone_ms_urgent = settings->tone_ms_urgent << 3;

    buzz_period = ((uint16_t) settings->buzz_period) << 2;     // stored in units of 4 ms
}

uint8_t fit_byte(uint16_t i)
{
    if (i > 255)  i = 255;
    return (uint8_t) i;
}

void write_settings()
{
    settings->gap_alarm = fit_byte(gap_alarm);
    settings->flashes_alarm = fit_byte(flashes_alarm);
    settings->ms_alarm = fit_byte(ms_alarm);
    settings->period_alarm = fit_byte(period_alarm >> 3);    // stored in units of 8 ms

    settings->gap_noalarm = fit_byte(gap_noalarm);
    settings->flashes_noalarm = fit_byte(flashes_noalarm);
    settings->ms_noalarm = fit_byte(ms_noalarm);
    settings->period_noalarm = fit_byte(period_noalarm >> 5);  // stored in units of 32 ms

    settings->self_test_sec = fit_byte(self_test_sec);

    settings->hz_low       = fit_byte(hz_low >> 5);            // stored in units of 32 Hz
    settings->hz_important = fit_byte(hz_important >> 5);
    settings->hz_urgent    = fit_byte(hz_urgent >> 5);

    settings->beeps_low = fit_byte(beeps_low);
    settings->beeps_important = fit_byte(beeps_important);
    settings->beeps_urgent = fit_byte(beeps_urgent);

    settings->tone_ms_low       = fit_byte(tone_ms_low >> 3);   // stored in units of 8 ms
    settings->tone_ms_important = fit_byte(tone_ms_important >> 3);
    settings->tone_ms_urgent    = fit_byte(tone_ms_urgent >> 3);

    settings->buzz_period = fit_byte(buzz_period >> 2);      // stored in units of 4 ms
}

void default_settings()
{
    gap_alarm = STROBE_MS_GAP;
    flashes_alarm = STROBE_FLASHES_ALARM;
    ms_alarm = STROBE_MS_ALARM;
    period_alarm = STROBE_PERIOD_ALARM;

    gap_noalarm = STROBE_MS_GAP;
    flashes_noalarm = STROBE_FLASHES_NOALARM;
    ms_noalarm = STROBE_MS_NOALARM;
    period_noalarm = STROBE_PERIOD_NOALARM;

    compute_pauses();  // based on the 4 parameters above

    self_test_sec = STROBE_INITIAL_RUN;

    hz_low = ALARM_TONE_HZ_LOW;
    hz_important = ALARM_TONE_HZ_IMPORTANT;
    hz_urgent = ALARM_TONE_HZ_URGENT;

    beeps_low = ALARM_BEEPS_LOW;
    beeps_important = ALARM_BEEPS_IMPORTANT;
    beeps_urgent = ALARM_BEEPS_URGENT;

    tone_ms_low = ALARM_TONE_MS_LOW;
    tone_ms_important = ALARM_TONE_MS_IMPORTANT;
    tone_ms_urgent = ALARM_TONE_MS_URGENT;

    buzz_period = BUZZ_PERIOD;
}

void EEPROM_defaults()
{
  eeprom_block.field.magic   = SKYSTROBE_EEPROM_MAGIC;
  eeprom_block.field.version = SKYSTROBE_EEPROM_VERSION;

  settings->strobe      = STROBE_AIRBORNE;
  settings->sound       = SOUND_ON;

  settings->connection  = CON_SERIAL;
  settings->bridge      = BRIDGE_NONE;

  settings->baudrate    = B38400;
  settings->protocol    = PROTOCOL_NMEA;

  strcpy(settings->server,  DEFAULT_AP_SSID);
  strcpy(settings->key,     DEFAULT_AP_PSK);

  settings->sw1         = NO_SWITCH;
  settings->sw2         = NO_SWITCH;
  settings->swlogic     = SWITCH_OR;

  default_settings();
  write_settings();   // so they'll be written to flash when web interface saves settings
}

void EEPROM_setup()
{
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

  if (eeprom_block.field.magic != SKYSTROBE_EEPROM_MAGIC) {
    Serial.println(F("WARNING! User defined settings are not initialized yet. Loading defaults..."));

    EEPROM_defaults();

  } else {
    Serial.print(F("EEPROM version: "));
    Serial.println(eeprom_block.field.version);

    if (eeprom_block.field.version != SKYSTROBE_EEPROM_VERSION) {
      Serial.println(F("WARNING! Version mismatch of user defined settings. Loading defaults..."));
      EEPROM_defaults();

    } else {
      read_settings();
    }
  }

  temp_connection = settings->connection;
  temp_bridge = settings->bridge;
}

void EEPROM_store()
{
  /* only do this now, to prevent BT crash in preceding seconds */
  /* - assumes system will be made to reboot right after this */
  settings->connection = temp_connection;
  settings->bridge     = temp_bridge;

  for (int i=0; i<sizeof(eeprom_t); i++) {
    EEPROM.write(i, eeprom_block.raw[i]);  
  }

  EEPROM.commit();
}


/*
 * Back-door settings access:
 *
 * Commands can be sent from a USB or BT terminal,
 *    if in NMEA protocol and data bridge mode.
 * Each command is in pseudo-NMEA format:
 *    $BDxxx,vvv*
 * Checksum not needed - anything after the * is ignored.
 *    xxx is a 3-character label - it is NOT case sensitive
 *    vvv is a setting value (positive integer)
 *    the character between xxx & v is ignored
 *       - the comma can be a space or whatever
 *
 * Valid commands: (just the label listed here)
 *
 * Labels that don't need a value: (command can be "$BDTST*")
 * SHO   show the current settings
 * DEF   restore the default (not user) settings
 * USR   restore the saved user settings loaded at boot
 * TST   restart the post-boot self-test sound and light show
 * SAV   save the (possibly changed) settings and reboot
 * RBT   reboot without saving anything
 *
 * Labels that need a value:
 *  (if none or invalid value given, reports the current value)
 * GPA   gap_alarm
 * FLA   flashes_alarm
 * MSA   ms_alarm
 * PDA   period_alarm
 * GPN   gap_noalarm
 * FLN   flashes_noalarm
 * MSN   ms_noalarm
 * PDN   period_noalarm
 * STS   self_test_sec
 * HZ1   hz_low
 * HZ2   hz_important
 * HZ3   hz_urgent
 * BP1   beeps_low
 * BP2   beeps_important
 * BP3   beeps_urgent
 * TN1   tone_ms_low
 * TN2   tone_ms_important
 * TN3   tone_ms_urgent
 * BZP   buzz_period
 *
 * For example, the command "$BDFLA,5" means:
 *     do 5 quick flashes in each burst when given a collision alarm.
 *
 * After sending a command, a response is returned to the terminal app,
 * either explaining what was changed, or reporting an error.
 *
 * After changing settings, the behavior of SkyStrobe will change accordingly.
 * But to save for future boots, need to use the SAV command.
 */

void show_setting(const char *label, uint16_t value, const char *desc)
{
    snprintf(NMEAbuf, sizeof(NMEAbuf), "  %s %d\r\n", label, value);
    backdoor_write(NMEAbuf);
    if (desc != NULL) {
        snprintf(NMEAbuf, sizeof(NMEAbuf), "      - %s\r\n", desc);
        backdoor_write(NMEAbuf);
    }
}

void show_settings()
{
    show_setting("GPA", gap_alarm, NULL);
    show_setting("FLA", flashes_alarm, NULL);
    delay(200);                                 // don't congest the output buffer
    show_setting("MSA", ms_alarm, NULL);
    show_setting("PDA", period_alarm, NULL);
    delay(200);
    show_setting("GPN", gap_noalarm, NULL);
    show_setting("FLN", flashes_noalarm, NULL);
    delay(200);
    show_setting("MSN", ms_noalarm, NULL);
    show_setting("PDN", period_noalarm, NULL);
    delay(200);
    show_setting("STS", self_test_sec, NULL);
    show_setting("HZ1", hz_low, NULL);
    delay(200);
    show_setting("HZ2", hz_important, NULL);
    show_setting("HZ3", hz_urgent, NULL);
    delay(200);
    show_setting("BP1", beeps_low, NULL);
    show_setting("BP2", beeps_important, NULL);
    delay(200);
    show_setting("BP3", beeps_urgent, NULL);
    show_setting("TN1", tone_ms_low, NULL);
    delay(200);
    show_setting("TN2", tone_ms_important, NULL);
    show_setting("TN3", tone_ms_urgent, NULL);
    delay(200);
    show_setting("BZP", buzz_period, NULL);
}

void invalid()
{
    backdoor_write("SkyStrobe: invalid command\r\n");
}

void update(uint16_t *pvar, char *value)
{
    uint16_t n = 0;
    if (value != NULL)
        n = (uint16_t) strtol(value,NULL,0);
    if (n <= 0)
        return;

    *pvar = n;   // only updates the setting if given a valid value
}

bool notcmd(char *label, const char *trial,
                uint16_t *psetting, char *value, const char *msg)
{
    if (strcmp(label,trial)==0) {
        update(psetting, value);
        show_setting(label, *psetting, msg);
        return false;        // command succeeded
    }
    return true;
}

void backdoor(char *sentence, int len)
{
    /*if (len < 6) {
        invalid();
        return;
    }
    sentence[6] = '\0';
    strupr(sentence);   // excludes the value

     if (sentence[1] != 'B' || sentence[2] != 'D') {
        invalid();
        return;
    } */        // already done that in callback_buffer() in NMEAHelper

    char *label = &sentence[3];   // skips "$BD"
    label[3] = '\0';
    strupr(label);

    char *value = NULL;
    if (len > 7) {
        sentence[len] = '\0';
        value = &sentence[7];
    }

    if (strcmp(label,"SHO")==0) {
        backdoor_write("SkyStrobe: current settings:\r\n");
        show_settings();
        return;
    }
    if (strcmp(label,"DEF")==0) {
        default_settings();
        backdoor_write("SkyStrobe: restored default settings:\r\n");
        show_settings();
        return;
    }
    if (strcmp(label,"USR")==0) {
        read_settings();
        backdoor_write("SkyStrobe: restored user settings:\r\n");
        show_settings();
        return;
    }
    if (strcmp(label,"TST")==0) {
        show_settings();
        self_test_strobe = true;
        self_test_sound = true;
        StrobeSetupMarker = millis();
        backdoor_write("SkyStrobe: restarting self-test\r\n");
        return;
    }
    if (strcmp(label,"SAV")==0) {
        backdoor_write("SkyStrobe: saving current settings:\r\n");
        show_settings();
        write_settings();
        backdoor_write("SkyStrobe: rebooting in 3 seconds...\r\n");
        when_to_reboot = millis() + 3000;
        return;
    }
    if (strcmp(label,"RBT")==0) {
        read_settings();
        backdoor_write("SkyStrobe: settings as booted:\r\n");
        show_settings();
        backdoor_write("SkyStrobe: rebooting (without changes)...\r\n");
        when_to_reboot = millis() + 3000;
        return;
    }

    if (notcmd(label,"GPA",&gap_alarm,    value,"ms gap between flashes in alarm burst")
     && notcmd(label,"FLA",&flashes_alarm,value,"number of flashes in alarm burst")
     && notcmd(label,"MSA",&ms_alarm,     value,"each alarm flash length in ms")
     && notcmd(label,"PDA",&period_alarm, value,"alarm flash bursts repeat period, ms")
     && notcmd(label,"GPN",&gap_noalarm,    value,"ms gap between flashes in no=alarm burst")
     && notcmd(label,"FLN",&flashes_noalarm,value,"number of flashes in no-alarm burst")
     && notcmd(label,"MSN",&ms_noalarm,     value,"each no-alarm flash length in ms")
     && notcmd(label,"PDN",&period_noalarm, value,"no-alarm flash bursts repeat period, ms")
     && notcmd(label,"STS",&self_test_sec, value, "post-boot self-test seconds")
     && notcmd(label,"HZ1",&hz_low,      value, "tone hz for alarm level 1")
     && notcmd(label,"HZ2",&hz_important,value, "tone hz for alarm level 2")
     && notcmd(label,"HZ3",&hz_urgent,   value, "tone hz for alarm level 3")
     && notcmd(label,"BP1",&beeps_low,      value, "number of beeps for alarm level 1")
     && notcmd(label,"BP2",&beeps_important,value, "number of beeps for alarm level 2")
     && notcmd(label,"BP3",&beeps_urgent,   value, "number of beeps for alarm level 3")
     && notcmd(label,"TN1",&tone_ms_low,      value, "ms each beep for alarm level 1")
     && notcmd(label,"TN2",&tone_ms_important,value, "ms each beep for alarm level 2")
     && notcmd(label,"TN3",&tone_ms_urgent,   value, "ms each beep for alarm level 3")
     && notcmd(label,"BZP",&buzz_period,value, "on & off time for gear warning, ms"))
        invalid();
}
