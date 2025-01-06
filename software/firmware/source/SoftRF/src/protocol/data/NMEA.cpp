/*
 * NMEAHelper.cpp
 * Copyright (C) 2017-2022 Linar Yusupov
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

// This file originally based on v1.2.
// Major revision in Jan. 2024: buffer each input separately, for bridging.

#include <TimeLib.h>

#include "NMEA.h"
#include "GDL90.h"
#include "GNS5892.h"
#include "../../driver/GNSS.h"
#include "../../driver/RF.h"
#include "../../system/SoC.h"
// which does #include "../../SoftRF.h"
#include "../../system/Time.h"
#include "../../driver/WiFi.h"
#include "../../driver/EEPROM.h"
#include "../../driver/RF.h"
#include "../../driver/Battery.h"
#include "../../driver/Baro.h"
#if defined(ESP32)
#include "../../driver/OLED.h"
#include "../../driver/Strobe.h"
#endif
#include "../../driver/Bluetooth.h"
#if defined(USE_SD_CARD)
#include "../../driver/SDcard.h"
#endif
#include "../../TrafficHelper.h"

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

uint8_t NMEA_Source = DEST_NONE;   // identifies which port a sentence came from

char NMEABuffer[NMEA_BUFFER_SIZE]; //buffer for NMEA data
char GPGGA_Copy[NMEA_BUFFER_SIZE];   //store last $GGA sentence

//static char NMEA_Callsign[NMEA_CALLSIGN_SIZE];

#if defined(NMEA_TCP_SERVICE)

WiFiServer NmeaTCPServer(NMEA_TCP_PORT);
NmeaTCP_t NmeaTCP[MAX_NMEATCP_CLIENTS];
bool TCP_active = false;

// TCP-client code copied from OGNbase
// see also https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi/examples

// #include <ESP32Ping.h>
#include <WiFiClient.h>

WiFiClient   client;

static int WiFi_connect_TCP()
{
    // if (Ping.ping(host, 2))
    {
        if (!client.connect(settings->host_ip,
                            settings->tcpport? ALT_TCP_PORT : NMEA_TCP_PORT,
                            5000)) {
            Serial.println(F("Failed to connect as TCP client"));
            return 0;
        }
//Serial.println("Connected as TCP client");
        return 1;
    }
//Serial.println(F("TCP host not responding to ping"));
    return 0;
}

static int WiFi_disconnect_TCP()
{
    client.stop();
    return 0;
}

int WiFi_transmit_TCP(const char *buf, size_t size)
{
  if (TCP_active) {
    if (settings->tcpmode == TCP_MODE_SERVER) {
      for (uint8_t acc_ndx = 0; acc_ndx < MAX_NMEATCP_CLIENTS; acc_ndx++) {

        if (NmeaTCP[acc_ndx].client && NmeaTCP[acc_ndx].client.connected()){
          if (NmeaTCP[acc_ndx].ack) {
            NmeaTCP[acc_ndx].client.write(buf, size);
          }
        }
      }
    }
    else if (settings->tcpmode == TCP_MODE_CLIENT) {
      if (client.connected())
      {
        client.write((byte *) buf, size);
      }
    }
  }
  return 0;
}

static int WiFi_receive_TCP(char* RXbuffer, int RXbuffer_size)
{
    int i = 0;

    if (client.connected())
    {
        while (client.available() && i < RXbuffer_size - 1) {
            RXbuffer[i] = client.read();
            i++;
        }
        RXbuffer[i] = '\0';
//if ((settings->nmea_d || settings->nmea2_d)  && (settings->debug_flags & DEBUG_DEEPER)) {
//Serial.print("TCP>");
//Serial.print(RXbuffer);
//}
        return i;
    }
    client.stop();
    return -1;
}

static void WiFi_flush_TCP()
{
static bool db;
//db = ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_DEEPER));
    if (client.connected())
    {
//if (db && client.available())
//Serial.println(F("TCP_input_flushed"));
        while (client.available()) {
            char c = client.read();
            yield();
        }
        return;
    }
    client.stop();
}

static int WiFi_isconnected_TCP()
{
    return client.connected();
}

#endif // defined(NMEA_TCP_SERVICE)

#if defined(USE_NMEALIB)
#include <nmealib.h>

NmeaMallocedBuffer nmealib_buf;
#endif /* USE_NMEALIB */

const char *NMEA_CallSign_Prefix[] = {
  [RF_PROTOCOL_LEGACY]    = "FLO",
  [RF_PROTOCOL_OGNTP]     = "OGN",
  [RF_PROTOCOL_P3I]       = "PAW",
  [RF_PROTOCOL_ADSB_1090] = "ADS",
  [RF_PROTOCOL_ADSB_UAT]  = "UAT",
  [RF_PROTOCOL_FANET]     = "FAN",
  [RF_PROTOCOL_GDL90]     = "GDL",   // data from external device
  [RF_PROTOCOL_LATEST]    = "FLR"
};

const uint8_t data_source_code[] = {
    [TX_TYPE_NONE]  = 0,
    [TX_TYPE_A]     = 6,
    [TX_TYPE_C]     = 6,
    [TX_TYPE_S]     = 6,
    [TX_TYPE_TISB]  = 4,
    [TX_TYPE_ADSR]  = 3,
    [TX_TYPE_ADSB]  = 1,
    [TX_TYPE_FLARM] = 0
};

#define isTimeToPGRMZ() (millis() - PGRMZ_TimeMarker > 1000)
unsigned long PGRMZ_TimeMarker = 0;

extern uint32_t tx_packets_counter, rx_packets_counter;

#if defined(ENABLE_AHRS)
#include "../../driver/AHRSHelper.h"

#define isTimeToRPYL()  (millis() - RPYL_TimeMarker > AHRS_INTERVAL)
unsigned long RPYL_TimeMarker = 0;
#endif /* ENABLE_AHRS */

#if defined(USE_NMEA_CFG)

TinyGPSCustom C_Version;   /* 1 */
TinyGPSCustom C_Mode;
TinyGPSCustom C_Protocol;
TinyGPSCustom C_Band;
TinyGPSCustom C_AcftType;
TinyGPSCustom C_Alarm;
TinyGPSCustom C_TxPower;
TinyGPSCustom C_Volume;
TinyGPSCustom C_Pointer;
TinyGPSCustom C_NMEA_gnss; /* 10 */
TinyGPSCustom C_NMEA_private;
TinyGPSCustom C_NMEA_legacy;
TinyGPSCustom C_NMEA_sensors;
TinyGPSCustom C_NMEA_Output;
TinyGPSCustom C_GDL90_Output;
TinyGPSCustom C_D1090_Output;
TinyGPSCustom C_Stealth;
TinyGPSCustom C_noTrack;
TinyGPSCustom C_PowerSave; /* 19 */

// additional settings added by MB
TinyGPSCustom D_Version;   /* 1 */
TinyGPSCustom D_id_method;
TinyGPSCustom D_aircraft_id;
TinyGPSCustom D_ignore_id;
TinyGPSCustom D_follow_id;
TinyGPSCustom D_baud_rate;
TinyGPSCustom D_power_ext;
TinyGPSCustom D_NMEA_debug;
TinyGPSCustom D_debug_flags;
TinyGPSCustom D_NMEA2;    /* 10 */
TinyGPSCustom D_NMEA2_gnss;
TinyGPSCustom D_NMEA2_private;
TinyGPSCustom D_NMEA2_legacy;
TinyGPSCustom D_NMEA2_sensors;
TinyGPSCustom D_NMEA2_debug;
TinyGPSCustom D_relay;
TinyGPSCustom D_bluetooth;  /* 17 */
TinyGPSCustom D_baudrate2;
TinyGPSCustom D_invert2;
TinyGPSCustom D_extern1;
TinyGPSCustom D_extern2;  /* 21 */
TinyGPSCustom D_altpin0;
TinyGPSCustom D_voice;
TinyGPSCustom D_strobe;

TinyGPSCustom F_Version;   /* 1 */

TinyGPSCustom T_testmode;

#if defined(USE_OGN_ENCRYPTION)
/* Security and privacy */
TinyGPSCustom S_Version;
TinyGPSCustom S_IGC_Key;
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
#include "../../driver/EPD.h"

TinyGPSCustom V_Version; /* 1 */
TinyGPSCustom V_Adapter;
TinyGPSCustom V_Connection;
TinyGPSCustom V_Units;
TinyGPSCustom V_Zoom;
TinyGPSCustom V_Protocol;
TinyGPSCustom V_Baudrate;
TinyGPSCustom V_Server;
TinyGPSCustom V_Key;
TinyGPSCustom V_Rotate;  /* 10 */
TinyGPSCustom V_Orientation;
TinyGPSCustom V_AvDB;
TinyGPSCustom V_ID_Pref;
TinyGPSCustom V_VMode;
TinyGPSCustom V_Voice;
TinyGPSCustom V_AntiGhost;
TinyGPSCustom V_Filter;
TinyGPSCustom V_PowerSave;
TinyGPSCustom V_Team;    /* 19 */
#endif /* USE_SKYVIEW_CFG */
#endif /* USE_NMEA_CFG */

static char *ltrim(char *s)
{
  if(s) {
    while(*s && isspace(*s))
      ++s;
  }

  return s;
}

// if '*' found, add checksum (and \r\n) after it.
// otherwise do not change the string but return its length.
unsigned int NMEA_add_checksum(char *buf)
{
  //calculate the checksum - get sentence length as a side effect
  unsigned int n;
  char c;
  unsigned char cs = 0;
  for (n = 1; n < NMEA_BUFFER_SIZE - 5; n++) {
    c = buf[n];
    if (c == '*')
        break;
    if (c == '\0')
        break;
    cs ^= c;
  }
//Serial.print("' - n:");
//Serial.println(n);
  if (c == '*') {
      char *csum_ptr = &buf[n+1];
      snprintf_P(csum_ptr, 5, PSTR("%02X\r\n"), cs);
//Serial.print("checksum added: ");
//Serial.print(buf);
      return (n + 5);
  }
  if (c == '\0')
      return n;
  return 0;
}

void sendPFLAJ()
{
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAJ,A,%d,%d,0*"),
                 ThisAircraft.airborne, ThisAircraft.airborne);
    int nmealen = NMEA_add_checksum();
    NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, nmealen, false);
}

// send self-test and version sentences out, imitating a FLARM
void sendPFLAV(bool nowait)
{
  static uint32_t whensent = 0;
  if (!nowait) { 
      if (OurTime == whensent)
          return;                 // only continue once per second
      whensent = OurTime;
      if (OurTime < 1000000)
          return;
      if ((((uint32_t) OurTime) & 0x1F) != 7)   // send once in 32 seconds
          return;
  }
  uint32_t timebits = (((uint32_t) OurTime) & 0x60);
  if (settings->nmea_l || settings->nmea2_l) {
    if (nowait || timebits == 0x20) {
      uint32_t pps = SoC->get_PPS_TimeMarker();
      snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAE,A,0,0%s*"),
          (pps > 0 && millis()-pps < 2000)? ",PPS received" : "");
      int nmealen = NMEA_add_checksum();
      NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, nmealen, false);
    }
    if (nowait || timebits == 0x60) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAV,A,2.4,7.24,%s-%s*"),
                   SOFTRF_IDENT, SOFTRF_FIRMWARE_VERSION);  // our version in obstacle db text field
      int nmealen = NMEA_add_checksum();
      NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, nmealen, false);
    }
//  if (nowait || timebits == 0x00 || timebits == 0x40) {    // every 64 seconds
//      sendPFLAJ();
//  }
  }
}

// copy into plain static variables for efficiency in NMEA_Loop():
bool is_a_prime_mk2 = false;
bool has_serial2 = false;
bool rx1090found = false;
static unsigned int UDP_NMEA_Output_Port = NMEA_UDP_PORT;

void NMEA_setup()
{
  if (settings->alt_udp)
    UDP_NMEA_Output_Port = NMEA_UDP_PORT2;  // or use ALT_UDP_PORT (4352)?

#if defined(ESP32)
  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 /* && hw_info.revision >= 8 */) {

    is_a_prime_mk2 = true;

    // use Serial2 as an auxillary port for data bridging (or ADS-B data input)
    uint32_t Serial2Baud = baudrates[settings->baudrate2];
    size_t Serial2BufSize = SerialBufSize;
    if (settings->rx1090 == ADSB_RX_GNS5892) {
        Serial2Baud = GNS5892_BAUDRATE;
        Serial2BufSize = GNS5892_INPUT_BUF_SIZE;
        Serial.println(F("Using Serial2 for ADS-B module"));
    }

    if (Serial2Baud == 0 || settings->gnss_pins == EXT_GNSS_39_4) {
        // note BAUD_DEFAULT here means Serial2 disabled, not 38400
        Serial.println(F("Serial2 NOT started"));
    } else {
        Serial2.setRxBufferSize(Serial2BufSize);
        uint8_t rx_pin = Serial2RxPin;   // VN
        if (hw_info.revision < 8)
            rx_pin = Serial0AltRxPin;    // VP
        const char *label = "Aux Serial";
        if (settings->rx1090)
            label = "ADS-B Module";
        if (ESP32_pin_reserved(rx_pin, false, label) == false
         && ESP32_pin_reserved(Serial2TxPin, false, label) == false) {
            Serial2.begin(Serial2Baud, SERIAL_8N1, rx_pin, Serial2TxPin, settings->invert2);
            has_serial2 = true;
            Serial.printf("Serial2 started on pins %d,%d at baud rate %d, logic:%d\r\n",
               rx_pin, Serial2TxPin, Serial2Baud, settings->invert2);
            delay(500);
            int limit = 199;
            if (Serial2.available() > 0) {
                Serial.println("Input data available on Serial2:");
                while (Serial2.available() > 0 && --limit > 0)
                    Serial.write(Serial2.read());
                Serial.println("...");
            }
        }
    }
  }
#endif

#if defined(USE_NMEA_CFG)
  const char *psrf_c = "PSRFC";
  int term_num = 1;

  C_Version.begin      (gnss, psrf_c, term_num++); /* 1 */
  C_Mode.begin         (gnss, psrf_c, term_num++);
  C_Protocol.begin     (gnss, psrf_c, term_num++);
  C_Band.begin         (gnss, psrf_c, term_num++);
  C_AcftType.begin     (gnss, psrf_c, term_num++);
  C_Alarm.begin        (gnss, psrf_c, term_num++);
  C_TxPower.begin      (gnss, psrf_c, term_num++);
  C_Volume.begin       (gnss, psrf_c, term_num++);
  C_Pointer.begin      (gnss, psrf_c, term_num++);
  C_NMEA_gnss.begin    (gnss, psrf_c, term_num++); /* 10 */
  C_NMEA_private.begin (gnss, psrf_c, term_num++);
  C_NMEA_legacy.begin  (gnss, psrf_c, term_num++);
  C_NMEA_sensors.begin (gnss, psrf_c, term_num++);
  C_NMEA_Output.begin  (gnss, psrf_c, term_num++);
  C_GDL90_Output.begin (gnss, psrf_c, term_num++);
  C_D1090_Output.begin (gnss, psrf_c, term_num++);
  C_Stealth.begin      (gnss, psrf_c, term_num++);
  C_noTrack.begin      (gnss, psrf_c, term_num++);
  C_PowerSave.begin    (gnss, psrf_c, term_num  ); /* 19 */

  const char *psrf_d = "PSRFD";
  term_num = 1;

  D_Version.begin       (gnss, psrf_d, term_num++); /* 1 */
  D_id_method.begin     (gnss, psrf_d, term_num++);
  D_aircraft_id.begin   (gnss, psrf_d, term_num++);
  D_ignore_id.begin     (gnss, psrf_d, term_num++);
  D_follow_id.begin     (gnss, psrf_d, term_num++);
  D_baud_rate.begin     (gnss, psrf_d, term_num++);
  D_power_ext.begin     (gnss, psrf_d, term_num++);
  D_NMEA_debug.begin    (gnss, psrf_d, term_num++);
  D_debug_flags.begin   (gnss, psrf_d, term_num++);
  D_NMEA2.begin         (gnss, psrf_d, term_num++); /* 10 */
  D_NMEA2_gnss.begin    (gnss, psrf_d, term_num++);
  D_NMEA2_private.begin (gnss, psrf_d, term_num++);
  D_NMEA2_legacy.begin  (gnss, psrf_d, term_num++);
  D_NMEA2_sensors.begin (gnss, psrf_d, term_num++);
  D_NMEA2_debug.begin   (gnss, psrf_d, term_num++);
  D_relay.begin         (gnss, psrf_d, term_num++);
  D_bluetooth.begin     (gnss, psrf_d, term_num++);
  D_baudrate2.begin     (gnss, psrf_d, term_num++);
  D_invert2.begin       (gnss, psrf_d, term_num++);
  D_extern1.begin       (gnss, psrf_d, term_num++);
  D_extern2.begin       (gnss, psrf_d, term_num++); /* 21 */
  D_altpin0.begin       (gnss, psrf_d, term_num++);
  D_voice.begin         (gnss, psrf_d, term_num++);
  D_strobe.begin        (gnss, psrf_d, term_num++);

  const char *psrf_f = "PSRFF";
  term_num = 1;
  F_Version.begin       (gnss, psrf_f, term_num++); /* 1 */

  const char *psrf_t = "PSRFT";
  term_num = 1;
  T_testmode.begin      (gnss, psrf_t, term_num++); /* 1 */

#if defined(USE_OGN_ENCRYPTION)
/* Security and privacy */
  const char *psrf_s = "PSRFS";
  term_num = 1;

  S_Version.begin      (gnss, psrf_s, term_num++);
  S_IGC_Key.begin      (gnss, psrf_s, term_num  );
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
  const char *pskv_c = "PSKVC";
  term_num = 1;

  V_Version.begin      (gnss, pskv_c, term_num++); /* 1 */
  V_Adapter.begin      (gnss, pskv_c, term_num++);
  V_Connection.begin   (gnss, pskv_c, term_num++);
  V_Units.begin        (gnss, pskv_c, term_num++);
  V_Zoom.begin         (gnss, pskv_c, term_num++);
  V_Protocol.begin     (gnss, pskv_c, term_num++);
  V_Baudrate.begin     (gnss, pskv_c, term_num++);
  V_Server.begin       (gnss, pskv_c, term_num++);
  V_Key.begin          (gnss, pskv_c, term_num++);
  V_Rotate.begin       (gnss, pskv_c, term_num++); /* 10 */
  V_Orientation.begin  (gnss, pskv_c, term_num++);
  V_AvDB.begin         (gnss, pskv_c, term_num++);
  V_ID_Pref.begin      (gnss, pskv_c, term_num++);
  V_VMode.begin        (gnss, pskv_c, term_num++);
  V_Voice.begin        (gnss, pskv_c, term_num++);
  V_AntiGhost.begin    (gnss, pskv_c, term_num++);
  V_Filter.begin       (gnss, pskv_c, term_num++);
  V_PowerSave.begin    (gnss, pskv_c, term_num++);
  V_Team.begin         (gnss, pskv_c, term_num  ); /* 19 */
#endif /* USE_SKYVIEW_CFG */
#endif /* USE_NMEA_CFG */

#if defined(NMEA_TCP_SERVICE)
  if (settings->nmea_out  == DEST_TCP
   || settings->nmea_out2 == DEST_TCP
   || settings->gdl90_in  == DEST_TCP
   || settings->gdl90     == DEST_TCP) {
    // note: no TCP input possible unless TCP output destination is active
    if (settings->tcpmode == TCP_MODE_SERVER) {
        NmeaTCPServer.begin();
        Serial.print(F("NMEA TCP server has started at port: "));
        Serial.println(NMEA_TCP_PORT);
        NmeaTCPServer.setNoDelay(true);
        TCP_active = true;
    } else if (settings->tcpmode == TCP_MODE_CLIENT) {
        if (WiFi_connect_TCP()) {
            Serial.print(F("Connected as TCP client to host: "));
            Serial.println(settings->host_ip);
            TCP_active = true;
        } else {
            Serial.print(F("Failed to connect to host: "));
            Serial.println(settings->host_ip);
        }
    }
  }
#endif /* NMEA_TCP_SERVICE */

#if defined(USE_NMEALIB)
  memset(&nmealib_buf, 0, sizeof(nmealib_buf));
#endif /* USE_NMEALIB */

  memset(GPGGA_Copy, 0, sizeof(GPGGA_Copy));

  PGRMZ_TimeMarker = millis();

#if defined(ENABLE_AHRS)
  RPYL_TimeMarker = millis();
#endif /* ENABLE_AHRS */

  sendPFLAV(true);
}

void NMEA_Out(uint8_t dest, const char *buf, size_t size, bool nl)
{
#if 0
  Serial.print("NMEA_Out(");
  Serial.print(dest);
  Serial.print("): ");
  Serial.write(buf, size);
  if (nl) Serial.print("\r\n");
#endif

  if (dest == NMEA_Source)          // do not echo NMEA back to its source
    return;                         // NMEA_Source = DEST_NONE for internal NMEA

  if (dest == settings->gdl90_in)   // do not send NMEA to GDL90 source
    return;

  switch (dest)
  {
  case DEST_UART:
    {
      if (SoC->UART_ops) {
        SoC->UART_ops->write((const byte*) buf, size);
        if (nl)
          SoC->UART_ops->write((const byte *) "\r\n", 2);
      } else {
        Serial.write(buf, size);
        if (nl)
          Serial.write((const byte *) "\r\n",2);
      }
    }
    break;
  case DEST_UART2:
    {
      if (has_serial2) {
        Serial2.write(buf, size);
        if (nl)
          Serial2.write((const byte *) "\r\n",2);
      }
    }
    break;
  case DEST_UDP:
    {
      size_t udp_size = size;

      if (size > sizeof(UDPpacketBuffer) - 2)
        udp_size = sizeof(UDPpacketBuffer) - 2;
      memcpy(UDPpacketBuffer, buf, udp_size);

      if (nl) {
        UDPpacketBuffer[udp_size++] = '\r';
        UDPpacketBuffer[udp_size++] = '\n';
      }

      SoC->WiFi_transmit_UDP(UDP_NMEA_Output_Port, (byte *) UDPpacketBuffer, udp_size);
    }
    break;
  case DEST_TCP:
    {
#if defined(NMEA_TCP_SERVICE)
      if (TCP_active) {
        WiFi_transmit_TCP(buf, size);
        if (nl)
          WiFi_transmit_TCP("\r\n", 2);
      }
#endif
    }
    break;
  case DEST_USB:
    {
      if (SoC->USB_ops) {
        SoC->USB_ops->write((const byte *) buf, size);
        if (nl)
          SoC->USB_ops->write((const byte *) "\r\n", 2);
      }
    }
    break;
  case DEST_BLUETOOTH:
    {
      if (BTactive && SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->write((const byte *) buf, size);
        if (nl)
          SoC->Bluetooth_ops->write((const byte *) "\r\n", 2);
      }
    }
    break;
  case DEST_NONE:
  default:
    break;
  }
  yield();
}

void NMEA_Outs(bool out1, bool out2, const char *buf, size_t size, bool nl) {
    if (out1)
        NMEA_Out(settings->nmea_out,  buf, size, nl);
    if (out2)
        NMEA_Out(settings->nmea_out2, buf, size, nl);
}

void NMEAOutD()
{
    NMEA_Outs(settings->nmea_d, settings->nmea2_d, NMEABuffer, strlen(NMEABuffer), false);
}

bool NMEA_encode(const char *buf, const int len)
{
    for (int i=0; i<len; i++) {
        if (gnss.encode(buf[i]))     // valid sentence
            return true;
    }
    return false;
}

// Send buffered sentences to bridged outputs
bool NMEA_bridge_sent = false;
void NMEA_bridge_send(char *buf, int len)
{
#if 0
    Serial.print("bridge_send(");
    Serial.print(NMEA_Source);
    Serial.print("): ");
    Serial.write(buf, len);
#endif
    // First check whether it is GNSS or FLARM sentences, skip them (assume echo)
    if (buf[1]=='G' && buf[2]=='P')
        return;
    if (buf[1]=='P' && buf[2]=='F' && buf[3]=='L' && buf[4]=='A')
        return;

#if defined(USE_NMEA_CFG)
    // Also trap PSRF/PSKV config sentences, process internally instead
    if (buf[1]=='P' && buf[2]=='S') {
      if ((buf[3]=='R' && buf[4]=='F') || (buf[3]=='K' && buf[4]=='V')) {
        NMEA_bridge_sent = true;   // not really sent, but substantial processing
        // if checksum is wrong, send it back to the source:
        if (buf[len-5] == '*') {
            char c1 = buf[len-4];
            char c2 = buf[len-3];
            buf[len-4] = '\0';   // include the '*'
            len = NMEA_add_checksum(buf);
            if (buf[len-4] != c1 || buf[len-3] != c2) {
                uint8_t dest = NMEA_Source;
                NMEA_Source = DEST_NONE;
                NMEA_Out(dest, "-- correct checksum is: ", 24, false);
                NMEA_Out(dest, buf, len, false);
                NMEA_Source = dest;
                return;
            }
        }
        if (NMEA_encode(buf, len)) {           // valid sentence
            NMEA_Process_SRF_SKV_Sentences();
            return;
        }
      }
    }
#endif

    // send to configured outputs, but do not echo to source
    if (settings->nmea_e && settings->nmea_out != NMEA_Source) {
        NMEA_Out(settings->nmea_out, buf, len, false);
        NMEA_bridge_sent = true;
    }
    if (settings->nmea2_e && settings->nmea_out2 != NMEA_Source) {
        NMEA_Out(settings->nmea_out2, buf, len, false);
        NMEA_bridge_sent = true;
    }
}

// set up separate buffers for potentially-bridged output from each input
// common code for all these buffers:
void NMEA_bridge_buf(char c, char* buf, int& n)
{
    if (c == '$') {
        n = 0;
        // start new sentence, drop any preceding data
        // fall through to buf[n++] = c;
    } else if (n == 0) {      // wait for a '$' (or '!')
        if (c != '!')
            return;
        // if '!', start new sentence of some related protocols
        // fall through to buf[n++] = c;
    } else if (c=='\r' || c=='\n') {
        if (n > 5 && n <= 128) {
            // sentences missing "*xx" ending are ignored unless started with '!'
            // >>> or could forward all sentences even without checksum?
            if (buf[0] == '!' || buf[n-3] == '*') {
                buf[n++] = '\r';
                buf[n++] = '\n';      // add a proper line-ending
                buf[n]   = '\0';
                NMEA_bridge_send(buf, n);
            }
        }
        n = 0;
        return;
    } else if (n >= 128) {
        n = 0;
        return;
    }
    buf[n++] = c;
}

void NMEA_loop()
{
  NMEA_bridge_sent = false;

  NMEA_Source = DEST_NONE;

#if defined(ESP32)

  if (is_a_prime_mk2) {

    /*
     * Check SW/HW UARTs, USB, TCP and BT for data
     */

    bool gdl90 = false;

    if ((settings->debug_flags & DEBUG_SIMULATE) == 0
#if defined(USE_SD_CARD)
        || SIMfileOpen
#endif
        || (settings->gnss_pins != EXT_GNSS_NONE)) {

      // if not reading sim data from Serial, poll Serial here:

#if 0
      static char usb_buf[128+3];
      static int usb_n = 0;
      if (SoC->USB_ops) {
        gdl90 = (settings->gdl90_in == DEST_USB);
        while (SoC->USB_ops->available() > 0) {
            NMEA_Source = DEST_USB;
            int c = SoC->USB_ops->read();
            if (gdl90)
                GDL90_bridge_buf(c, usb_buf, usb_n);
            else
                NMEA_bridge_buf(c, usb_buf, usb_n);
        }
      }
#endif

      static char uart_buf[128+3];
      static int uart_n = 0;
      gdl90 = (settings->gdl90_in == DEST_UART);
      while (Serial.available() > 0) {
          NMEA_Source = DEST_UART;
          int c = Serial.read();
          if (gdl90)
              GDL90_bridge_buf(c, uart_buf, uart_n);
          else
              NMEA_bridge_buf(c, uart_buf, uart_n);
      }

    }  // end of polling Serial

    static char uart2_buf[128+3];
    static int uart2_n = 0;
    if (has_serial2) {
        if (settings->rx1090 == ADSB_RX_GNS5892) {
            // Serial2 is dedicated to the ADS-B receiver module
            gns5892_loop();
        } else {
            gdl90 = (settings->gdl90_in == DEST_UART2);
            while (Serial2.available() > 0) {
                NMEA_Source = DEST_UART2;
                int c = Serial2.read();
                if (gdl90)
                    GDL90_bridge_buf(c, uart2_buf, uart2_n);
                else
                    NMEA_bridge_buf(c, uart2_buf, uart2_n);
            }
        }
        if (NMEA_bridge_sent) {  // also set by GDL90_bridge_buf() or gns5892_loop()
            yield();
            return;           // check wireless inputs next time around
        }
    }

    static char bt_buf[128+3];
    static int bt_n = 0;
    if (SoC->Bluetooth_ops) {
      gdl90 = (settings->gdl90_in == DEST_BLUETOOTH);
      while (BTactive && SoC->Bluetooth_ops->available() > 0) {
          NMEA_Source = DEST_BLUETOOTH;
          int c = SoC->Bluetooth_ops->read();
          if (gdl90)
              GDL90_bridge_buf(c, bt_buf, bt_n);
          else
              NMEA_bridge_buf(c, bt_buf, bt_n);
      }
    }

    if (NMEA_bridge_sent) {
        yield();
        return;     // probably not using other wireless, but in any case postpone
    }

    static char udp_buf[128+3];
    static int udp_n = 0;
    while (udp_is_ready) {
      gdl90 = (settings->gdl90_in == DEST_UDP);
      size_t size = WiFi_Receive_UDP((uint8_t *) UDPpacketBuffer, sizeof(UDPpacketBuffer));
      if (size <= 0)
          break;
      NMEA_Source = DEST_UDP;
      for (size_t i=0; i < size; i++) {
          char c = UDPpacketBuffer[i];
          if (gdl90)
              GDL90_bridge_buf(c, udp_buf, udp_n);
          else
              NMEA_bridge_buf(c, udp_buf, udp_n);
      }
    }

#if defined(NMEA_TCP_SERVICE)
    static char tcpinbuf[128];
    static char tcpoutbuf[MAX_NMEATCP_CLIENTS][128+3];
    static int tcp_n[MAX_NMEATCP_CLIENTS] = {0};
//  static bool tcp_n_init = false;
    if (TCP_active) {
      gdl90 = (settings->gdl90_in == DEST_TCP);
      if (settings->tcpmode == TCP_MODE_CLIENT) {
        while (1) {
          int n = WiFi_receive_TCP(tcpinbuf, 128);
          if (n <= 0)
              break;
          NMEA_Source = DEST_TCP;
          for (int i=0; i<n; i++) {
               if (gdl90)
                   GDL90_bridge_buf(tcpinbuf[i], tcpoutbuf[0], tcp_n[0]);
               else
                   NMEA_bridge_buf(tcpinbuf[i], tcpoutbuf[0], tcp_n[0]);
          }
        }
      }
      else if (settings->tcpmode == TCP_MODE_SERVER) {
        // Note: this may collect input from more than one client.  To keep their sentences
        // from getting mixed need MAX_NMEATCP_CLIENTS (which is just 2) separate buffers.
        // Note: same NMEA_Source for all clients, won't forward from one to another
        for (int i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
          if (NmeaTCP[i].client && NmeaTCP[i].client.connected()) {
//            if (! tcp_n_init)
//              tcp_n[i] = 0;
            while (NmeaTCP[i].client.available()) {
                NMEA_Source = DEST_TCP;
                int c = NmeaTCP[i].client.read();
                if (gdl90)
                    GDL90_bridge_buf(c, tcpoutbuf[i], tcp_n[i]);
                else
                    NMEA_bridge_buf(c, tcpoutbuf[i], tcp_n[i]);
            }
          }
        }
//        tcp_n_init = true; 
      }
    }
#endif

    if (NMEA_bridge_sent) {
        yield();
        return;           // process sensors next time around
    }

  }  // end if (is_a_prime_mk2)

#endif

  NMEA_Source = DEST_NONE;  // for all internal messages sent below

  sendPFLAV(false);

  if ((settings->nmea_s || settings->nmea2_s)
      && ThisAircraft.pressure_altitude != 0.0 && isTimeToPGRMZ()) {

    int altitude = constrain(
            (int) (ThisAircraft.pressure_altitude * _GPS_FEET_PER_METER),
            -1000, 60000);

    /* https://developer.garmin.com/downloads/legacy/uploads/2015/08/190-00684-00.pdf */
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PGRMZ,%d,f,%c*"),
               altitude, isValidGNSSFix() ? '3' : '1'); /* feet , 3D fix */

    unsigned int nmealen = NMEA_add_checksum();
    NMEA_Outs(settings->nmea_s, settings->nmea2_s, NMEABuffer, nmealen, false);

#if !defined(EXCLUDE_LK8EX1)
    char str_Vcc[6];
    dtostrf(Battery_voltage(), 3, 1, str_Vcc);

    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$LK8EX1,999999,%d,%d,%d,%s*"),
            constrain((int) ThisAircraft.pressure_altitude, -1000, 99998), /* meters */
            (int) ((ThisAircraft.vs * 100) / (_GPS_FEET_PER_METER * 60)),  /* cm/s   */
            constrain((int) Baro_temperature(), -99, 98),                  /* deg. C */
            str_Vcc);

    nmealen = NMEA_add_checksum();
    NMEA_Outs(settings->nmea_s, settings->nmea2_s, NMEABuffer, nmealen, false);

#endif /* EXCLUDE_LK8EX1 */

    PGRMZ_TimeMarker = millis();
  }

#if defined(ENABLE_AHRS)
  if ((settings->nmea_s  || settings->nmea2_s) && isTimeToRPYL()) {

    AHRS_NMEA();

    RPYL_TimeMarker = millis();
  }
#endif /* ENABLE_AHRS */

#if defined(NMEA_TCP_SERVICE)

    if (TCP_active && settings->tcpmode == TCP_MODE_SERVER) {

    // TCP clients housekeeping:
  
    int i;

    if (NmeaTCPServer.hasClient()) {
      for(i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
        // find free/disconnected spot
        if (!NmeaTCP[i].client || !NmeaTCP[i].client.connected()) {
          if(NmeaTCP[i].client) {
            NmeaTCP[i].client.stop();
            NmeaTCP[i].connect_ts = 0;
          }
          NmeaTCP[i].client = NmeaTCPServer.available();
          NmeaTCP[i].connect_ts = OurTime;
          NmeaTCP[i].ack = false;
          NmeaTCP[i].client.print(F("PASS?"));
          break;
        }
      }
      if (i >= MAX_NMEATCP_CLIENTS) {
        // no free/disconnected spot so reject
        NmeaTCPServer.available().stop();
      }
    }

    for (i = 0; i < MAX_NMEATCP_CLIENTS; i++) {
      if (NmeaTCP[i].client && NmeaTCP[i].client.connected() &&
         !NmeaTCP[i].ack && NmeaTCP[i].connect_ts > 0 &&
         (OurTime - NmeaTCP[i].connect_ts) >= NMEATCP_ACK_TIMEOUT) {

          if (! is_a_prime_mk2) {
              /* Clean TCP input buffer from any pass codes sent by client */
              while (NmeaTCP[i].client.available()) {
                char c = NmeaTCP[i].client.read();
                yield();
              }
          }

          /* send acknowledge */
          NmeaTCP[i].client.print(F("AOK"));
          NmeaTCP[i].ack = true;
      }
    }
  }
#endif

}

void NMEA_fini()
{
#if defined(NMEA_TCP_SERVICE)
  if (TCP_active) {
    if (settings->tcpmode == TCP_MODE_SERVER)
        NmeaTCPServer.stop();
    else if (settings->tcpmode == TCP_MODE_CLIENT)
        WiFi_disconnect_TCP();
  }
#endif /* NMEA_TCP_SERVICE */
}

void NMEA_Export()
{
    if (! settings->nmea_l && ! settings->nmea2_l)
         return;

    NMEA_Source = DEST_NONE;

    container_t *cip, *fop;
    int alt_diff;
    int abs_alt_diff;
    float distance;
    float adj_dist;
    float bearing;

    int alarm_level = ALARM_LEVEL_NONE;
    uint32_t follow_id = settings->follow_id;

    /* High priority object (most relevant target) */
    int HP_index = MAX_TRACKING_OBJECTS;
    int HP_alt_diff = 0;
    int HP_alarm_level = ALARM_LEVEL_NONE;
    float HP_adj_dist  = 999999999;
    float HP_distance  = 999999999;
    float HP_bearing = 0;
    uint32_t HP_addr = 0;
    bool HP_nondir = false;
    bool HP_stealth = false;
    int total_objects = 0;
    int head = 0;

    bool has_Fix = (isValidFix() || (settings->mode == SOFTRF_MODE_TXRX_TEST));
    bool deeper = (settings->debug_flags & DEBUG_DEEPER);

    if (has_Fix) {

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        cip = &Container[i];

        if (cip->addr && ((OurTime - cip->timestamp) <= EXPORT_EXPIRATION_TIME)) {
#if 0
          Serial.println(i);
          Serial.printf("%06X\r\n", cip->addr);
          Serial.println(cip->latitude, 4);
          Serial.println(cip->longitude, 4);
          Serial.println(cip->altitude);
          Serial.println(cip->addr_type);
          Serial.println(cip->vs);
          Serial.println(cip->aircraft_type);
          Serial.println(cip->stealth);
          Serial.println(cip->no_track);
#endif
          bool stealth = (cip->stealth || ThisAircraft.stealth);  /* reciprocal */

          alarm_level = cip->alarm_level;

          distance = cip->distance;
          bearing = cip->bearing;

          alt_diff = (int) cip->alt_diff;                 /* sent to NMEA */
          abs_alt_diff = (int) fabs(cip->adj_alt_diff);   /* used to pick high-priority traffic */
          adj_dist = cip->adj_distance;

          bool show = true;

          /* mask some data following FLARM protocol: */
          if (stealth && alarm_level <= ALARM_LEVEL_CLOSE) {
            if (distance > STEALTH_DISTANCE || abs(alt_diff) > STEALTH_VERTICAL) {
                show = false;
            }
          }

       /* if (cip->protocol == RF_PROTOCOL_LEGACY && cip->airborne == 0)
                   show = false; */

          if ((alarm_level > ALARM_LEVEL_NONE
               || (distance < ALARM_ZONE_NONE && abs_alt_diff < VERTICAL_VISIBILITY_RANGE)
               || cip->addr == follow_id || deeper)
              && show) {

             /* put candidate traffic to report into a sorted list */

             int next, previous;
             if (total_objects == 0) {  /* first inserted into the list */
               head = i;
               cip->next = MAX_TRACKING_OBJECTS;
               total_objects = 1;
             } else {
               next = head;
               while (next < MAX_TRACKING_OBJECTS) {
                 fop = &Container[next];
                 if (fop->alarm_level <= alarm_level
                  && fop->addr != follow_id
                  && fop->adj_distance >= adj_dist)
                        break;   /* insert before this one */
                 /* else */
                   previous = next;  /* the preceding list entry */
                   next = fop->next;
               }
               cip->next = next;
               if (head == next)
                 head = i;
               else
                 Container[previous].next = i;
               total_objects++;
             }

             /* Alarm or close traffic is treated as highest priority */
             if (alarm_level > HP_alarm_level ||
                    (alarm_level == HP_alarm_level && adj_dist <= HP_adj_dist)) {  // was <
                HP_bearing = bearing;
                HP_alt_diff = alt_diff;
                HP_alarm_level = alarm_level;
                HP_distance = distance;
                HP_adj_dist = adj_dist;
                HP_addr = (stealth? 0xFFFFF0 + i : cip->addr);
                HP_stealth = stealth;
                HP_nondir = (cip->tx_type <= TX_TYPE_S);
             }
          }
        }
      }

      fop = &Container[head];

      for (int i=0; i < total_objects && i < MAX_NMEA_OBJECTS; i++) {

         // note that MAX_NMEA_OBJECTS (6) < MAX_TRACKING_OBJECTS (8)

         // may want to skip the HP object if there are many to report
         // since it will be in the PFLAU sentence - but XCsoar etc
         // seem to ignore the PFLAU, so report the HP object both ways
         //if (total_objects < MAX_NMEA_OBJECTS || fop->addr != HP_addr) {

         uint8_t addr_type = fop->addr_type;
         if (addr_type > ADDR_TYPE_FLARM)
             addr_type = ADDR_TYPE_RANDOM;
         uint32_t id = fop->addr;
         alarm_level = fop->alarm_level;
         alt_diff = (int) (fop->alt_diff);  /* sent to NMEA */
         int dx = fop->dx;
         int dy = fop->dy;

         // if this aircraft not airborne, no-track targets should not be reported,
         //  unless the target is closer than 200m horizontally and 100m vertically.

         if (fop->no_track && ! ThisAircraft.airborne) {
            if (abs(alt_diff) > 100)
                continue;               // do not report this aircraft
           if (dx*dx + dy*dy > (200*200))
                continue;               // do not report this aircraft
         }

         // If either target or this aircraft is "stealth" then:
         //   course should be empty
         //   speed should be empty
         //   climbrate should be empty
         //   rel_alt should be fuzzyfied if closer than 2k/300m but no alarm

         // For non-directional targets:
         //   <RelativeEast>  (dx) needs to be empty (not zero)
         //   <RelativeNorth> (dy) needs to be the estimated distance
         //   course should be empty
         //   speed should be empty
         //   climbrate should be empty

         bool stealth = (fop->stealth || ThisAircraft.stealth);  /* reciprocal */

         char str_dx[6] = "";
         char str_climb_rate[8] = "";
         char str_course[4] = "";
         char str_speed[4] = "";
         if (fop->tx_type <= TX_TYPE_S) {     // a non-directional target
             dy = (int) fop->distance;
             if (dy > 2*ALARM_ZONE_LOW && !deeper)
                 continue;                   // only report if close (2000m)
         } else if (fop->distance > 99000 && !deeper) {
             continue;
         } else if (stealth) {
            if (abs(alt_diff) > 300)
                continue;               // do not report this aircraft
           if (dx*dx + dy*dy > (2000*2000))
                continue;               // do not report this aircraft
           id = 0xFFFFF0 + i;            // show as anonymous
           addr_type = ADDR_TYPE_RANDOM;    // (0) - not ADDR_TYPE_ANONYMOUS
           if (alarm_level <= ALARM_LEVEL_CLOSE)
               alt_diff = (alt_diff & 0xFFFFFF00) + 128;   /* fuzzify */
         } else {
             snprintf(str_dx, 6, "%d", dx);
             dtostrf(
               constrain(fop->vs / (_GPS_FEET_PER_METER * 60.0), -32.7, 32.7),
               5, 1, str_climb_rate);
             dtostrf(
               constrain(fop->course, 0.0, 360.0),
               3, 0, str_course);
             float speed  = fop->speed * _GPS_MPS_PER_KNOT;
             if (fop->airborne && speed == 0)
                 speed = 0.1;
             if (! fop->airborne && speed != 0)
                 speed = 0.0;
             dtostrf(
               constrain(speed, 0.0, 999.0),
               3, 0, str_speed);
         }

         if (alarm_level > ALARM_LEVEL_NONE)  --alarm_level;
           /* for NMEA export bypass CLOSE added between NONE and LOW */

         // data_source should be 1, 3, 4, 6 for ADS-B, ADS-R, TIS-B, Mode-S
         //   (GDL90 should probably be treated as ADS-B)
         // convert tx_type to the code used by FLARM:
         int data_source = data_source_code[fop->tx_type];

#if 1
         /*
          * When callsign is available - send it to a NMEA client.
          * If it is not - generate a callsign substitute,
          * based upon a protocol ID and the ICAO address
          */
         if (fop->callsign[0] == '\0') {

           snprintf_P(NMEABuffer, sizeof(NMEABuffer),
//            PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X!%s_%06X,%d,,%d,%s,%d,%d,%d,%d" PFLAA_EXT1_FMT "*"),
// aircraft type is supposed to be hex:
              PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X!%s_%06X,%s,,%s,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, dy, str_dx,
              alt_diff, addr_type, id, NMEA_CallSign_Prefix[fop->protocol], id,
              ltrim(str_course), ltrim(str_speed), ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0), data_source, fop->rssi  PFLAA_EXT1_ARGS );

         } else {   /* there is a callsign from incoming data */

           if (fop->callsign[sizeof(fop->callsign)-1] != '?')   // not set up by icao_to_n()
               fop->callsign[sizeof(fop->callsign)-1] = '\0';   // ensure termination

           snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X!%s_%s,%s,,%s,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, dy, str_dx,
              alt_diff, addr_type, id, NMEA_CallSign_Prefix[fop->protocol], fop->callsign,
              ltrim(str_course), ltrim(str_speed), ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0), data_source, fop->rssi  PFLAA_EXT1_ARGS );
         }
#else
         // skip the callsign
         snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PFLAA,%d,%d,%s,%d,%d,%06X,%s,,%s,%s,%X" PFLAA_EXT1_FMT "*"),
            alarm_level, dy, str_dx,
            alt_diff, addr_type, id,
            ltrim(str_course), ltrim(str_speed), ltrim(str_climb_rate), fop->aircraft_type,
            (fop->no_track? 1 : 0), data_source, fop->rssi  PFLAA_EXT1_ARGS );
#endif
         unsigned int nmealen = NMEA_add_checksum();
         NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, nmealen, false);

        //}  /* done skipping the HP object */

        if (fop->next >= MAX_TRACKING_OBJECTS)  break;    /* belt and suspenders */

        fop = &Container[fop->next];
      }
    }

    /* One PFLAU NMEA sentence is mandatory regardless of traffic reception status */
    float voltage = Battery_voltage();
    if (voltage < BATTERY_THRESHOLD_INVALID)
        voltage = 0;
    int power_status = (voltage > 0 && voltage < Battery_threshold()) ?
                         POWER_STATUS_BAD : POWER_STATUS_GOOD;

    if (total_objects > 0) {
        if (HP_addr == 0) {
           /* no aircraft has been identified as high priority, use */
           /*  the aircraft from the top of the sorted list, if any */
           fop = &Container[head];
           if (fop->addr) {
               HP_bearing = fop->bearing;
               HP_alt_diff = fop->alt_diff;
               HP_alarm_level = fop->alarm_level;
               HP_distance = fop->distance;
               HP_nondir = false;
               if (fop->stealth || ThisAircraft.stealth) {
                   HP_addr = 0xFFFFF0;
                   HP_stealth = true;
               } else {
                   HP_addr = fop->addr;
                   HP_stealth = false;
               }
           }
        }
    }

    int gps_status = (ThisAircraft.airborne ? GNSS_STATUS_3D_MOVING : GNSS_STATUS_3D_GROUND);
    int tx_status = (settings->txpower == RF_TX_POWER_OFF ? TX_STATUS_OFF : TX_STATUS_ON);
#if defined(ESP32)
    if (do_alarm_demo) {
        if ((millis() - SetupTimeMarker) < (1000*STROBE_INITIAL_RUN)) {
            gps_status = GNSS_STATUS_3D_MOVING;
            HP_alarm_level = ALARM_LEVEL_IMPORTANT;
        } else {
            OLED_no_msg();
            do_alarm_demo = false;  // turn demo off here, in case buzzer is set to OFF
        }
    } else
#endif
    {
        if (! has_Fix) {
            gps_status = GNSS_STATUS_NONE;
            tx_status = TX_STATUS_OFF;
        }
    }
    if (HP_addr) {
        if (HP_stealth && HP_alarm_level <= ALARM_LEVEL_CLOSE) {
            HP_alt_diff = (HP_alt_diff & 0xFFFFFF00) + 128;   /* fuzzify */
        }
        // rel_bearing needs to be empty (not zero) for non-directional targets
        char str_relbrg[6] = "";
        if (! HP_nondir) {             // not a non-directional target
            int rel_bearing = (int) (HP_bearing - ThisAircraft.course);
            rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));
            snprintf(str_relbrg, 6, "%d", rel_bearing);
        }
        if (HP_alarm_level > ALARM_LEVEL_NONE)  --HP_alarm_level;
        int alarm_type = (HP_alarm_level > 0)? ALARM_TYPE_AIRCRAFT : ALARM_TYPE_TRAFFIC;
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,%d,%d,%d,%d,%d,%s,%d,%d,%u,%06X" PFLAU_EXT1_FMT "*"),
                total_objects, tx_status, gps_status,
                power_status, HP_alarm_level, str_relbrg,
                alarm_type, HP_alt_diff, (int) HP_distance, HP_addr
                PFLAU_EXT1_ARGS );
    } else if (do_alarm_demo) {
        // simulate an aircraft
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,1,1,%d,1,%d,0,2,0,500,AAAAAA" PFLAU_EXT1_FMT "*"),
                GNSS_STATUS_3D_MOVING, (ALARM_LEVEL_IMPORTANT - 1)
                PFLAU_EXT1_ARGS );
    } else {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,0,%d,%d,%d,%d,,0,,,," PFLAU_EXT1_FMT "*"),
                tx_status, gps_status,
                power_status, ALARM_LEVEL_NONE
                PFLAU_EXT1_ARGS );
    }

    unsigned int nmealen = NMEA_add_checksum();
    NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, nmealen, false);

//#if defined(USE_SD_CARD)
#if 0
if (settings->debug_flags) {
    nmealen -= 2;   // overwrite existing \r\n
    snprintf_P(NMEABuffer+nmealen, sizeof(NMEABuffer)-nmealen, " at %02d:%02d:%02d\r\n",
         gnss.time.hour(), gnss.time.minute(), gnss.time.second());
    Serial.print(NMEABuffer);
    SD_log(NMEABuffer);
}
#endif

    static int beatcount = 0;
    if (++beatcount < 10)
        return;
    beatcount = 0;

#if !defined(EXCLUDE_SOFTRF_HEARTBEAT)
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFH,%06X,%d,%d,%d,%d,%d,%d*"),
            ThisAircraft.addr,settings->rf_protocol,
            rx_packets_counter,tx_packets_counter,millis(),(int)(voltage*100),ESP.getFreeHeap());
    nmealen = NMEA_add_checksum();
    NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, nmealen, false);
#endif /* EXCLUDE_SOFTRF_HEARTBEAT */
}

#if defined(USE_NMEALIB)

void NMEA_Position()    // only called in txrx_test() mode (and maybe from RPi)
{
  NmeaInfo info;
  size_t i;
  struct timeval tv;

  if (settings->nmea_g || settings->nmea2_g) {

    nmeaInfoClear(&info);

    info.sig = NMEALIB_SIG_SENSITIVE;
    info.fix = NMEALIB_FIX_3D;

    tv.tv_sec  = ThisAircraft.timestamp;
    tv.tv_usec = 0;

    nmeaTimeSet(&info.utc, &info.present, &tv);

    info.latitude = ((int) ThisAircraft.latitude) * 100.0;
    info.latitude += (ThisAircraft.latitude - (int) ThisAircraft.latitude) * 60.0;
    info.longitude = ((int) ThisAircraft.longitude) * 100.0;
    info.longitude += (ThisAircraft.longitude - (int) ThisAircraft.longitude) * 60.0;
    info.speed = ThisAircraft.speed * _GPS_KMPH_PER_KNOT;
    info.height = ThisAircraft.geoid_separation;
    info.elevation = ThisAircraft.altitude - info.height; /* above MSL */
    info.track = ThisAircraft.course;

#if 0
    info.mtrack = 55;
    info.magvar = 55;
#endif
    info.hdop = 2.3;
    info.vdop = 1.2;
    info.pdop = 2.594224354;

    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_FIX);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SPEED);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_TRACK);
#if 0
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MTRACK);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_MAGVAR);
#endif
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_VDOP);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_PDOP);

#if 0
    info.satellites.inUseCount = NMEALIB_MAX_SATELLITES;
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSECOUNT);
    for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
      info.satellites.inUse[i] = (unsigned int) (i + 1);
    }
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINUSE);

    info.satellites.inViewCount = NMEALIB_MAX_SATELLITES;
    for (i = 0; i < NMEALIB_MAX_SATELLITES; i++) {
      info.satellites.inView[i].prn = (unsigned int) i + 1;
      info.satellites.inView[i].elevation = (int) ((i * 10) % 90);
      info.satellites.inView[i].azimuth = (unsigned int) (i + 1);
      info.satellites.inView[i].snr = 99 - (unsigned int) i;
    }
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT);
    nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEW);
#endif

    size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
      (NMEALIB_SENTENCE_GPGGA | NMEALIB_SENTENCE_GPGSA | NMEALIB_SENTENCE_GPRMC));

    if (gen_sz) {
      NMEA_Outs(settings->nmea_g, settings->nmea2_g, nmealib_buf.buffer, gen_sz, false);
    }
  }
}

void NMEA_GGA()
{
  if (! settings->nmea_g && ! settings->nmea2_g)
    return;

  NmeaInfo info;

  float latitude = gnss.location.lat();
  float longitude = gnss.location.lng();

  nmeaInfoClear(&info);

  info.utc.hour = gnss.time.hour();
  info.utc.min = gnss.time.minute();
  info.utc.sec = gnss.time.second();
  info.utc.hsec = gnss.time.centisecond();

  info.latitude = ((int) latitude) * 100.0;
  info.latitude += (latitude - (int) latitude) * 60.0;
  info.longitude = ((int) longitude) * 100.0;
  info.longitude += (longitude - (int) longitude) * 60.0;

  info.sig = (NmeaSignal) gnss.location.Quality();
  info.satellites.inViewCount = gnss.satellites.value();

  info.hdop = gnss.hdop.hdop();

  info.elevation = gnss.altitude.meters(); /* above MSL */
  info.height = gnss.separation.meters();

  if (info.height == 0.0 && info.sig != (NmeaSignal) Invalid) {
    info.height = EGM96GeoidSeparation();
    info.elevation -= info.height;
  }

  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_UTCTIME);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LAT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_LON);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SIG);
  /* Should be SATINUSECOUNT, but it seems to be a bug in NMEALib */
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_SATINVIEWCOUNT);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HDOP);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_ELV);
  nmeaInfoSetPresent(&info.present, NMEALIB_PRESENT_HEIGHT);

  size_t gen_sz = nmeaSentenceFromInfo(&nmealib_buf, &info, (NmeaSentence)
                                        NMEALIB_SENTENCE_GPGGA );

  if (gen_sz) {
    //strncpy(GPGGA_Copy, nmealib_buf.buffer, gen_sz);  // for traffic alarm logging
    //GPGGA_Copy[gen_sz] = '\0';
    NMEA_Outs(settings->nmea_g, settings->nmea2_g, nmealib_buf.buffer, gen_sz, false);
  }
}
#endif /* USE_NMEALIB */

#if defined(USE_NMEA_CFG)

#include "../../driver/Buzzer.h"
#include "../../driver/LED.h"
#include "GDL90.h"
#include "D1090.h"

//#if !defined(SERIAL_FLUSH)
//#define SERIAL_FLUSH()       Serial.flush()
//#endif

// this is used specifically to respond to the source
// while NMEA_Out() specifically avoids that,
// so mask the source
void nmea_cfg_reply()
{
    unsigned int nmealen = NMEA_add_checksum();
    uint8_t dest = NMEA_Source;
    NMEA_Source = DEST_NONE;
    NMEA_Out(dest, NMEABuffer, nmealen, false);
    NMEA_Source = dest;
}

static void nmea_cfg_restart(bool save_settings)
{
  SoC->WDT_fini();
  if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
  if (save_settings)
      EEPROM_store();
  Serial.println();
  Serial.println(F("Restart is in progress. Please, wait..."));
  Serial.println();
  reboot();
}

bool isdecdigit(const char *p)
{
    if (*p >= '0' && *p <= '9')  return true;
    return false;
}

bool ishexdigit(const char *p)
{
    if (*p >= '0' && *p <= '9')  return true;
    if (*p >= 'A' && *p <= 'F')  return true;
    if (*p >= 'a' && *p <= 'f')  return true;
    return false;
}

bool cfg_is_updated;

int updated(TinyGPSCustom &field, const char *label, int cur_val)
{
    const char *p = field.value();
    if (p[0] == '\0')              // empty field
        return cur_val;
    int cfg_val = cur_val;
    if (isdecdigit(p)) {
        cfg_val = atoi(p);
        p = "not changed, still";
    } else {
        p = "given value invalid, left as";
    }
    if (cfg_val != cur_val) {
        p = "changed to";
        cfg_is_updated = true;
    }
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
        PSTR("%s %s %d\r\n"), label, p, cfg_val);
    nmea_cfg_reply();   // no '*' and thus checksum will not be added
    return cfg_val;
}

void NMEA_Process_SRF_SKV_Sentences()
{
  if (C_Version.isUpdated()) {

      if (strncmp(C_Version.value(), "RST", 3) == 0) {             // $PSRFC,RST*2D
          Serial.println(F("PSRFC Reboot..."));
          nmea_cfg_restart(false);
          // just reboot, settings not saved

      } else if (strncmp(C_Version.value(), "SAV", 3) == 0) {      // $PSRFC,SAV*3C
          Serial.println(F("PSRFC Save & Reboot..."));
          nmea_cfg_restart(true);
          // save settings to EEPROM and reboot

      } else if (strncmp(C_Version.value(), "OFF", 3) == 0) {      // $PSRFC,OFF*37
          Serial.println(F("PSRFC Shutdown..."));
          shutdown(SOFTRF_SHUTDOWN_NMEA);

#if defined(USE_OLED)
      } else if (strncmp(C_Version.value(), "PAG", 3) == 0) {      // $PSRFC,PAG*2E
          Serial.println(F("PSRFC Page Switch"));
          OLED_Next_Page();
#endif
      } else if (strncmp(C_Version.value(), "DEM", 3) == 0) {      // $PSRFC,DEM*34
          Serial.println(F("PSRFC Alarm Demo"));
          SetupTimeMarker = millis();
          do_alarm_demo = true;
#if defined(USE_OLED)
          OLED_msg("ALARM", " DEMO");
#endif

      } else if (strncmp(C_Version.value(), "TX0", 3) == 0) {      // $PSRFC,TX0*44
          Serial.println(F("PSRFC TX Off"));
          settings->txpower = RF_TX_POWER_OFF;

      } else if (strncmp(C_Version.value(), "TX1", 3) == 0) {      // $PSRFC,TX1*45
          Serial.println(F("PSRFC TX On"));
          settings->txpower = RF_TX_POWER_FULL;

      } else if (strncmp(C_Version.value(), "?", 1) == 0) {        // $PSRFC,?*47

          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
            PSRFX_VERSION,        settings->mode,     settings->rf_protocol,
            settings->band,       settings->aircraft_type, settings->alarm,
            settings->txpower,    settings->volume,   settings->pointer,
            settings->nmea_g,     settings->nmea_p,   settings->nmea_l,
            settings->nmea_s,     settings->nmea_out, settings->gdl90,
            settings->d1090,      settings->stealth,  settings->no_track,
            settings->power_save );

          nmea_cfg_reply();

      } else if (isdecdigit(C_Version.value())) {

          cfg_is_updated = false;

          settings->mode          = updated(C_Mode, "Mode", settings->mode);
          settings->rf_protocol   = updated(C_Protocol, "Protocol", settings->rf_protocol);
          settings->band          = updated(C_Band, "Region", settings->band);
          settings->aircraft_type = updated(C_AcftType, "AcftType", settings->aircraft_type);
          settings->alarm         = updated(C_Alarm, "Alarm", settings->alarm);
          settings->txpower       = updated(C_TxPower, "TxPower", settings->txpower);
          settings->volume        = updated(C_Volume, "Volume", settings->volume);
          settings->pointer       = updated(C_Pointer, "Pointer", settings->pointer);
          settings->nmea_g        = updated(C_NMEA_gnss, "NMEA_gnss", settings->nmea_g);
          settings->nmea_p        = updated(C_NMEA_private, "NMEA_private", settings->nmea_p);
          settings->nmea_l        = updated(C_NMEA_legacy, "NMEA_legacy", settings->nmea_l);
          settings->nmea_s        = updated(C_NMEA_sensors, "NMEA_sensors", settings->nmea_s);
          settings->nmea_out      = updated(C_NMEA_Output, "NMEA_Output", settings->nmea_out);
          settings->gdl90         = updated(C_GDL90_Output, "GDL90_Output", settings->gdl90);
          settings->d1090         = updated(C_D1090_Output, "D1090_Output", settings->d1090);
          settings->stealth       = updated(C_Stealth, "Stealth", settings->stealth);
          settings->no_track      = updated(C_noTrack, "noTrack", settings->no_track);
          settings->power_save    = updated(C_PowerSave, "PowerSave", settings->power_save);

          if (cfg_is_updated && atoi(C_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFC,0,,,,,,,,,,,,,,,,,,*48\r\n", 31);
              // fill in the structure inside TinyGPS with empty strings for all fields
      }
  }

  if (D_Version.isUpdated()) {

      if (strncmp(D_Version.value(), "?", 1) == 0) {

          //char psrfd_buf[MAX_PSRFD_LEN];
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFD,%d,%d,%06X,%06X,%06X,%d,%d,%d,%02X,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
            PSRFX_VERSION,            settings->id_method,  settings->aircraft_id,
            settings->ignore_id,      settings->follow_id,  settings->baud_rate,
            settings->power_external, settings->nmea_d,     settings->debug_flags,
            settings->nmea_out2,      settings->nmea2_g,    settings->nmea2_p,
            settings->nmea2_l,        settings->nmea2_s,    settings->nmea2_d,
            settings->relay,          settings->bluetooth,  settings->baudrate2,
            settings->invert2,        settings->nmea_e,     settings->nmea2_e,
            settings->altpin0,        settings->voice,      settings->strobe);

          nmea_cfg_reply();

      } else if (isdecdigit(D_Version.value())) {

          cfg_is_updated = false;

          settings->baud_rate      = updated(D_baud_rate, "Baud rate", settings->baud_rate);
          settings->power_external = updated(D_power_ext, "Power source", settings->power_external);
          settings->nmea_d         = updated(D_NMEA_debug, "NMEA_debug", settings->nmea_d);
          settings->debug_flags    = updated(D_debug_flags, "Debug flags", settings->debug_flags);
          settings->nmea_out2      = updated(D_NMEA2, "NMEA_Output2", settings->nmea_out2);
          if (settings->nmea_out2 == settings->nmea_out)
              settings->nmea_out2 = DEST_NONE;
          settings->nmea_g         = updated(D_NMEA2_gnss, "NMEA2_gnss", settings->nmea_g);
          settings->nmea_p         = updated(D_NMEA2_private, "NMEA2_private", settings->nmea_p);
          settings->nmea_l         = updated(D_NMEA2_legacy, "NMEA2_legacy", settings->nmea_l);
          settings->nmea_s         = updated(D_NMEA2_sensors, "NMEA2_sensors", settings->nmea_s);
          settings->nmea_d         = updated(D_NMEA2_debug, "NMEA2_debug", settings->nmea_d);
          settings->relay          = updated(D_relay, "Relay", settings->relay);
          settings->bluetooth      = updated(D_bluetooth, "Bluetooth", settings->bluetooth);
#if defined(ESP32)
          settings->baudrate2      = updated(D_baudrate2, "Baud rate 2", settings->baudrate2);
          settings->invert2        = updated(D_invert2, "Serial2 logic", settings->invert2);
#endif
          settings->nmea_e         = updated(D_extern1, "NMEA1_ext", settings->nmea_e);
          settings->nmea2_e        = updated(D_extern2, "NMEA2_ext", settings->nmea2_e);
#if defined(ESP32)
          settings->altpin0        = updated(D_altpin0, "Use alt RX pin", settings->altpin0);
#endif
          settings->voice          = updated(D_voice, "Voice", settings->voice);
          settings->strobe         = updated(D_strobe, "Strobe", settings->strobe);

          settings->id_method      = updated(D_id_method, "ID method", settings->id_method);
          if (ishexdigit(D_aircraft_id.value())) {
              uint32_t val = strtoul(D_aircraft_id.value(), NULL, 16);
              if (val != settings->aircraft_id) {
                 settings->aircraft_id = val;
                 cfg_is_updated = true;
              }
              Serial.print(F("Aircraft ID = ")); Serial.println(settings->aircraft_id, HEX);
          }
          if (ishexdigit(D_ignore_id.value())) {
              uint32_t val = strtoul(D_ignore_id.value(), NULL, 16);
              if (val != settings->ignore_id) {
                settings->ignore_id = val;
                cfg_is_updated = true;
              }
              Serial.print(F("Ignore ID = ")); Serial.println(settings->ignore_id, HEX);
          }
          if (ishexdigit(D_follow_id.value())) {
              uint32_t val = strtoul(D_follow_id.value(), NULL, 16);
              if (val != settings->follow_id) {
                settings->follow_id = val;
                cfg_is_updated = true;
              }
              Serial.print(F("Follow ID = ")); Serial.println(settings->follow_id, HEX);
          }

          if (cfg_is_updated && atoi(D_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFD,0,,,,,,,,,,,,,,,,,,,,,,,*47\r\n", 36);
      }
  }

  if (F_Version.isUpdated()) {

      if (strncmp(F_Version.value(), "?", 1) == 0) {           // $PSRFF,?*42
        //char psrff_buf[MAX_PSRFF_LEN];
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
          PSTR("$PSRFF,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
          PSRFX_VERSION,         settings->rx1090,    settings->mode_s,
          settings->gdl90_in,    settings->gnss_pins, settings->ppswire,
          settings->logalarms,   settings->sd_card,   settings->logflight,
          settings->loginterval, settings->alt_udp,   settings->tcpport,
          settings->tcpmode,     geoid_from_setting,  settings->freq_corr);
        nmea_cfg_reply();
      }

      // at this point PSRFF is output-only as above
      // if changed to allow settings, note that:
      //     geoid_from_setting = descale(settings->geoid, 5, 1, 1) - 10;
      //     settings->geoid = enscale(geoid_from_setting+10, 5, 1, 1);
  }

  // $PSRFT,0*5F  or  $PSRFT,1*5E  to set test_mode to 0 or 1, or $PSRFT,?*50 to query
  // Also reports the chip ID, which is not settable, transmitted if ID type is "device"
  if (T_testmode.isUpdated()) {

      char tval = T_testmode.value()[0];
      if (tval == '?') {
          snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PSRFT,%d,%06X*"),
                test_mode, (SoC->getChipId() & 0x00FFFFFF));
          nmea_cfg_reply();
          // put custom code here for debugging, for example:
#if defined(ESP32)
          if (settings->rx1090)
              show_zone_stats();
#endif
      } else {
          if (tval == '0')
              test_mode = false;
          else if (tval == '1')
              test_mode = true;
          else
              test_mode = !test_mode;
          if (test_mode) {
#if defined(ESP32)
              OLED_msg("TEST",   " MODE");
#endif
              Serial.println("Test Mode on");
          } else {
#if defined(ESP32)
              OLED_msg("NOTEST", " MODE");
#endif
              Serial.println("Test Mode off");
          }
          do_test_mode();
      }
  }

#if defined(USE_OGN_ENCRYPTION)
  if (S_Version.isUpdated()) {

      if (strncmp(S_Version.value(), "?", 1) == 0) {

        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFS,%d,%08X%08X%08X%08X*"),
            PSRFX_VERSION,
            settings->igc_key[0]? 0x88888888 : 0,
            settings->igc_key[1]? 0x88888888 : 0,
            settings->igc_key[2]? 0x88888888 : 0,
            settings->igc_key[3]? 0x88888888 : 0);
            /* mask the key from prying eyes */

        nmea_cfg_reply();

      } else if (isdecdigit(S_Version.value())) {

          char buf[32 + 1];

          strncpy(buf, S_IGC_Key.value(), sizeof(buf));

          settings->igc_key[3] = strtoul(buf + 24, NULL, 16);
          buf[24] = 0;
          settings->igc_key[2] = strtoul(buf + 16, NULL, 16);
          buf[16] = 0;
          settings->igc_key[1] = strtoul(buf +  8, NULL, 16);
          buf[ 8] = 0;
          settings->igc_key[0] = strtoul(buf +  0, NULL, 16);

          snprintf_P(buf, sizeof(buf),
            PSTR("%08X%08X%08X%08X"),
            settings->igc_key[0], settings->igc_key[1],
            settings->igc_key[2], settings->igc_key[3]);

          Serial.print(F("IGC Key = ")); Serial.println(buf);

          if (atoi(S_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSRFS,0,*74\r\n", 14);
      }
  }
#endif /* USE_OGN_ENCRYPTION */

#if defined(USE_SKYVIEW_CFG)
  if (V_Version.isUpdated()) {

      if (strncmp(V_Version.value(), "?", 1) == 0) {

          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSKVC,%d,%d,%d,%d,%d,%d,%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%X*"),
            PSKVC_VERSION,  ui->adapter,      ui->connection,
            ui->units,      ui->zoom,         ui->protocol,
            ui->baudrate,   ui->server,       ui->key,
            ui->rotate,     ui->orientation,  ui->adb,
            ui->idpref,     ui->vmode,        ui->voice,
            ui->aghost,     ui->filter,       ui->power_save,
            ui->team);

          nmea_cfg_reply();

      } else if (isdecdigit(V_Version.value())) {

          cfg_is_updated = false;

          ui->adapter     = updated(V_Adapter, "Adapter", ui->adapter);
          ui->connection  = updated(V_Connection, "Connection", ui->connection);
          ui->units       = updated(V_Units, "Units", ui->units);
          ui->zoom        = updated(V_Zoom, "Zoom", ui->zoom);
          ui->protocol    = updated(V_Protocol, "Protocol", ui->protocol);
          ui->baudrate    = updated(V_Baudrate, "Baudrate", ui->baudrate);
          if (V_Server.value()[0]) {
              strncpy(ui->server, V_Server.value(), sizeof(ui->server));
              Serial.print(F("Server = ")); Serial.println(ui->server);
              cfg_is_updated = true;
          }
          if (V_Key.value()[0]) {
              strncpy(ui->key, V_Key.value(), sizeof(ui->key));
              Serial.print(F("Key = ")); Serial.println(ui->key);
              cfg_is_updated = true;
          }
          ui->rotate      = updated(V_Rotate, "Rotation", ui->rotate);
          ui->orientation = updated(V_Orientation, "Orientation", ui->orientation);
          ui->adb         = updated(V_AvDB, "AvDB", ui->adb);
          ui->idpref      = updated(V_ID_Pref, "ID_Pref", ui->idpref);
          ui->vmode       = updated(V_VMode, "VMode", ui->vmode);
          ui->voice       = updated(V_Voice, "Voice", ui->voice);
          ui->aghost      = updated(V_AntiGhost, "AntiGhost", ui->aghost);
          ui->filter      = updated(V_Filter, "Filter", ui->filter);
          ui->power_save  = updated(V_PowerSave, "PowerSave", ui->power_save);

          if (ishexdigit(V_Team.value())) {
              uint32_t val = strtoul(V_Team.value(), NULL, 16);
              if (val != ui->team) {
                ui->team = val;
                cfg_is_updated = true;
              }
              Serial.print(F("Team = ")); Serial.println(ui->team, HEX);
          }

          if (cfg_is_updated && atoi(V_Version.value()))
              nmea_cfg_restart(true);
          else
              NMEA_encode("$PSKVC,0,,,,,,,,,,,,,,,,,,*41\r\n", 31);
      }
  }
#endif /* USE_SKYVIEW_CFG */
}
#endif /* USE_NMEA_CFG */

static char hexbuf[NMEA_BUFFER_SIZE];
char *bytes2Hex(byte *buffer, size_t size)
{
  char *p = hexbuf;
  for (int i=0; i < size && i < NMEA_BUFFER_SIZE/2-1; i++) {
    byte c = buffer[i];
    byte cl = c & 0x0F;
    byte cu = (c >> 4) & 0x0F;
    *p++ = (cu < 0x0A)? '0'+cu : 'A'+cu-0x0A;
    *p++ = (cl < 0x0A)? '0'+cl : 'A'+cl-0x0A;
  }
  *p = '\0';
  return hexbuf;
}
