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
#include "../../driver/WiFi.h"
#include "../../driver/EEPROM.h"
#include "../../driver/RF.h"
#include "../../driver/Battery.h"
#include "../../driver/Baro.h"
#include "../../driver/Bluetooth.h"
#include "../../TrafficHelper.h"

#define ADDR_TO_HEX_STR(s, c) (s += ((c) < 0x10 ? "0" : "") + String((c), HEX))

uint8_t NMEA_Source = DEST_NONE;   // identifies which port a sentence came from

char NMEABuffer[NMEA_BUFFER_SIZE]; //buffer for NMEA data
char GPGGA_Copy[NMEA_BUFFER_SIZE];   //store last $GGA sentence

static char NMEA_Callsign[NMEA_CALLSIGN_SIZE];

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
//if ((settings->nmea_d || settings->nmea2_d)  && (settings->debug_flags & DEBUG_FAKEFIX)) {
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
//db = ((settings->nmea_d || settings->nmea2_d) && (settings->debug_flags & DEBUG_FAKEFIX));
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
  [RF_PROTOCOL_LATEST]    = "FLR",
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

void NMEA_add_checksum(char *buf, size_t limit)
{
  size_t sentence_size = strlen(buf);

  //calculate the checksum
  unsigned char cs = 0;
  for (unsigned int n = 1; n < sentence_size - 1; n++) {
    cs ^= buf[n];
  }

  char *csum_ptr = buf + sentence_size;
  snprintf_P(csum_ptr, limit, PSTR("%02X\r\n"), cs);
}

// send self-test and version sentences out, imitating a FLARM
void sendPFLAV()
{
  static uint32_t whensend = 28000;
  if (settings->nmea_l || settings->nmea2_l) {
    uint32_t millisnow = millis();
    if (millisnow > whensend) {
      snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAE,A,0,0*"));
      NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - 16);
      NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, strlen(NMEABuffer), false);
      snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PFLAV,A,2.4,7.20,%s-%s*"),
                   SOFTRF_IDENT, SOFTRF_FIRMWARE_VERSION);  // our version in obstacle db text field
      NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - 48);
      NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, strlen(NMEABuffer), false);
      whensend = millisnow + 73000;
    }
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
    }
    if (Serial2Baud != 0) {
        Serial2.setRxBufferSize(Serial2BufSize);
        Serial2.begin(Serial2Baud, SERIAL_8N1, Serial2RxPin, Serial2TxPin, settings->invert2);
        has_serial2 = true;
        Serial.printf("Serial2 started at baud rate %d, logic:%d\r\n",
               Serial2Baud, settings->invert2);
    } else {
        // note BAUD_DEFAULT here means Serial2 disabled, not 38400
        Serial.println(F("Serial2 NOT started"));
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

  PGRMZ_TimeMarker = millis();

#if defined(ENABLE_AHRS)
  RPYL_TimeMarker = millis();
#endif /* ENABLE_AHRS */

  sendPFLAV();
}

void NMEA_Out(uint8_t dest, const char *buf, size_t size, bool nl)
{
#if 0
  Serial.print("NMEA_Out(");
  Serial.print(dest);
  Serial.print("): ", dest);
  Serial.write(buf, size);
  if (nl) Serial.write('\n');
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
          SoC->UART_ops->write((const byte *) "\n", 1);
      } else {
        Serial.write(buf, size);
        if (nl)
          Serial.write('\n');
      }
    }
    break;
  case DEST_UART2:
    {
      if (has_serial2) {
        Serial2.write(buf, size);
        if (nl)
          Serial2.write('\n');
      }
    }
    break;
  case DEST_UDP:
    {
      size_t udp_size = size;

      if (size >= sizeof(UDPpacketBuffer))
        udp_size = sizeof(UDPpacketBuffer) - 1;
      memcpy(UDPpacketBuffer, buf, udp_size);

      if (nl)
        UDPpacketBuffer[udp_size] = '\n';

      SoC->WiFi_transmit_UDP(UDP_NMEA_Output_Port, (byte *) UDPpacketBuffer,
                              nl ? udp_size + 1 : udp_size);
    }
    break;
  case DEST_TCP:
    {
#if defined(NMEA_TCP_SERVICE)
      if (TCP_active) {
        WiFi_transmit_TCP(buf, size);
        if (nl)
          WiFi_transmit_TCP("\n", 1);
      }
#endif
    }
    break;
  case DEST_USB:
    {
      if (SoC->USB_ops) {
        SoC->USB_ops->write((const byte *) buf, size);
        if (nl)
          SoC->USB_ops->write((const byte *) "\n", 1);
      }
    }
    break;
  case DEST_BLUETOOTH:
    {
      if (BTactive && SoC->Bluetooth_ops) {
        SoC->Bluetooth_ops->write((const byte *) buf, size);
        if (nl)
          SoC->Bluetooth_ops->write((const byte *) "\n", 1);
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
    // First check whether it is GNSS or FLARM sentences, skip them (echo)
    if (buf[1]=='G' && buf[2]=='P')
        return;
    if (buf[1]=='P' && buf[2]=='F' && buf[3]=='L' && buf[4]=='A')
        return;

#if defined(USE_NMEA_CFG)
    // Also trap PSRF config sentences, process internally instead
    // Not sure whether to process SKV sentences here or pass them on?
    if (buf[1]=='P' && buf[2]=='S' && buf[3]=='R' && buf[4]=='F') {
        NMEA_bridge_sent = true;   // not really sent, but substantial processing
        for (int i=0; i<len; i++) {
            if (gnss.encode(buf[i])) {   // valid sentence
                NMEA_Process_SRF_SKV_Sentences();
                return;
            }
        }
        // if $PSRF but not a valid sentence, send it back to the source:
        NMEA_Out(NMEA_Source, buf, len, false);
        NMEA_Out(NMEA_Source, "-- invalid", 10, true);
        return;
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

  sendPFLAV();

  if ((settings->nmea_s || settings->nmea2_s)
      && ThisAircraft.pressure_altitude != 0.0 && isTimeToPGRMZ()) {

    int altitude = constrain(
            (int) (ThisAircraft.pressure_altitude * _GPS_FEET_PER_METER),
            -1000, 60000);

    /* https://developer.garmin.com/downloads/legacy/uploads/2015/08/190-00684-00.pdf */
    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$PGRMZ,%d,f,%c*"),
               altitude, isValidGNSSFix() ? '3' : '1'); /* feet , 3D fix */

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));
    NMEA_Outs(settings->nmea_s, settings->nmea2_s, NMEABuffer, strlen(NMEABuffer), false);

#if !defined(EXCLUDE_LK8EX1)
    char str_Vcc[6];
    dtostrf(Battery_voltage(), 3, 1, str_Vcc);

    snprintf_P(NMEABuffer, sizeof(NMEABuffer), PSTR("$LK8EX1,999999,%d,%d,%d,%s*"),
            constrain((int) ThisAircraft.pressure_altitude, -1000, 99998), /* meters */
            (int) ((ThisAircraft.vs * 100) / (_GPS_FEET_PER_METER * 60)),  /* cm/s   */
            constrain((int) Baro_temperature(), -99, 98),                  /* deg. C */
            str_Vcc);

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));
    NMEA_Outs(settings->nmea_s, settings->nmea2_s, NMEABuffer, strlen(NMEABuffer), false);

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
          NmeaTCP[i].connect_ts = now();
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
         (now() - NmeaTCP[i].connect_ts) >= NMEATCP_ACK_TIMEOUT) {

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

    int alt_diff;
    int abs_alt_diff;
    float distance;
    float adj_dist;
    float bearing;

    int alarm_level = ALARM_LEVEL_NONE;
    int data_source = DATA_SOURCE_FLARM;
    time_t this_moment = now();
    uint32_t follow_id = settings->follow_id;

    /* High priority object (most relevant target) */
    int HP_index = MAX_TRACKING_OBJECTS;
    int HP_alt_diff = 0;
    int HP_alarm_level = ALARM_LEVEL_NONE;
    float HP_adj_dist  = 999999999;
    float HP_distance  = 999999999;
    float HP_bearing = 0;
    uint32_t HP_addr = 0;
    bool HP_stealth = false;
    int total_objects = 0;
    int head = 0;

    bool has_Fix = (isValidFix() || (settings->mode == SOFTRF_MODE_TXRX_TEST));

    if (has_Fix) {

      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        ufo_t *cip = &Container[i];

        if (cip->addr && ((this_moment - cip->timestamp) <= EXPORT_EXPIRATION_TIME)) {
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
               || cip->addr == follow_id)
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
                 if (Container[next].alarm_level <= alarm_level
                  && Container[next].addr != follow_id
                  && Container[next].adj_distance >= adj_dist)
                        break;   /* insert before this one */
                 /* else */
                   previous = next;  /* the preceding list entry */
                   next = Container[next].next;
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
             }
          }
        }
      }

      ufo_t *fop = &Container[head];

      for (int i=0; i < total_objects && i < MAX_NMEA_OBJECTS; i++) {

         // note that MAX_NMEA_OBJECTS (6) < MAX_TRACKING_OBJECTS (8)

         uint8_t addr_type = fop->addr_type > ADDR_TYPE_ANONYMOUS ?
                                  ADDR_TYPE_ANONYMOUS : fop->addr_type;

         bool stealth = (fop->stealth || ThisAircraft.stealth);  /* reciprocal */

         alarm_level = fop->alarm_level;

         uint32_t id = fop->addr;
         if (stealth) {
           id = 0xFFFFF0 + i;   /* show as anonymous */
           addr_type = ADDR_TYPE_ANONYMOUS;
         }

         // may want to skip the HP object if there are many to report
         // since it will be in the PFLAU sentence - but XCsoar etc
         // seem to ignore the PFLAU, so report the HP object both ways
         //if (total_objects < MAX_NMEA_OBJECTS || fop->addr != HP_addr) {

         alt_diff = (int) (fop->alt_diff);  /* sent to NMEA */

         int course = (int) fop->course;
         int speed  = (int) (fop->speed * _GPS_MPS_PER_KNOT);

         /* mask some data following FLARM protocol: */
         char str_climb_rate[8] = "";
         if (stealth && alarm_level <= ALARM_LEVEL_CLOSE) {
             alt_diff = (alt_diff & 0xFFFFFF00) + 128;   /* fuzzify */
             course = 0;
             speed  = 0;
         } else {
             dtostrf(
               constrain(fop->vs / (_GPS_FEET_PER_METER * 60.0), -32.7, 32.7),
               5, 1, str_climb_rate);
         }

         if (alarm_level > ALARM_LEVEL_NONE)  --alarm_level;
           /* for NMEA export bypass CLOSE added between NONE and LOW */

         data_source = fop->protocol == RF_PROTOCOL_ADSB_UAT ?
                            DATA_SOURCE_ADSB : DATA_SOURCE_FLARM;

#if 1
         /*
          * When callsign is available - send it to a NMEA client.
          * If it is not - generate a callsign substitute,
          * based upon a protocol ID and the ICAO address
          */
         if (fop->callsign[0] == '\0') {

           snprintf_P(NMEABuffer, sizeof(NMEABuffer),
//            PSTR("$PFLAA,%d,%d,%d,%d,%d,%06X!%s_%06X,%d,,%d,%s,%d,%d,%d,%d" PFLAA_EXT1_FMT "*"),
// aircraft type is supposed to be hex:
              PSTR("$PFLAA,%d,%d,%d,%d,%d,%06X!%s_%06X,%d,,%d,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, (int) fop->dy, (int) fop->dx,
              alt_diff, addr_type, id, NMEA_CallSign_Prefix[fop->protocol], id,
              course, speed, ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0),
              (fop->protocol==RF_PROTOCOL_ADSB_1090? 1 : (fop->protocol==RF_PROTOCOL_GDL90? 1 : 0)),
              fop->rssi
              PFLAA_EXT1_ARGS );

         } else {   /* there is a callsign from incoming data */

           /* memset((void *) NMEA_Callsign, 0, sizeof(NMEA_Callsign));
              memcpy(NMEA_Callsign, fop->callsign, sizeof(fop->callsign)); */
           fop->callsign[sizeof(fop->callsign)-1] = '\0';

           snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PFLAA,%d,%d,%d,%d,%d,%06X!%s,%d,,%d,%s,%X,%d,%d,%d" PFLAA_EXT1_FMT "*"),
              alarm_level, (int) fop->dy, (int) fop->dx,
              alt_diff, addr_type, id, fop->callsign,
              course, speed, ltrim(str_climb_rate), fop->aircraft_type,
              (fop->no_track? 1 : 0),
              (fop->protocol==RF_PROTOCOL_ADSB_1090? 1 : (fop->protocol==RF_PROTOCOL_GDL90? 1 : 0)),
              fop->rssi
              PFLAA_EXT1_ARGS );
         }
#else
         // skip the callsign
         snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PFLAA,%d,%d,%d,%d,%d,%06X,%d,,%d,%s,%X" PFLAA_EXT1_FMT "*"),
            alarm_level, (int) fop->dy, (int) fop->dx,
            alt_diff, addr_type, id,
            course, speed, ltrim(str_climb_rate), fop->aircraft_type
            PFLAA_EXT1_ARGS );
#endif
         NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));
         NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, strlen(NMEABuffer), false);

        //}  /* done skipping the HP object */

        if (fop->next >= MAX_TRACKING_OBJECTS)  break;    /* belt and suspenders */

        fop = &Container[fop->next];
      }
    }

    /* One PFLAU NMEA sentence is mandatory regardless of traffic reception status */
    float voltage    = Battery_voltage();
    if (voltage < BATTERY_THRESHOLD_INVALID)
        voltage = 0;
    int power_status = (voltage > 0 && voltage < Battery_threshold()) ?
                         POWER_STATUS_BAD : POWER_STATUS_GOOD;

    if (total_objects > 0) {
        if (HP_addr == 0) {
           /* no aircraft has been identified as high priority, use */
           /*  the aircraft from the top of the sorted list, if any */
           ufo_t *cip = &Container[head];
           if (cip->addr) {
               HP_bearing = cip->bearing;
               HP_alt_diff = cip->alt_diff;
               HP_alarm_level = cip->alarm_level;
               HP_distance = cip->distance;
               if (cip->stealth || ThisAircraft.stealth) {
                   HP_addr = 0xFFFFF0;
                   HP_stealth = true;
               } else {
                   HP_addr = cip->addr;
                   HP_stealth = false;
               }
           }
        }
    }

    int gps_status = (ThisAircraft.airborne ? GNSS_STATUS_3D_MOVING : GNSS_STATUS_3D_GROUND);
    int tx_status = (settings->txpower == RF_TX_POWER_OFF ? TX_STATUS_OFF : TX_STATUS_ON);
    if (! has_Fix) {
        gps_status = GNSS_STATUS_NONE;
        tx_status = TX_STATUS_OFF;
    }
    if (HP_addr) {
        if (HP_stealth && HP_alarm_level <= ALARM_LEVEL_CLOSE) {
            HP_alt_diff = (HP_alt_diff & 0xFFFFFF00) + 128;   /* fuzzify */
        }
        int rel_bearing = (int) (HP_bearing - ThisAircraft.course);
        rel_bearing += (rel_bearing < -180 ? 360 : (rel_bearing > 180 ? -360 : 0));
        if (HP_alarm_level > ALARM_LEVEL_NONE)  --HP_alarm_level;
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,%d,%d,%d,%d,%d,%d,%d,%d,%u,%06X" PFLAU_EXT1_FMT "*"),
                total_objects, tx_status, gps_status,
                power_status, HP_alarm_level, rel_bearing,
                ALARM_TYPE_AIRCRAFT, HP_alt_diff, (int) HP_distance, HP_addr
                PFLAU_EXT1_ARGS );
    } else {
        snprintf_P(NMEABuffer, sizeof(NMEABuffer),
                PSTR("$PFLAU,0,%d,%d,%d,%d,,0,,," PFLAU_EXT1_FMT "*"),
                tx_status, gps_status,
                power_status, ALARM_LEVEL_NONE
                PFLAU_EXT1_ARGS );
    }

    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));
    NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, strlen(NMEABuffer), false);

    static int beatcount = 0;
    if (++beatcount < 10)
        return;
    beatcount = 0;

#if !defined(EXCLUDE_SOFTRF_HEARTBEAT)
    snprintf_P(NMEABuffer, sizeof(NMEABuffer),
            PSTR("$PSRFH,%06X,%d,%d,%d,%d,%d*"),
            ThisAircraft.addr,settings->rf_protocol,
            rx_packets_counter,tx_packets_counter,(int)(voltage*100),ESP.getFreeHeap());
    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));
    NMEA_Outs(settings->nmea_l, settings->nmea2_l, NMEABuffer, strlen(NMEABuffer), false);
#endif /* EXCLUDE_SOFTRF_HEARTBEAT */

    if (settings->debug_flags & DEBUG_RESVD1) {
        Serial.printf("ThisAircraft.baro_alt_diff = %.0f\r\n", ThisAircraft.baro_alt_diff);
        Serial.printf("OthAcfts Avg baro_alt_diff = %.0f\r\n", average_baro_alt_diff);
    }
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
    info.elevation = ThisAircraft.altitude; /* above MSL */
    info.height = LookupSeparation(ThisAircraft.latitude, ThisAircraft.longitude);
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
    info.height = LookupSeparation(latitude, longitude);
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
    strncpy(GPGGA_Copy, nmealib_buf.buffer, gen_sz);  // for traffic alarm logging
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

void nmea_cfg_send()
{
    NMEA_add_checksum(NMEABuffer, sizeof(NMEABuffer) - strlen(NMEABuffer));
#if defined(USE_NMEA_CFG)
    uint8_t dest = NMEA_Source;           // answer the config source
#else
    uint8_t dest = settings->nmea_out;    // is nmea_cfg_send() called?
#endif /* USE_NMEA_CFG */
    NMEA_Out(dest, NMEABuffer, strlen(NMEABuffer), false);
}

static void nmea_cfg_restart()
{
  Serial.println();
  Serial.println(F("Restart is in progress. Please, wait..."));
  Serial.println();
  reboot();
}

void NMEA_Process_SRF_SKV_Sentences()
{
      if (C_Version.isUpdated()) {
        if (strncmp(C_Version.value(), "RST", 3) == 0) {
            SoC->WDT_fini();
            nmea_cfg_restart();
        } else if (strncmp(C_Version.value(), "OFF", 3) == 0) {
          shutdown(SOFTRF_SHUTDOWN_NMEA);
        } else if (strncmp(C_Version.value(), "?", 1) == 0) {

          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSRFC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*"),
              PSRFC_VERSION,        settings->mode,     settings->rf_protocol,
              settings->band,       settings->aircraft_type, settings->alarm,
              settings->txpower,    settings->volume,   settings->pointer,
              settings->nmea_g,     settings->nmea_p,   settings->nmea_l,
              settings->nmea_s,     settings->nmea_out, settings->gdl90,
              settings->d1090,      settings->stealth,  settings->no_track,
              settings->power_save );

          nmea_cfg_send();

        } else if (atoi(C_Version.value()) == PSRFC_VERSION) {
          bool cfg_is_updated = false;

          if (C_Mode.isUpdated())
          {
            settings->mode = atoi(C_Mode.value());
            Serial.print(F("Mode = ")); Serial.println(settings->mode);
            cfg_is_updated = true;
          }
          if (C_Protocol.isUpdated())
          {
            settings->rf_protocol = atoi(C_Protocol.value());
            Serial.print(F("Protocol = ")); Serial.println(settings->rf_protocol);
            cfg_is_updated = true;
          }
          if (C_Band.isUpdated())
          {
            settings->band = atoi(C_Band.value());
            Serial.print(F("Region = ")); Serial.println(settings->band);
            cfg_is_updated = true;
          }
          if (C_AcftType.isUpdated())
          {
            settings->aircraft_type = atoi(C_AcftType.value());
            Serial.print(F("AcftType = ")); Serial.println(settings->aircraft_type);
            cfg_is_updated = true;
          }
          if (C_Alarm.isUpdated())
          {
            settings->alarm = atoi(C_Alarm.value());
            Serial.print(F("Alarm = ")); Serial.println(settings->alarm);
            cfg_is_updated = true;
          }
          if (C_TxPower.isUpdated())
          {
            settings->txpower = atoi(C_TxPower.value());
            Serial.print(F("TxPower = ")); Serial.println(settings->txpower);
            cfg_is_updated = true;
          }
          if (C_Volume.isUpdated())
          {
            settings->volume = atoi(C_Volume.value());
            Serial.print(F("Volume = ")); Serial.println(settings->volume);
            cfg_is_updated = true;
          }
           if (C_Pointer.isUpdated())
          {
            settings->pointer = atoi(C_Pointer.value());
            Serial.print(F("Pointer = ")); Serial.println(settings->pointer);
            cfg_is_updated = true;
          }
          if (C_NMEA_gnss.isUpdated())
          {
            settings->nmea_g = atoi(C_NMEA_gnss.value());
            Serial.print(F("NMEA_gnss = ")); Serial.println(settings->nmea_g);
            cfg_is_updated = true;
          }
          if (C_NMEA_private.isUpdated())
          {
            settings->nmea_p = atoi(C_NMEA_private.value());
            Serial.print(F("NMEA_private = ")); Serial.println(settings->nmea_p);
            cfg_is_updated = true;
          }
          if (C_NMEA_legacy.isUpdated())
          {
            settings->nmea_l = atoi(C_NMEA_legacy.value());
            Serial.print(F("NMEA_legacy = ")); Serial.println(settings->nmea_l);
            cfg_is_updated = true;
          }
           if (C_NMEA_sensors.isUpdated())
          {
            settings->nmea_s = atoi(C_NMEA_sensors.value());
            Serial.print(F("NMEA_sensors = ")); Serial.println(settings->nmea_s);
            cfg_is_updated = true;
          }
          if (C_NMEA_Output.isUpdated())
          {
            settings->nmea_out = atoi(C_NMEA_Output.value());
            Serial.print(F("NMEA_Output = ")); Serial.println(settings->nmea_out);
            cfg_is_updated = true;
          }
          if (C_GDL90_Output.isUpdated())
          {
            settings->gdl90 = atoi(C_GDL90_Output.value());
            Serial.print(F("GDL90_Output = ")); Serial.println(settings->gdl90);
            cfg_is_updated = true;
          }
          if (C_D1090_Output.isUpdated())
          {
            settings->d1090 = atoi(C_D1090_Output.value());
            Serial.print(F("D1090_Output = ")); Serial.println(settings->d1090);
            cfg_is_updated = true;
          }
          if (C_Stealth.isUpdated())
          {
            settings->stealth = atoi(C_Stealth.value());
            Serial.print(F("Stealth = ")); Serial.println(settings->stealth);
            cfg_is_updated = true;
          }
          if (C_noTrack.isUpdated())
          {
            settings->no_track = atoi(C_noTrack.value());
            Serial.print(F("noTrack = ")); Serial.println(settings->no_track);
            cfg_is_updated = true;
          }
          if (C_PowerSave.isUpdated())
          {
            settings->power_save = atoi(C_PowerSave.value());
            Serial.print(F("PowerSave = ")); Serial.println(settings->power_save);
            cfg_is_updated = true;
          }

          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
        }
      }

      if (D_Version.isUpdated()) {
        if (strncmp(D_Version.value(), "?", 1) == 0) {
          char psrfd_buf[MAX_PSRFD_LEN];
          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSRFD,%d,%d,%06X,%06X,%06X,%d,%d,%d,%02X,%d,%d,%d,%d,%d,%d*"),
              PSRFD_VERSION,            settings->id_method,  settings->aircraft_id,
              settings->ignore_id,      settings->follow_id,  settings->baud_rate,
              settings->power_external, settings->nmea_d,     settings->debug_flags,
              settings->nmea_out2,      settings->nmea2_g,    settings->nmea2_p,
              settings->nmea2_l,        settings->nmea2_s,    settings->nmea2_d);

          nmea_cfg_send();

        } else if (atoi(D_Version.value()) == PSRFD_VERSION) {
          bool cfg_is_updated = false;

          if (D_id_method.isUpdated()) {
            settings->id_method = atoi(D_id_method.value());
            Serial.print(F("ID method = ")); Serial.println(settings->id_method);
            cfg_is_updated = true;
          }
          if (D_aircraft_id.isUpdated()) {
            settings->aircraft_id = strtoul(D_aircraft_id.value(), NULL, 16);
            Serial.print(F("Aircraft ID = ")); Serial.println(settings->aircraft_id, HEX);
            cfg_is_updated = true;
          }
          if (D_ignore_id.isUpdated()) {
            settings->ignore_id = strtoul(D_ignore_id.value(), NULL, 16);
            Serial.print(F("Ignore ID = ")); Serial.println(settings->ignore_id, HEX);
            cfg_is_updated = true;
          }
          if (D_follow_id.isUpdated()) {
            settings->follow_id = strtoul(D_follow_id.value(), NULL, 16);
            Serial.print(F("Follow ID = ")); Serial.println(settings->follow_id, HEX);
            cfg_is_updated = true;
          }
          if (D_baud_rate.isUpdated()) {
            settings->baud_rate = atoi(D_baud_rate.value());
            Serial.print(F("Baud rate = ")); Serial.println(settings->baud_rate);
            cfg_is_updated = true;
          }
          if (D_power_ext.isUpdated()) {
            settings->power_external = atoi(D_power_ext.value());
            Serial.print(F("Power source = ")); Serial.println(settings->power_external);
            cfg_is_updated = true;
          }
          if (D_NMEA_debug.isUpdated()) {
            settings->nmea_d = atoi(D_NMEA_debug.value());
            Serial.print(F("NMEA_debug = ")); Serial.println(settings->nmea_d);
            cfg_is_updated = true;
          }
          if (D_debug_flags.isUpdated()) {
            settings->debug_flags = atoi(D_debug_flags.value());
            Serial.print(F("Debug flags = ")); Serial.println(settings->debug_flags);
            cfg_is_updated = true;
          }
          if (D_NMEA2.isUpdated())
          {
            int nmea1 = settings->nmea_out;
            int nmea2 = atoi(D_NMEA2.value());
            Serial.print(F("NMEA_Output2 (given) = ")); Serial.println(nmea2);
            if (nmea2 == nmea1)
                nmea2 = DEST_NONE;
            if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
              if ((nmea1==DEST_UART || nmea1==DEST_USB)
               && (nmea2==DEST_UART || nmea2==DEST_USB))
                  nmea2 = DEST_NONE;      // USB & UART wired together
            }
//            bool wireless1 = (nmea1==DEST_UDP || nmea1==DEST_TCP || nmea1==DEST_BLUETOOTH);
//            bool wireless2 = (nmea2==DEST_UDP || nmea2==DEST_TCP || nmea2==DEST_BLUETOOTH);
//            if (wireless1 && wireless2)
//                  nmea2 = DEST_NONE;      // only one wireless output route possible
            Serial.print(F("NMEA_Output2 (adjusted) = ")); Serial.println(nmea2);
            settings->nmea_out2 = nmea2;
            cfg_is_updated = true;
          }
          if (D_NMEA2_gnss.isUpdated())
          {
            settings->nmea2_g = atoi(D_NMEA2_gnss.value());
            Serial.print(F("NMEA2_gnss = ")); Serial.println(settings->nmea_g);
            cfg_is_updated = true;
          }
          if (D_NMEA2_private.isUpdated())
          {
            settings->nmea2_p = atoi(D_NMEA2_private.value());
            Serial.print(F("NMEA2_private = ")); Serial.println(settings->nmea_p);
            cfg_is_updated = true;
          }
          if (D_NMEA2_legacy.isUpdated())
          {
            settings->nmea2_l = atoi(D_NMEA2_legacy.value());
            Serial.print(F("NMEA2_legacy = ")); Serial.println(settings->nmea_l);
            cfg_is_updated = true;
          }
           if (D_NMEA2_sensors.isUpdated())
          {
            settings->nmea2_s = atoi(D_NMEA2_sensors.value());
            Serial.print(F("NMEA2_sensors = ")); Serial.println(settings->nmea_s);
            cfg_is_updated = true;
          }
          if (D_NMEA2_debug.isUpdated()) {
            settings->nmea2_d = atoi(D_NMEA2_debug.value());
            Serial.print(F("NMEA2_debug = ")); Serial.println(settings->nmea_d);
            cfg_is_updated = true;
          }
          if (D_relay.isUpdated()) {
            settings->relay = atoi(D_relay.value());
            Serial.print(F("Relay = ")); Serial.println(settings->relay);
            cfg_is_updated = true;
          }
          if (D_bluetooth.isUpdated()) {
            settings->bluetooth = atoi(D_bluetooth.value());
            Serial.print(F("Bluetooth = ")); Serial.println(settings->bluetooth);
            cfg_is_updated = true;
          }
#if defined(ESP32)
          if (D_altpin0.isUpdated()) {
            settings->altpin0 = atoi(D_altpin0.value());
            Serial.print(F("Use alt RX pin = ")); Serial.println(settings->altpin0);
            cfg_is_updated = true;
          }
          if (D_baudrate2.isUpdated()) {
            settings->baudrate2 = atoi(D_baudrate2.value());
            Serial.print(F("Baud rate 2 = ")); Serial.println(settings->baudrate2);
            cfg_is_updated = true;
          }
          if (D_invert2.isUpdated()) {
            settings->invert2 = atoi(D_invert2.value());
            Serial.print(F("Serial2 logic = ")); Serial.println(settings->invert2);
            cfg_is_updated = true;
          }
#endif
          if (D_extern1.isUpdated()) {
            settings->nmea_e = atoi(D_extern1.value());
            Serial.print(F("NMEA1_ext = ")); Serial.println(settings->nmea_e);
            cfg_is_updated = true;
          }
          if (D_extern2.isUpdated()) {
            settings->nmea2_e = atoi(D_extern2.value());
            Serial.print(F("NMEA2_ext = ")); Serial.println(settings->nmea2_e);
            cfg_is_updated = true;
          }

          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
        }
      }

#if defined(USE_OGN_ENCRYPTION)
      if (S_Version.isUpdated()) {
        if (strncmp(S_Version.value(), "?", 1) == 0) {

          snprintf_P(NMEABuffer, sizeof(NMEABuffer),
              PSTR("$PSRFS,%d,%08X%08X%08X%08X*"),
              PSRFS_VERSION,
              settings->igc_key[0]? 0x88888888 : 0,
              settings->igc_key[1]? 0x88888888 : 0,
              settings->igc_key[2]? 0x88888888 : 0,
              settings->igc_key[3]? 0x88888888 : 0);
              /* mask the key from prying eyes */

          nmea_cfg_send();

        } else if (atoi(S_Version.value()) == PSRFS_VERSION) {
          bool cfg_is_updated = false;

          if (S_IGC_Key.isUpdated())
          {
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
            cfg_is_updated = true;
          }
          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
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

          nmea_cfg_send();

        } else if (atoi(V_Version.value()) == PSKVC_VERSION) {
          bool cfg_is_updated = false;

          if (V_Adapter.isUpdated())
          {
            ui->adapter = atoi(V_Adapter.value());
            Serial.print(F("Adapter = ")); Serial.println(ui->adapter);
            cfg_is_updated = true;
          }
          if (V_Connection.isUpdated())
          {
            ui->connection = atoi(V_Connection.value());
            Serial.print(F("Connection = ")); Serial.println(ui->connection);
            cfg_is_updated = true;
          }
          if (V_Units.isUpdated())
          {
            ui->units = atoi(V_Units.value());
            Serial.print(F("Units = ")); Serial.println(ui->units);
            cfg_is_updated = true;
          }
          if (V_Zoom.isUpdated())
          {
            ui->zoom = atoi(V_Zoom.value());
            Serial.print(F("Zoom = ")); Serial.println(ui->zoom);
            cfg_is_updated = true;
          }
          if (V_Protocol.isUpdated())
          {
            ui->protocol = atoi(V_Protocol.value());
            Serial.print(F("Protocol = ")); Serial.println(ui->protocol);
            cfg_is_updated = true;
          }
          if (V_Baudrate.isUpdated())
          {
            ui->baudrate = atoi(V_Baudrate.value());
            Serial.print(F("Baudrate = ")); Serial.println(ui->baudrate);
            cfg_is_updated = true;
          }
          if (V_Server.isUpdated())
          {
            strncpy(ui->server, V_Server.value(), sizeof(ui->server));
            Serial.print(F("Server = ")); Serial.println(ui->server);
            cfg_is_updated = true;
          }
           if (V_Key.isUpdated())
          {
            strncpy(ui->key, V_Key.value(), sizeof(ui->key));
            Serial.print(F("Key = ")); Serial.println(ui->key);
            cfg_is_updated = true;
          }
          if (V_Rotate.isUpdated())
          {
            ui->rotate = atoi(V_Rotate.value());
            Serial.print(F("Rotation = ")); Serial.println(ui->rotate);
            cfg_is_updated = true;
          }
          if (V_Orientation.isUpdated())
          {
            ui->orientation = atoi(V_Orientation.value());
            Serial.print(F("Orientation = ")); Serial.println(ui->orientation);
            cfg_is_updated = true;
          }
          if (V_AvDB.isUpdated())
          {
            ui->adb = atoi(V_AvDB.value());
            Serial.print(F("AvDB = ")); Serial.println(ui->adb);
            cfg_is_updated = true;
          }
          if (V_ID_Pref.isUpdated())
          {
            ui->idpref = atoi(V_ID_Pref.value());
            Serial.print(F("ID_Pref = ")); Serial.println(ui->idpref);
            cfg_is_updated = true;
          }
           if (V_VMode.isUpdated())
          {
            ui->vmode = atoi(V_VMode.value());
            Serial.print(F("VMode = ")); Serial.println(ui->vmode);
            cfg_is_updated = true;
          }
          if (V_Voice.isUpdated())
          {
            ui->voice = atoi(V_Voice.value());
            Serial.print(F("Voice = ")); Serial.println(ui->voice);
            cfg_is_updated = true;
          }
          if (V_AntiGhost.isUpdated())
          {
            ui->aghost = atoi(V_AntiGhost.value());
            Serial.print(F("AntiGhost = ")); Serial.println(ui->aghost);
            cfg_is_updated = true;
          }
          if (V_Filter.isUpdated())
          {
            ui->filter = atoi(V_Filter.value());
            Serial.print(F("Filter = ")); Serial.println(ui->filter);
            cfg_is_updated = true;
          }
          if (V_PowerSave.isUpdated())
          {
            ui->power_save = atoi(V_PowerSave.value());
            Serial.print(F("PowerSave = ")); Serial.println(ui->power_save);
            cfg_is_updated = true;
          }
          if (V_Team.isUpdated())
          {
            ui->team = strtoul(V_Team.value(), NULL, 16);
            Serial.print(F("Team = ")); Serial.println(ui->team, HEX);
            cfg_is_updated = true;
          }

          if (cfg_is_updated) {
            SoC->WDT_fini();
            if (SoC->Bluetooth_ops) { SoC->Bluetooth_ops->fini(); }
            EEPROM_store();
            nmea_cfg_restart();
          }
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
