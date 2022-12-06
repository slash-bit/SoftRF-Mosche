/*
 * WiFiHelper.cpp
 * Copyright (C) 2016-2022 Linar Yusupov
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

#include <FS.h>
#include <TimeLib.h>

#include "EEPROMHelper.h"
#include "SoCHelper.h"
#include "WiFiHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "WebHelper.h"
#include "Sound.h"

#include "SkyStrobe.h"

String host_name = HOSTNAME;

IPAddress local_IP(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

/**
 * Default WiFi connection information.
 *
 */
const char* ap_default_psk = "12345678"; ///< Default PSK.

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Uni_Udp;

unsigned int UDP_Data_Port = 0;           // local port to listen for UDP packets

char UDPpacketBuffer[UDP_PACKET_BUFSIZE]; // buffer to hold incoming packets

static uint32_t WiFi_STA_TimeMarker = 0;
static bool WiFi_STA_connected = false;
static bool WiFi_STA_trying = false;
static uint32_t connecting_timemarker = 0;
static bool reconnecting = false;

#if defined(POWER_SAVING_WIFI_TIMEOUT)
static unsigned long WiFi_No_Clients_Time_ms = 0;
#endif

size_t WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  int noBytes = Uni_Udp.parsePacket();
  if ( noBytes ) {

    if (noBytes > max_size) {
      noBytes = max_size;
    }

    // We've received a packet, read the data from it
    Uni_Udp.read(buf,noBytes); // read the packet into the buffer

    return (size_t) noBytes;
  } else {
    return 0;
  }
}

void WiFi_Transmit_UDP(char *buf, size_t size)
{
    SoC->WiFi_Transmit_UDP(
        (settings->protocol==PROTOCOL_GDL90 ? GDL90_DST_PORT : NMEA_UDP_PORT),
          (byte *)buf, size);
}

/*
 * Setup WiFi in simultaneous modes STA (for incoming data) and AP (for web interface).
 */
void WiFi_setup()
{
  // Set Hostname.
  host_name += String((SoC->getChipId() & 0xFFFFFF), HEX);
  SoC->WiFi_hostname(host_name);

  // Print hostname.
  Serial.println("Hostname: " + host_name);

  WiFiMode_t mode = (settings->connection == CON_WIFI_UDP ||
                     settings->connection == CON_WIFI_TCP ) ?
                     WIFI_AP_STA : WIFI_AP;
                     // note: WIFI_AP_STA, not WIFI_STA
  WiFi.mode(mode);

  SoC->WiFi_setOutputPower(WIFI_TX_POWER_MED); // 10 dB
  // WiFi.setOutputPower(0); // 0 dB
  //system_phy_set_max_tpw(4 * 0); // 0 dB
  delay(10);

  Serial.print(F("Setting soft-AP configuration ... "));
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ?
    F("Ready") : F("Failed!"));

  Serial.print(F("Setting up soft-AP ... "));
  Serial.println(WiFi.softAP(host_name.c_str(), ap_default_psk) ?
    F("Ready") : F("Failed!"));

  Serial.print(F("Soft-AP IP address: "));
  Serial.println(WiFi.softAPIP());

  if (settings->connection == CON_WIFI_UDP) {
    switch (settings->protocol)
    {
    case PROTOCOL_NMEA:
      UDP_Data_Port = NMEA_UDP_PORT;
      break;
    case PROTOCOL_GDL90:
      UDP_Data_Port = GDL90_DST_PORT;
      break;
    default:
      UDP_Data_Port = 0;
      break;
    }

    if (UDP_Data_Port) {
      Uni_Udp.begin(UDP_Data_Port);
      Serial.print(F("UDP server has started at port: "));
      Serial.println(UDP_Data_Port);
    }
  }

  if (settings->connection == CON_WIFI_UDP ||
      settings->connection == CON_WIFI_TCP ) {
    if (strnlen(settings->server, sizeof(settings->server)) > 0 &&
        strnlen(settings->key,  sizeof(settings->key))  > 0) {
      WiFi.begin(settings->server, settings->key);

      Serial.print(F("Waiting for WiFi connection to "));
      Serial.print(settings->server);
      Serial.println(F(" AP..."));
    }

    WiFi_STA_connected = false;
    WiFi_STA_trying = true;
    WiFi_STA_TimeMarker = millis();
    reconnecting = false;
    connecting_timemarker = millis();    // blink LED as long as not connected
  }

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  WiFi_No_Clients_Time_ms = millis();
#endif
}

void WiFi_loop()
{
  if (settings->connection == CON_WIFI_UDP ||
      settings->connection == CON_WIFI_TCP ) {
    if (WiFi.status() == WL_CONNECTED) {
      if (WiFi_STA_connected == false) {
        WiFi_STA_connected = true;
        WiFi_STA_trying = false;
        Serial.print(F("Connected to WiFi AP "));
        Serial.println(settings->server);
        Serial.print(F("IP address: "));
        Serial.println(WiFi.localIP());
        red_LED(false);
        blue_LED(true);
        connecting_timemarker = 0;
      }
    } else {
      if (WiFi_STA_connected == true) {
        WiFi_STA_connected = false;
        Serial.print(F("Disconnected from WiFi AP "));
        Serial.println(settings->server);
        red_LED(true);
        WiFi.reconnect();
        WiFi_STA_trying = true;
        WiFi_STA_TimeMarker = millis();
        connecting_timemarker = millis();     // will cause blinking red below
        reconnecting = true;
      }
    }

#if 1
    if (WiFi_STA_connected == false) {
      if (millis() - WiFi_STA_TimeMarker > (WiFi_STA_trying ? 3000 : 6000)) {
        if (WiFi_STA_trying) {
          //Serial.println("Pausing STA connection attempt");
          WiFi.enableSTA(false);
          WiFi_STA_trying = false;
          connecting_timemarker = 0;          // stop blinking
        } else {
          //Serial.println("Resuming STA connection attempt");
          WiFi.enableSTA(true);
          WiFi.reconnect();
          WiFi_STA_trying = true;
          connecting_timemarker = millis();   // resume blinking
          reconnecting = true;
        }
        WiFi_STA_TimeMarker = millis();
      }
    }
#endif
  }

  if (connecting_timemarker > 0 && millis() > connecting_timemarker + 500) {
      if(reconnecting) {
        green_LED(false);
        toggle_red_LED();      // reconnections blink red
      } else {
        red_LED(false);
        toggle_green_LED();    // only initial connection blinks green
      }
      connecting_timemarker = millis();     // keep blinking as long as not connected
  }

#if defined(POWER_SAVING_WIFI_TIMEOUT)
  // if (settings->power_save & POWER_SAVE_WIFI)
  if (WiFi.getMode() == WIFI_AP && settings->bridge == BRIDGE_NONE) {
    if (SoC->WiFi_clients_count() == 0) {
      if ((millis() - WiFi_No_Clients_Time_ms) > POWER_SAVING_WIFI_TIMEOUT) {
        Web_fini();
        WiFi_fini();
        Serial.println(F("WIFI OFF"));
        blue_LED(false);
        red_LED(true);
      }
    } else {
      WiFi_No_Clients_Time_ms = millis();
    }
  }
#endif
}

void WiFi_fini()
{
  if (UDP_Data_Port)
    Uni_Udp.stop();

  WiFi.mode(WIFI_OFF);
}
