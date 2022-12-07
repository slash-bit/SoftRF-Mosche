/*
 * WebHelper.cpp
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

#include <Arduino.h>
#include <TimeLib.h>

#include "SoCHelper.h"
#include "WebHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"

#include "jquery_min_js.h"

static uint32_t prev_rx_pkt_cnt = 0;


byte getVal(char c)
{
   if(c >= '0' && c <= '9')
     return (byte)(c - '0');
   else
     return (byte)(toupper(c)-'A'+10);
}

#if DEBUG
void Hex2Bin(String str, byte *buffer)
{
  char hexdata[2 * PKT_SIZE + 1];
  
  str.toCharArray(hexdata, sizeof(hexdata));
  for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
  {
    buffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
  }
}
#endif

static const char about_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>About</title>\
  </head>\
<body>\
<h1>About</h1>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;SkyStrobe by Moshe Braner</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://github.com/moshe-braner/SoftRF</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;Based on SkyView by Linar Yusupov</h4>\
<h4>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;http://github.com/lyusupov/SoftRF</h4>\
<br>\
<h2>Credits</h2>\
<p align=center>(in historical order)</p>\
<table width=100%%>\
<tr><th align=left>Ivan Grokhotkov</th><td align=left>Arduino core for ESP8266</td></tr>\
<tr><th align=left>Paul Stoffregen</th><td align=left>Arduino Time library</td></tr>\
<tr><th align=left>Mikal Hart</th><td align=left>TinyGPS++ and PString libraries</td></tr>\
<tr><th align=left>Hristo Gochkov</th><td align=left>Arduino core for ESP32</td></tr>\
<tr><th align=left>Ryan David</th><td align=left>GDL90 decoder</td></tr>\
<tr><th align=left>flashrom.org project</th><td align=left>Flashrom library</td></tr>\
<tr><th align=left>Evandro Copercini</th><td align=left>ESP32 BT SPP library</td></tr>\
<tr><th align=left>Tim Eckel and Horst Reiterer</th><td align=left>ToneAC library</td></tr>\
</table>\
<hr>\
Copyright (C) 2022 &nbsp;&nbsp;&nbsp; Moshe Braner\
</body>\
</html>";

void handleSettings() {

  size_t size = 3000;
  char *offset;
  size_t len = 0;
  char *Settings_temp = (char *) malloc(size);

  if (Settings_temp == NULL) {
    return;
  }

  offset = Settings_temp;

  /* Common part 1 */
  snprintf_P ( offset, size,
    PSTR("<html>\
<head>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>Settings</title>\
</head>\
<body>\
<h1 align=center>Settings</h1>\
<form action='/input' method='GET'>\
<table width=100%%>"));

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Strobe</th>\
<td align=right>\
<select name='strobe'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Alarm</option>\
<option %s value='%d'>Airborne</option>\
<option %s value='%d'>Always</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Sound</th>\
<td align=right>\
<select name='sound'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>On</option>\
</select>\
</td>\
</tr>\
<tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
<tr>\
<th align=left>Input type</th>\
<td align=right>\
<select name='connection'>\
<option %s value='%d'>Serial</option>\
<option %s value='%d'>WiFi UDP</option>\
<option %s value='%d'>Bluetooth SPP</option>\
<option %s value='%d'>Bluetooth LE</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>NMEA</option>\
<option %s value='%d'>GDL90</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Baud rate</th>\
<td align=right>\
<select name='baudrate'>\
<option %s value='%d'>4800</option>\
<option %s value='%d'>9600</option>\
<option %s value='%d'>19200</option>\
<option %s value='%d'>38400</option>\
<option %s value='%d'>57600</option>"),
  (settings->strobe == STROBE_OFF ? "selected" : ""), STROBE_OFF,
  (settings->strobe == STROBE_ALARM ? "selected" : ""), STROBE_ALARM,
  (settings->strobe == STROBE_AIRBORNE ? "selected" : ""), STROBE_AIRBORNE,
  (settings->strobe == STROBE_ALWAYS ? "selected" : ""), STROBE_ALWAYS,
  (settings->sound == SOUND_OFF ? "selected" : ""), SOUND_OFF,
  (settings->sound == SOUND_ON ? "selected" : ""), SOUND_ON,
  (settings->connection == CON_SERIAL        ? "selected" : ""), CON_SERIAL,
  (settings->connection == CON_WIFI_UDP      ? "selected" : ""), CON_WIFI_UDP,
  (settings->connection == CON_BLUETOOTH_SPP ? "selected" : ""), CON_BLUETOOTH_SPP,
  (settings->connection == CON_BLUETOOTH_LE  ? "selected" : ""), CON_BLUETOOTH_LE,
  (settings->protocol   == PROTOCOL_NMEA     ? "selected" : ""), PROTOCOL_NMEA,
  (settings->protocol   == PROTOCOL_GDL90    ? "selected" : ""), PROTOCOL_GDL90,
  (settings->baudrate   == B4800             ? "selected" : ""), B4800,
  (settings->baudrate   == B9600             ? "selected" : ""), B9600,
  (settings->baudrate   == B19200            ? "selected" : ""), B19200,
  (settings->baudrate   == B38400            ? "selected" : ""), B38400,
  (settings->baudrate   == B57600            ? "selected" : ""), B57600
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 2 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>115200</option>\
<option %s value='%d'>2000000</option>"),
    (settings->baudrate   == B115200        ? "selected" : ""), B115200,
    (settings->baudrate   == B2000000       ? "selected" : ""), B2000000
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  }

    /* Common part 3 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Source Id</th>\
<td align=right>\
<INPUT type='text' name='server' maxlength='21' size='15' value='%s'>\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
&nbsp;&nbsp;Key</th>\
<td align=right>\
<INPUT type='text' name='key' maxlength='17' size='15' value='%s'>\
</td>\
</tr>\
<tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
<tr>\
<th align=left>Bridge Output</th>\
<td align=right>\
<select name='bridge'>\
<option %s value='%d'>None</option>\
<option %s value='%d'>Serial</option>\
<option %s value='%d'>WiFi UDP</option>\
<option %s value='%d'>Bluetooth SPP</option>\
<option %s value='%d'>Bluetooth LE</option>\
</select>\
</td>\
</tr>\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>"),
  settings->server, settings->key,
  (settings->bridge == BRIDGE_NONE           ? "selected" : ""), BRIDGE_NONE,
  (settings->bridge == BRIDGE_SERIAL         ? "selected" : ""), BRIDGE_SERIAL,
  (settings->bridge == BRIDGE_UDP            ? "selected" : ""), BRIDGE_UDP,
  (settings->bridge == BRIDGE_BT_SPP         ? "selected" : ""), BRIDGE_BT_SPP,
  (settings->bridge == BRIDGE_BT_LE          ? "selected" : ""), BRIDGE_BT_LE );

  SoC->swSer_enableRx(false);
  server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
  server.sendHeader(String(F("Pragma")), String(F("no-cache")));
  server.sendHeader(String(F("Expires")), String(F("-1")));
  server.send ( 200, "text/html", Settings_temp );
  SoC->swSer_enableRx(true);
  free(Settings_temp);
}

void handleRoot() {
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;
  bool low_voltage = (Battery_voltage() <= Battery_threshold());

  time_t timestamp = now();
  char str_Vcc[8];

  size_t size = 2000;
  char *offset;
  size_t len = 0;

  char *Root_temp = (char *) malloc(size);
  if (Root_temp == NULL) {
    return;
  }
  offset = Root_temp;

  dtostrf(vdd, 4, 2, str_Vcc);

  snprintf_P ( offset, size,
    PSTR("<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SkyStrobe status</title>\
  </head>\
<body>\
 <table width=100%%>\
  <tr><!-- <td align=left><h1>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</h1></td> -->\
  <td align=center><h1>SkyStrobe status</h1></td>\
  <!-- <td align=right><img src='/logo.png'></td> --></tr>\
 </table>\
 <table width=100%%>\
  <tr><th align=left>Device Id</th><td align=right>%06X</td></tr>\
  <tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>\
  <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  <tr><th align=left>Free memory</th><td align=right>%u</td></tr>\
  <tr><th align=left>Battery voltage</th><td align=right><font color=%s>%s</font></td></tr>\
  <tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
  <tr><th align=left>Strobe mode</th><td align=right>%s</td></tr>\
  <tr><th align=left>Sound</th><td align=right>%s</td></tr>\
  <tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
  <tr><th align=left>Input type</th><td align=right>%s</td></tr>"),
    SoC->getChipId() & 0xFFFFFF, SKYSTROBE_FIRMWARE_VERSION,
    (SoC == NULL ? "NONE" : SoC->name),
    hr, min % 60, sec % 60, ESP.getFreeHeap(),
    low_voltage ? "red" : "green", str_Vcc,
    settings->strobe == STROBE_ALARM    ? "Alarm" :
    settings->strobe == STROBE_AIRBORNE ? "Airborne" :
    settings->strobe == STROBE_ALWAYS   ? "Always" : "Off",
    settings->sound == SOUND_ON   ? "On" : "Off",
    settings->connection == CON_SERIAL        ? "Serial" :
    settings->connection == CON_BLUETOOTH_SPP ? "Bluetooth SPP" :
    settings->connection == CON_BLUETOOTH_LE  ? "Bluetooth LE" :
    settings->connection == CON_WIFI_UDP      ? "WiFi" : "NONE"
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  if (settings->connection == CON_WIFI_UDP) {
    snprintf_P ( offset, size,
      PSTR("\
  <tr><th align=left>Link partner</th><td align=right>%s</td></tr>\
  <tr><th align=left>Link status</th><td align=right>%s established</td></tr>\
  <tr><th align=left>Assigned IP address</th><td align=right>%s</td></tr>"),
      settings->server && strlen(settings->server) > 0 ? settings->server : "NOT SET",
      WiFi.status() == WL_CONNECTED ? "" : "not",
      WiFi.localIP().toString().c_str()
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  }
  
  switch (settings->protocol)
    {
    case PROTOCOL_GDL90:
      snprintf_P ( offset, size,
        PSTR("\
  <tr><th align=left>Data status</th><td align=right>%s connected</td></tr>\
  <tr><th align=left>Data type</th><td align=right>%s %s</td></tr>\
  "),
        GDL90_isConnected()  ? "" : "not",
        GDL90_isConnected()  && !GDL90_hasHeartBeat() ? "UNK" : "",
        GDL90_hasHeartBeat() ? "GDL90"  : "--"
      );
      break;
    case PROTOCOL_NMEA:
    default:
      snprintf_P ( offset, size,
        PSTR("\
  <tr><th align=left>Data status</th><td align=right>%s active</td></tr>\
  <tr><th align=left>Data type</th><td align=right>%s %s %s</td></tr>\
  "),
        NMEA_isConnected() ? "" : "not",
        NMEA_isConnected() && !(NMEA_hasGNSS() || NMEA_hasFLARM()) ? "OTHER" : "",
        NMEA_hasGNSS()     ? "GNSS"  : "",
        NMEA_hasFLARM()    ? "FLARM" : ""
      );
      break;
    }

    len = strlen(offset);
    offset += len;
    size -= len;

  snprintf_P ( offset, size,
    PSTR("\
  <tr><th align=left>Bridge output</th><td align=right>%s</td></tr>\
 </table>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=left><input type=button onClick=\"location.href='/settings'\" value='Settings'></td>\
    <td align=center><input type=button onClick=\"location.href='/about'\" value='About'></td>\
    <td align=right><input type=button onClick=\"location.href='/firmware'\" value='Firmware update'></td>\
  </tr>\
 </table>\
</body>\
</html>"),
    settings->bridge == BRIDGE_BT_SPP ? "Bluetooth SPP" :
    settings->bridge == BRIDGE_BT_LE  ? "Bluetooth LE" :
    settings->bridge == BRIDGE_UDP    ? "WiFi UDP" :
    settings->bridge == BRIDGE_SERIAL ? "Serial" : "NONE"
  );

  SoC->swSer_enableRx(false);
  server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
  server.sendHeader(String(F("Pragma")), String(F("no-cache")));
  server.sendHeader(String(F("Expires")), String(F("-1")));
  server.send ( 200, "text/html", Root_temp );
  SoC->swSer_enableRx(true);
  free(Root_temp);
}

void handleInput() {

  char *Input_temp = (char *) malloc(1900);
  if (Input_temp == NULL) {
    return;
  }

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i).equals("connection")) {
      settings->connection = server.arg(i).toInt();
    } else if (server.argName(i).equals("bridge")) {
      settings->bridge = server.arg(i).toInt();
    } else if (server.argName(i).equals("protocol")) {
      settings->protocol = server.arg(i).toInt();
    } else if (server.argName(i).equals("baudrate")) {
      settings->baudrate = server.arg(i).toInt();
    } else if (server.argName(i).equals("server")) {
      server.arg(i).toCharArray(settings->server, sizeof(settings->server));
    } else if (server.argName(i).equals("key")) {
      server.arg(i).toCharArray(settings->key, sizeof(settings->key));
    } else if (server.argName(i).equals("strobe")) {
      settings->strobe = server.arg(i).toInt();
    } else if (server.argName(i).equals("sound")) {
      settings->sound = server.arg(i).toInt();
    }
  }

  if (settings->connection != CON_SERIAL
         && settings->bridge != BRIDGE_SERIAL)     // disallow wireless->wireless bridge
     settings->bridge = BRIDGE_NONE;

  snprintf_P ( Input_temp, 2000,
PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='15; url=/'>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>SkyStrobe Settings</title>\
</head>\
<body>\
<h1 align=center>New settings:</h1>\
<table width=100%%>\
<tr><th align=left>Strobe</th><td align=right>%d</td></tr>\
<tr><th align=left>Sound</th><td align=right>%d</td></tr>\
<tr><th align=left>Connection</th><td align=right>%d</td></tr>\
<tr><th align=left>Protocol</th><td align=right>%d</td></tr>\
<tr><th align=left>Baud rate</th><td align=right>%d</td></tr>\
<tr><th align=left>Server</th><td align=right>%s</td></tr>\
<tr><th align=left>Key</th><td align=right>%s</td></tr>\
<tr><th align=left>Bridge</th><td align=right>%d</td></tr>\
</table>\
<hr>\
  <p align=center><h1 align=center>Restart is in progress... Please, wait!</h1></p>\
</body>\
</html>"),
  settings->strobe, settings->sound, settings->connection, settings->protocol,
  settings->baudrate, settings->server, settings->key, settings->bridge );

  SoC->swSer_enableRx(false);
  server.send ( 200, "text/html", Input_temp );
//  SoC->swSer_enableRx(true);
  delay(1000);
  free(Input_temp);
  EEPROM_store();
  delay(1000);
  ESP.restart();
}

void handleNotFound() {

  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text/plain", message );
}

void Web_setup()
{
  server.on ( "/", handleRoot );
  server.on ( "/settings", handleSettings );
  server.on ( "/about", []() {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), about_html);
    SoC->swSer_enableRx(true);
  } );

  server.on ( "/input", handleInput );
  server.on ( "/inline", []() {
    server.send ( 200, "text/plain", "this works as well" );
  } );
  server.on("/firmware", HTTP_GET, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), String(F("*")));
    server.send_P(200,
      PSTR("text/html"),
      PSTR("\
<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Firmware update</title>\
  </head>\
<body>\
<body>\
 <h1 align=center>Firmware update</h1>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=left>\
<script src='/jquery.min.js'></script>\
<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>\
    <input type='file' name='update'>\
    <input type='submit' value='Update'>\
</form>\
<div id='prg'>progress: 0%</div>\
<script>\
$('form').submit(function(e){\
    e.preventDefault();\
      var form = $('#upload_form')[0];\
      var data = new FormData(form);\
       $.ajax({\
            url: '/update',\
            type: 'POST',\
            data: data,\
            contentType: false,\
            processData:false,\
            xhr: function() {\
                var xhr = new window.XMLHttpRequest();\
                xhr.upload.addEventListener('progress', function(evt) {\
                    if (evt.lengthComputable) {\
                        var per = evt.loaded / evt.total;\
                        $('#prg').html('progress: ' + Math.round(per*100) + '%');\
                    }\
               }, false);\
               return xhr;\
            },\
            success:function(d, s) {\
                console.log('success!')\
           },\
            error: function (a, b, c) {\
            }\
          });\
});\
</script>\
    </td>\
  </tr>\
 </table>\
</body>\
</html>")
    );
  SoC->swSer_enableRx(true);
  });
  server.onNotFound ( handleNotFound );

  server.on("/update", HTTP_POST, [](){
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), "*");
    server.send(200, String(F("text/plain")), (Update.hasError())?"FAIL":"OK");
//    SoC->swSer_enableRx(true);
    delay(1000);
    ESP.restart();
  },[](){
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      Serial.setDebugOutput(true);
      SoC->WiFiUDP_stopAll();
      SoC->WDT_fini();
      Serial.printf("Update: %s\r\n", upload.filename.c_str());
      uint32_t maxSketchSpace = SoC->maxSketchSpace();
      if(!Update.begin(maxSketchSpace)){//start with max available size
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_WRITE){
      if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
        Update.printError(Serial);
      }
    } else if(upload.status == UPLOAD_FILE_END){
      if(Update.end(true)){ //true to set the size to the current progress
        Serial.printf("Update Success: %u\r\nRebooting...\r\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });

//#if !defined(NOLOGO)
#if 0
  server.on ( "/logo.png", []() {
    server.send_P ( 200, "image/png", Logo, sizeof(Logo) );
  } );
#endif

  server.on ( "/jquery.min.js", []() {

    PGM_P content = jquery_min_js_gz;
    size_t bytes_left = jquery_min_js_gz_len;
    size_t chunk_size;

    server.setContentLength(bytes_left);
    server.sendHeader(String(F("Content-Encoding")),String(F("gzip")));
    server.send(200, String(F("application/javascript")), "");

    do {
      chunk_size = bytes_left > JS_MAX_CHUNK_SIZE ? JS_MAX_CHUNK_SIZE : bytes_left;
      server.sendContent_P(content, chunk_size);
      content += chunk_size;
      bytes_left -= chunk_size;
    } while (bytes_left > 0) ;

  } );

  server.begin();
  Serial.println (F("HTTP server has started at port: 80"));

  delay(1000);
}

void Web_loop()
{
  server.handleClient();
}

void Web_fini()
{
  server.stop();
}
