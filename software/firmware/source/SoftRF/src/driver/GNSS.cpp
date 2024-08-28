/*
 * GNSSHelper.cpp
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

// this file based on v1.2.

#if defined(ARDUINO)
#include <Arduino.h>
#endif
#include <TimeLib.h>

#include "GNSS.h"
#include "EEPROM.h"
#include "../protocol/data/NMEA.h"
#include "../system/SoC.h"
#include "WiFi.h"
#include "RF.h"
#include "Battery.h"
#include "../protocol/data/D1090.h"

#if !defined(EXCLUDE_EGM96)
#include <egm96s.h>
#endif /* EXCLUDE_EGM96 */

//#define DO_GNSS_DEBUG

#if !defined(DO_GNSS_DEBUG)
#define GNSS_DEBUG_PRINT
#define GNSS_DEBUG_PRINTLN
#else
#define GNSS_DEBUG_PRINT    Serial.print
#define GNSS_DEBUG_PRINTLN  Serial.println
#endif

#if !defined(GNSS_FLUSH)
#define GNSS_FLUSH()        Serial_GNSS_Out.flush()
#endif

bool gnss_needs_reset = false;

static bool is_prime_mk2 = false;

uint32_t GNSSTimeSyncMarker = 0;
volatile unsigned long PPS_TimeMarker = 0;

#if defined(ESP32)
static uint8_t get_pps_pin()
{
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK3)
        return SOC_GPIO_PIN_S3_GNSS_PPS;
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2) {
        if (settings->gnss_pins == EXT_GNSS_NONE) {
            if (hw_info.revision >= 8 || settings->ppswire)
                return SOC_GPIO_PIN_TBEAM_V08_PPS;
        } else if (settings->ppswire) {
            if (hw_info.revision < 8) {
                if (settings->gnss_pins == EXT_GNSS_39_4)
                    return SOC_GPIO_PIN_VOICE;   // pin 25 for PPS - VP is used for rx
                if (settings->baudrate2 != BAUD_DEFAULT) {
                    // if VP is used for aux serial, use pin 25 for PPS if possible
                    if (settings->gnss_pins == EXT_GNSS_13_2)
                        return SOC_GPIO_PIN_VOICE;
                    return SOC_UNUSED_PIN;
                }
            } else {   // hw_info.revision >= 8
                if (settings->sd_card == SD_CARD_13_VP) {
                    if (settings->voice == VOICE_OFF && settings->strobe == STROBE_OFF)
                        return SOC_GPIO_PIN_VOICE;   // pin 25 for PPS
                    return SOC_UNUSED_PIN;
                }
            }
            return Serial0AltRxPin;     // "VP" pin used for external GNSS PPS
        }
    }
    return SOC_UNUSED_PIN;
}
#else
static uint8_t get_pps_pin() { return SOC_GPIO_PIN_GNSS_PPS; }
#endif

const gnss_chip_ops_t *gnss_chip = NULL;
extern const gnss_chip_ops_t goke_ops; /* forward declaration */

boolean gnss_set_sucess = false ;
TinyGPSPlus gnss;  // Create an Instance of the TinyGPS++ object called gnss

uint8_t GNSSbuf[250]; // at least 3 lines of 80 characters each
                      // and 40+30*N bytes for "UBX-MON-VER" payload

int GNSS_cnt           = 0;
uint16_t FW_Build_Year = 2000 + ((__DATE__[ 9]) - '0') * 10 + ((__DATE__[10]) - '0');

const char *GNSS_name[] = {
  [GNSS_MODULE_NONE]    = "NONE",
  [GNSS_MODULE_NMEA]    = "NMEA",
  [GNSS_MODULE_U6]      = "U6",
  [GNSS_MODULE_U7]      = "U7",
  [GNSS_MODULE_U8]      = "U8",
  [GNSS_MODULE_U9]      = "U9",
  [GNSS_MODULE_U10]     = "U10",
  [GNSS_MODULE_U11]     = "U11",
  [GNSS_MODULE_MAV]     = "MAV",
  [GNSS_MODULE_SONY]    = "SONY",
  [GNSS_MODULE_AT65]    = "AT65",
  [GNSS_MODULE_MT33]    = "MT33",
  [GNSS_MODULE_GOKE]    = "GOKE"
};

#if defined(ENABLE_GNSS_STATS)
/*
 * Sony: GGA -  24 , RMC -  38
 * L76K: GGA -  70+, RMC - 135+
 * Goke: GGA - 185+, RMC - 265+
 * Neo6: GGA - 138 , RMC -  67
 * MT33: GGA -  48 , RMC - 175
 */

gnss_stat_t gnss_stats;
#endif /* ENABLE_GNSS_STATS */

bool nmea_handshake(const char *req, const char *resp, bool skipline)
{
  bool rval = false;

  if (resp == NULL || strlen(resp) == 0) {
    return rval;
  }

  // clean any leftovers
  Serial_GNSS_In.flush();

  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }

  unsigned long start_time = millis();
  unsigned long timeout_ms = (req == NULL ? 3000 : 2000) ;

  while ((millis() - start_time) < timeout_ms) {

    while (Serial_GNSS_In.read() != '\n' && (millis() - start_time) < timeout_ms) { yield(); }

    delay(5);

    /* wait for pause after NMEA burst */
    if (req && Serial_GNSS_In.available() > 0) {
      continue;
    } else {
      /* send request */
      if (req) {
        Serial_GNSS_Out.write((uint8_t *) req, strlen(req));
        GNSS_FLUSH();
      }

      /* skip first line when expected response contains 2 of them */
      if (skipline) {
        start_time = millis();
        while (Serial_GNSS_In.read() != '\n' && (millis() - start_time) < timeout_ms) { yield(); }
      }

      int i=0;
      char c;

      /* take response into buffer */
      while ((millis() - start_time) < timeout_ms) {

        c = Serial_GNSS_In.read();

        if (isPrintable(c) || c == '\r' || c == '\n') {
          if (i >= sizeof(GNSSbuf)) break;
          GNSSbuf[i++] = c;
        } else {
          /* ignore */
          continue;
        }

        if (c == '\n') break;
      }

      if (!strncmp((char *) &GNSSbuf[0], resp, strlen(resp))) {
        rval = true;
        break;
      }
    }
  }

  return rval;
}

static gnss_id_t generic_nmea_probe()
{
  return nmea_handshake(NULL, "$G", false) ? GNSS_MODULE_NMEA : GNSS_MODULE_NONE;
}

static bool generic_nmea_setup()
{
  return true;
}

static void generic_nmea_loop()
{

}

static void generic_nmea_fini()
{

}

const gnss_chip_ops_t generic_nmea_ops = {
  generic_nmea_probe,
  generic_nmea_setup,
  generic_nmea_loop,
  generic_nmea_fini,
  /* use Ublox timing values for 'generic NMEA' module */
  138 /* GGA */, 67 /* RMC */
};

#if !defined(EXCLUDE_GNSS_UBLOX)
 /* CFG-MSG */
const uint8_t setGLL[] PROGMEM = {0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t setGSV[] PROGMEM = {0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
const uint8_t setVTG[] PROGMEM = {0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
#if !defined(NMEA_TCP_SERVICE)
const uint8_t setGSA[] PROGMEM = {0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
#else
/* <<< what does TCP have to do with it? */
const uint8_t setGSA[] PROGMEM = {0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
#endif
 /* CFG-PRT */
uint8_t setBR[] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96,
                   0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};

const uint8_t setNav5[] PROGMEM = {0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00,
                                   0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
                                   0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00};

 /* CFG-RST */
const uint8_t CFG_RST[12] PROGMEM = { 0xb5, 0x62, 0x06, 0x04,
                                      0x04, 0x00,     // payload is 4 bytes
                                      0x00, 0x00,     // navBbrMask
                                      0x01,           // resetMode   
                                      0x00, 0x0F, 0x66};

 /* CFG-RST */
//const uint8_t CFG_RST_COLD[12] PROGMEM = { 0xB5, 0x62, 0x06, 0x04,
//                                           0x04, 0x00,     // payload is 4 bytes
//                                           0xFF, 0xB9,     // navBbrMask
//                                           0x00,           // resetMode 0x00 = Hardware reset
//                                           0x00, 0xC6, 0x8B };

const uint8_t CFG_RST_COLD[] PROGMEM = { 0xFF, 0xB9, 0x00, 0x00 };

const uint8_t RXM_PMREQ_OFF[16] PROGMEM = {0xb5, 0x62, 0x02, 0x41, 0x08, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
                                           0x00, 0x00, 0x4d, 0x3b};
 /* CFG-CFG */
//const uint8_t factoryUBX[] PROGMEM = { 0xB5, 0x62, 0x06, 0x09,
//                                       0x0D, 0x00,              // payload is 13 bytes
//                                       0xFF, 0xFB, 0x00, 0x00,  // clearMask
//                                       0x00, 0x00, 0x00, 0x00,  // saveMask
//                                       0xFF, 0xFF, 0x00, 0x00,  // loadMask
//                                       0x17,                    // deviceMask
//                                       0x2B, 0x7E } ;
// 
// adapted code from github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/GPS/UBlox_Recovery
//
//const uint8_t cfg_clear2[] PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
const uint8_t cfg_clear2[] PROGMEM =
 { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };
//const uint8_t cfg_clear4[] PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x1B, 0xA1};
const uint8_t cfg_clear4[] PROGMEM =
 { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04 };
//const uint8_t cfg_clear1[] PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
const uint8_t cfg_clear1[] PROGMEM =
 { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
//const uint8_t cfg_load3[]  PROGMEM = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00,
//                                  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x07, 0x21, 0xB7};
const uint8_t cfg_load3[]  PROGMEM =
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x07 };

#if defined(USE_GNSS_PSM)
static bool gnss_psm_active = false;

/* Max Performance Mode (default) */
const uint8_t RXM_MAXP[] PROGMEM = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};

/* Power Save Mode */
const uint8_t RXM_PSM[] PROGMEM  = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
#endif /* USE_GNSS_PSM */

uint8_t makeUBXCFG(uint8_t cl, uint8_t id, uint8_t msglen, const uint8_t *msg)
{
  if (msglen > (sizeof(GNSSbuf) - 8) ) {
    msglen = sizeof(GNSSbuf) - 8;
  }

  // Construct the UBX packet
  GNSSbuf[0] = 0xB5;   // header
  GNSSbuf[1] = 0x62;   // header
  GNSSbuf[2] = cl;  // class
  GNSSbuf[3] = id;     // id
  GNSSbuf[4] = msglen; // length
  GNSSbuf[5] = 0x00;

  GNSSbuf[6+msglen] = 0x00; // CK_A
  GNSSbuf[7+msglen] = 0x00; // CK_B

  for (int i = 2; i < 6; i++) {
    GNSSbuf[6+msglen] += GNSSbuf[i];
    GNSSbuf[7+msglen] += GNSSbuf[6+msglen];
  }

  for (int i = 0; i < msglen; i++) {
    GNSSbuf[6+i] = pgm_read_byte(&msg[i]);
    GNSSbuf[6+msglen] += GNSSbuf[6+i];
    GNSSbuf[7+msglen] += GNSSbuf[6+msglen];
  }
  GNSS_DEBUG_PRINT("prepared:");
  for (int i = 0; i < msglen+8; i++) {
    GNSS_DEBUG_PRINT(GNSSbuf[i], HEX);
    GNSS_DEBUG_PRINT(",");
  }
  GNSS_DEBUG_PRINTLN("");
  return (msglen + 8);
}

// Send a byte array of UBX protocol to the GPS
static void sendUBX(const uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial_GNSS_Out.write( MSG[i]);
    GNSS_DEBUG_PRINT(MSG[i], HEX);
    GNSS_DEBUG_PRINT(",");
  }
  GNSS_DEBUG_PRINTLN("");
//  Serial_GNSS_Out.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
static boolean getUBX_ACK(uint8_t cl, uint8_t id) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[2] = {cl, id};
  unsigned long startTime = millis();
  GNSS_DEBUG_PRINT(F(" * Reading ACK response: "));

  // Construct the expected ACK packet
  makeUBXCFG(0x05, 0x01, 2, ackPacket);

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      GNSS_DEBUG_PRINTLN(F(" (SUCCESS!)"));
      return true;
    }

    // Timeout if no valid response in 2 seconds
    if (millis() - startTime > 2000) {
      GNSS_DEBUG_PRINTLN(F(" (FAILED!)"));
      return false;
    }

    // Make sure data is available to read
    if (Serial_GNSS_In.available()) {
      b = Serial_GNSS_In.read();
      GNSS_DEBUG_PRINT(b, HEX);
      if (b == 0xA)
          GNSS_DEBUG_PRINTLN("");
      else
          GNSS_DEBUG_PRINT(",");

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == GNSSbuf[ackByteID]) {
        ackByteID++;
        //GNSS_DEBUG_PRINT(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
    yield();
  }
}

static void setup_UBX()
{
  uint8_t msglen;

#if 0
  unsigned int baudrate = 38400;

  setBR[ 8] = (baudrate      ) & 0xFF;
  setBR[ 9] = (baudrate >>  8) & 0xFF;
  setBR[10] = (baudrate >> 16) & 0xFF;

  SoC->swSer_begin(9600);

  Serial.print(F("Switching baud rate onto "));
  Serial.println(baudrate);

  msglen = makeUBXCFG(0x06, 0x00, sizeof(setBR), setBR);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x00);

  if (!gnss_set_sucess) {
    Serial.print(F("WARNING: Unable to set baud rate onto "));
    Serial.println(baudrate); 
  }
  Serial_GNSS_In.flush();
  SoC->swSer_begin(baudrate);
#endif

  GNSS_DEBUG_PRINTLN(F("Airborne <2g navigation mode: "));

  // Set the navigation mode (Airborne, < 2g)
  msglen = makeUBXCFG(0x06, 0x24, sizeof(setNav5), setNav5);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x24);

  if (!gnss_set_sucess) {
    GNSS_DEBUG_PRINTLN(F("WARNING: Unable to set airborne <2g navigation mode."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GLL: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setGLL), setGLL);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    GNSS_DEBUG_PRINTLN(F("WARNING: Unable to disable NMEA GLL."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GSV: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setGSV), setGSV);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    GNSS_DEBUG_PRINTLN(F("WARNING: Unable to disable NMEA GSV."));
  }

  GNSS_DEBUG_PRINTLN(F("Switching off NMEA VTG: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setVTG), setVTG);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    GNSS_DEBUG_PRINTLN(F("WARNING: Unable to disable NMEA VTG."));
  }

#if !defined(NMEA_TCP_SERVICE)

  /* <<< what does TCP have to do with it? */
#if 0
  GNSS_DEBUG_PRINTLN(F("Switching off NMEA GSA: "));

  msglen = makeUBXCFG(0x06, 0x01, sizeof(setGSA), setGSA);
  sendUBX(GNSSbuf, msglen);
  gnss_set_sucess = getUBX_ACK(0x06, 0x01);

  if (!gnss_set_sucess) {
    GNSS_DEBUG_PRINTLN(F("WARNING: Unable to disable NMEA GSA."));
  }
#endif

#endif
}

/* ------ BEGIN -----------  https://github.com/Black-Thunder/FPV-Tracker */

enum ubloxState{ WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB };

ubloxState ubloxProcessDataState = WAIT_SYNC1;

unsigned short ubloxExpectedDataLength;
unsigned short ubloxClass, ubloxId;
unsigned char  ubloxCKA, ubloxCKB;

// process serial data
// data is stored inside #GNSSbuf, data size inside #GNSS_cnt
// warning : if #GNSSbuf is too short, data is truncated.
static int ubloxProcessData(unsigned char data) {
	int parsed = 0;

	switch (ubloxProcessDataState) {
	case WAIT_SYNC1:
		if (data == 0xb5) {
			ubloxProcessDataState = WAIT_SYNC2;
		}
		break;

	case WAIT_SYNC2:
		if (data == 0x62) {
			ubloxProcessDataState = GET_CLASS;
		}
		else if (data == 0xb5) {
			// ubloxProcessDataState = GET_SYNC2;
		}
		else {
			ubloxProcessDataState = WAIT_SYNC1;
		}
		break;
	case GET_CLASS:
		ubloxClass = data;
		ubloxCKA = data;
		ubloxCKB = data;
		ubloxProcessDataState = GET_ID;
		break;

	case GET_ID:
		ubloxId = data;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_LL;
		break;

	case GET_LL:
		ubloxExpectedDataLength = data;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_LH;
		break;

	case GET_LH:
		ubloxExpectedDataLength += data << 8;
		GNSS_cnt = 0;
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		ubloxProcessDataState = GET_DATA;
		break;

	case GET_DATA:
		ubloxCKA += data;
		ubloxCKB += ubloxCKA;
		if (GNSS_cnt < sizeof(GNSSbuf)) {
			GNSSbuf[GNSS_cnt++] = data;
		}
		if ((--ubloxExpectedDataLength) == 0) {
			ubloxProcessDataState = GET_CKA;
		}
		break;

	case GET_CKA:
		if (ubloxCKA != data) {
			ubloxProcessDataState = WAIT_SYNC1;
		}
		else {
			ubloxProcessDataState = GET_CKB;
		}
		break;

	case GET_CKB:
		if (ubloxCKB == data) {
			parsed = 1;
		}
		ubloxProcessDataState = WAIT_SYNC1;
		break;

	}

	return parsed;
}

/* ------ END -----------  https://github.com/Black-Thunder/FPV-Tracker */

static bool ublox_query(uint8_t requestedClass, uint8_t requestedID, uint16_t timelimit)
{
  unsigned long startTime = millis();

Serial.println("ublox_query()...");

  uint8_t msglen = makeUBXCFG(requestedClass, requestedID, 0, NULL);
  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }
  sendUBX(GNSSbuf, msglen);
  yield();

  int count = 1;

  // Get the message back from the GPS
  GNSS_DEBUG_PRINT(F(" * Reading response: "));

  uint32_t waittime = (millis() - startTime);

  while (waittime < (uint32_t) timelimit ) {

    if ((waittime & 0x3F) == 0)  yield();
    if ((waittime > 1000 && count==1) || (waittime > 2000 && count==2)) {
Serial.println("... re-sending UBX command");
      uint8_t msglen = makeUBXCFG(requestedClass, requestedID, 0, NULL); // MON-VER
      while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }
      sendUBX(GNSSbuf, msglen);
      ++count;
      yield();
    }

    if (waittime > timelimit-100) {
      Serial.println("... time out");
      break;
    }

    if (Serial_GNSS_In.available()) {

      unsigned char c = Serial_GNSS_In.read();
      GNSS_DEBUG_PRINT(c, HEX);

      if (ubloxProcessData(c)) {
        if (ubloxClass == requestedClass && ubloxId == requestedID)
            return true;
      }
    }

    waittime = (millis() - startTime);
  }

  return false;
}

static byte ublox_version()
{
    byte rval = GNSS_MODULE_NMEA;
    if (ublox_query(0x0A, 0x04, 3000) == false) // MON-VER
        return rval;

            // UBX-MON-VER data description
            // uBlox 6  - page 166 : https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
            // uBlox 7  - page 153 : https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
            // uBlox M8 - page 300 : https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf

    Serial.print(F("INFO: UBLOX GNSS module HW version: "));
    Serial.println((char *) &GNSSbuf[30]);

    Serial.print(F("INFO: UBLOX GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[0]);

#ifdef DO_GNSS_DEBUG
    for(unsigned i = 30 + 10; i < GNSS_cnt; i+=30) {
      Serial.print(F("INFO: GNSS module extension: "));
      Serial.println((char *) &GNSSbuf[i]);
    }
#endif

    if (GNSSbuf[33] == '4')
      rval = GNSS_MODULE_U6;
    else if (GNSSbuf[33] == '7')
      rval = GNSS_MODULE_U7;
    else if (GNSSbuf[33] == '8')
      rval = GNSS_MODULE_U8;
    else if (GNSSbuf[32] == '1' && GNSSbuf[33] == '9')
      rval = GNSS_MODULE_U9;
    else if (GNSSbuf[33] == 'A')
      rval = GNSS_MODULE_U10;

    return rval;
}

static gnss_id_t ublox_probe()
{
  /*
   * ESP8266 NodeMCU and ESP32 DevKit (with NodeMCU adapter)
   * have no any spare GPIO pin to provide GNSS Tx feedback
   */
  return(hw_info.model == SOFTRF_MODEL_STANDALONE && hw_info.revision == 0 ?
         GNSS_MODULE_NMEA : (gnss_id_t) ublox_version());
}

static bool ublox_setup()
{
#if 1
  // Set the navigation mode (Airborne, 1G)
  // Turning off some GPS NMEA sentences on the uBlox modules
  setup_UBX();
#else
  //Serial_GNSS_Out.write("$PUBX,41,1,0007,0003,9600,0*10\r\n");
  Serial_GNSS_Out.write("$PUBX,41,1,0007,0003,38400,0*20\r\n");

  GNSS_FLUSH();
  SoC->swSer_begin(38400);

  // Turning off some GPS NMEA strings on the uBlox modules
  Serial_GNSS_Out.write("$PUBX,40,GLL,0,0,0,0*5C\r\n"); delay(250);
  Serial_GNSS_Out.write("$PUBX,40,GSV,0,0,0,0*59\r\n"); delay(250);
  Serial_GNSS_Out.write("$PUBX,40,VTG,0,0,0,0*5E\r\n"); delay(250);
#if !defined(NMEA_TCP_SERVICE)
  /* <<< what does TCP have to do with it? */
  //Serial_GNSS_Out.write("$PUBX,40,GSA,0,0,0,0*4E\r\n"); delay(250);
#endif
#endif

  return true;
}

static void ublox_loop()
{
#if defined(USE_GNSS_PSM)
  if (settings->power_save & POWER_SAVE_GNSS) {
    if (hw_info.model == SOFTRF_MODEL_UNI) {

      if (!gnss_psm_active && isValidGNSSFix() && gnss.satellites.value() > 5) {
        // Setup for Power Save Mode (Default Cyclic 1s)
        for (int i = 0; i < sizeof(RXM_PSM); i++) {
          Serial_GNSS_Out.write(pgm_read_byte(&RXM_PSM[i]));
        }

        GNSS_DEBUG_PRINTLN(F("INFO: GNSS Power Save Mode"));
        gnss_psm_active = true;
      } else if (  gnss_psm_active &&
                 ((gnss.satellites.isValid() && gnss.satellites.value() <= 5) ||
                   gnss.satellites.age() > NMEA_EXP_TIME)) {
        // Setup for Continuous Mode
        for (int i = 0; i < sizeof(RXM_MAXP); i++) {
          Serial_GNSS_Out.write(pgm_read_byte(&RXM_MAXP[i]));
        }

        GNSS_DEBUG_PRINTLN(F("INFO: GNSS Continuous Mode"));
        gnss_psm_active = false;
      }
    }
  }
#endif /* USE_GNSS_PSM */
}

static void ublox_fini()
{
  // Controlled Software reset
  for (int i = 0; i < sizeof(CFG_RST); i++) {
    Serial_GNSS_Out.write(pgm_read_byte(&CFG_RST[i]));
  }

  delay(hw_info.gnss == GNSS_MODULE_U8 ? 1000 : 600);

  // power off until wakeup call
  for (int i = 0; i < sizeof(RXM_PMREQ_OFF); i++) {
    Serial_GNSS_Out.write(pgm_read_byte(&RXM_PMREQ_OFF[i]));
  }
}

const gnss_chip_ops_t ublox_ops = {
  ublox_probe,
  ublox_setup,
  ublox_loop,
  ublox_fini,
  138 /* GGA */, 67 /* RMC */
};


#if 0
// adapted code from github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/GPS/UBlox_Recovery
int getAck(uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t    ubxFrameCounter = 0;
    bool        ubxFrame = 0;
    uint32_t    startTime = millis();
    uint16_t    needRead;

    while (millis() - startTime < 800) {
        while (Serial_GNSS_In.available()) {
            int c = Serial_GNSS_In.read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |=  (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= sizeof(GNSSbuf)) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (Serial_GNSS_In.readBytes(GNSSbuf, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
        yield();
    }
    return 0;
}
#endif

static bool ublox_factory_reset()
{
  // reset GPS to factory settings
  //for (int i = 0; i < sizeof(factoryUBX); i++) {
  //  Serial_GNSS_Out.write(pgm_read_byte(&factoryUBX[i]));
  //}

  // adapted code from:
  //   github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/tree/master/examples/GPS/UBlox_Recovery
  //Serial_GNSS_In.write(cfg_clear2, sizeof(cfg_clear2));   // clear flash config
  int msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_clear2), cfg_clear2);
  sendUBX(GNSSbuf, msglen);
  //if (getAck(0x05, 0x01)) {
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("cleared flash config");
  } else return false;
//  Serial_GNSS_In.write(cfg_clear4, sizeof(cfg_clear4));   // clear EEPROM config
  msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_clear4), cfg_clear4);
  sendUBX(GNSSbuf, msglen);
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("cleared EEPROM config");
  } else return false;
//  Serial_GNSS_In.write(cfg_clear1, sizeof(cfg_clear1));   // clear BBR config
  msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_clear1), cfg_clear1);
  sendUBX(GNSSbuf, msglen);
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("cleared BBR config");
  } else return false;
//  Serial_GNSS_In.write(cfg_load3, sizeof(cfg_load3));    // load config from all 3 devices (but not SPI)
  msglen = makeUBXCFG(0x06, 0x09, sizeof(cfg_load3), cfg_load3);
  sendUBX(GNSSbuf, msglen);
  if (getUBX_ACK(0x06, 0x09)) {
      Serial.println("UBX config reloaded");
  } else return false;

  delay(600);

  // Cold Start (Forced Watchdog)
  Serial.println("Cold-starting GNSS module...");
  //for (int i = 0; i < sizeof(CFG_RST_COLD); i++) {
  //  Serial_GNSS_Out.write(pgm_read_byte(&CFG_RST_COLD[i]));
  //}
//  Serial_GNSS_In.write(CFG_RST_COLD, sizeof(CFG_RST_COLD));
  msglen = makeUBXCFG(0x06, 0x04, sizeof(CFG_RST_COLD), CFG_RST_COLD);
  sendUBX(GNSSbuf, msglen);

  delay(1000);

  return true;
}

static void reset_gnss()
{
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
        hw_info.model == SOFTRF_MODEL_PRIME_MK3) {
#if !defined(EXCLUDE_GNSS_UBLOX)
      if (hw_info.gnss == GNSS_MODULE_U6 ||
          hw_info.gnss == GNSS_MODULE_U7 ||
          hw_info.gnss == GNSS_MODULE_U8) {

            Serial.println("Ublox factory reset...");
            if (ublox_factory_reset())
                Serial.println("... done");
            else
                Serial.println("... failed");

      }
#endif
#if !defined(EXCLUDE_GNSS_AT65)
      if (hw_info.gnss == GNSS_MODULE_AT65) {
          if (Serial_GNSS_Out.availableForWrite() < 16)
              delay(600);
          if (Serial_GNSS_Out.availableForWrite() < 16) {
              Serial.println("Serial_GNSS_Out.write() blocked?");
              return;
          }
          Serial.println("AT65 factory defaults...");
          Serial_GNSS_Out.write("$PCAS10,3*1F\r\n");   // factory defaults
          delay(600);
          if (Serial_GNSS_Out.availableForWrite() < 16)
              delay(600);
          if (Serial_GNSS_Out.availableForWrite() < 16) {
              Serial.println("Serial_GNSS_Out.write() blocked?");
              return;
          }
          Serial_GNSS_Out.write("\0");
          delay(300);
          Serial.println("AT65 cold start...");
          Serial_GNSS_Out.write("$PCAS10,2*1E\r\n");   // cold start
          delay(600);
          Serial.println("... done");
      }
#endif
    }
}
#endif /* EXCLUDE_GNSS_UBLOX */

#if !defined(EXCLUDE_GNSS_SONY)
static gnss_id_t sony_probe()
{
  /* Wake-up */
  Serial_GNSS_Out.write("@WUP\r\n");       delay(500);

  /* Firmware version request */
  return nmea_handshake("@VER\r\n", "[VER] Done", true) ?
                        GNSS_MODULE_SONY : GNSS_MODULE_NONE;
}

static bool sony_setup()
{
  /* Idle */
  Serial_GNSS_Out.write("@GSTP\r\n");
  GNSS_FLUSH();
  delay(2000);

#if !defined(EXCLUDE_LOG_GNSS_VERSION)
  while (Serial_GNSS_In.available() > 0) { Serial_GNSS_In.read(); }

  Serial_GNSS_Out.write("@VER\r\n");

  int i=0;
  char c;
  unsigned long start_time = millis();

  /* take response into buffer */
  while ((millis() - start_time) < 2000) {

    c = Serial_GNSS_In.read();

    if (isPrintable(c) || c == '\r' || c == '\n') {
      if (i >= sizeof(GNSSbuf) - 1) break;
      GNSSbuf[i++] = c;
    } else {
      /* ignore */
      continue;
    }

    if (c == '\n') break;
  }

  GNSSbuf[i] = 0;

  if (strlen((char *) &GNSSbuf[0])) {
    Serial.print(F("INFO: Sony GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[0]);
  }

  delay(250);
#endif

  /* GGA + GSA + RMC */
  Serial_GNSS_Out.write("@BSSL 0x25\r\n"); delay(250);
  /* GPS + GLONASS. This command must be issued at Idle state */
  Serial_GNSS_Out.write("@GNS 3\r\n");     delay(250);
  /*  Positioning algorithm. This command must be issued at Idle state */
  Serial_GNSS_Out.write("@GUSE 0\r\n");    delay(250);

 if (get_pps_pin() != SOC_UNUSED_PIN) {
   /* Enable 1PPS output */
   Serial_GNSS_Out.write("@GPPS 1\r\n");    delay(250);
 }

#if defined(USE_GNSS_PSM)
  if (settings->power_save & POWER_SAVE_GNSS) {
    /*
     * Low power, 1 second interval, no sleep
     *
     * WARNING: use of this mode may cause issues
     */
    Serial_GNSS_Out.write("@GSOP 2 1000 0\r\n"); delay(250);
  }
#endif /* USE_GNSS_PSM */

  /*
   * Hot start for TTFF
   * When the conditions for the hot start have not been met,
   * positioning is started automatically using a warm start or cold start.
   */
  Serial_GNSS_Out.write("@GSR\r\n");

  GNSS_FLUSH(); delay(100);

  return true;
}

static void sony_loop()
{

}

static void sony_fini()
{
   /* Idle */
  Serial_GNSS_Out.write("@GSTP\r\n");
  GNSS_FLUSH(); delay(1500);

  /* Sony GNSS sleep level (0-2)
   * This command must be issued at Idle state.
   * When this command is issued at Exec state, error is returned.
   */

//  Serial_GNSS_Out.write("@SLP 0\r\n");
  Serial_GNSS_Out.write("@SLP 1\r\n");
//  Serial_GNSS_Out.write("@SLP 2\r\n");

  GNSS_FLUSH(); delay(100);
}

const gnss_chip_ops_t sony_ops = {
  sony_probe,
  sony_setup,
  sony_loop,
  sony_fini,
  24 /* GGA */, 38 /* RMC */
};
#endif /* EXCLUDE_GNSS_SONY */

#if !defined(EXCLUDE_GNSS_MTK)
static gnss_id_t mtk_probe()
{
  /* Firmware version request */
  return nmea_handshake("$PMTK605*31\r\n", "$PMTK705", false) ?
                        GNSS_MODULE_MT33 : GNSS_MODULE_NMEA;
}

static bool mtk_setup()
{
#if !defined(EXCLUDE_LOG_GNSS_VERSION)
  Serial_GNSS_Out.write("$PMTK605*31\r\n");

  int i=0;
  char c;
  unsigned long start_time = millis();

  /* take response into buffer */
  while ((millis() - start_time) < 2000) {

    c = Serial_GNSS_In.read();

    if (isPrintable(c) || c == '\r' || c == '\n') {
      if (i >= sizeof(GNSSbuf) - 1) break;
      GNSSbuf[i++] = c;
    } else {
      /* ignore */
      continue;
    }

    if (c == '\n') break;
  }

  GNSSbuf[i] = 0;

  size_t len = strlen((char *) &GNSSbuf[0]);

  if (len > 19) {
    for (int i=9; i < len; i++) {
      if (GNSSbuf[i] == ',') {
        GNSSbuf[i] = 0;
      }
    }
    Serial.print(F("INFO: MTK GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[9]);
  }

  delay(250);
#endif

  /* RMC + GGA + GSA */
  Serial_GNSS_Out.write("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
  GNSS_FLUSH(); delay(250);

  /* Aviation mode */
  Serial_GNSS_Out.write("$PMTK886,2*2A\r\n");
  GNSS_FLUSH(); delay(250);

  return true;
}

static void mtk_loop()
{

}

static void mtk_fini()
{
  /* Stop mode */
  Serial_GNSS_Out.write("$PMTK161,0*28\r\n");
  GNSS_FLUSH(); delay(250);
}

const gnss_chip_ops_t mtk_ops = {
  mtk_probe,
  mtk_setup,
  mtk_loop,
  mtk_fini,
  48 /* GGA */, 175 /* RMC */
};
#endif /* EXCLUDE_GNSS_MTK */

#if !defined(EXCLUDE_GNSS_GOKE)
static gnss_id_t goke_probe()
{
  /* Firmware version request */
  return nmea_handshake("$PGKC462*2F\r\n", "$PGKC463", false) ?
                        GNSS_MODULE_GOKE : GNSS_MODULE_NMEA;
}

static void goke_sendcmd(const char *cmd)
{
  while (Serial_GNSS_In.available() > 0) { while (Serial_GNSS_In.read() != '\n') {yield();} }
  Serial_GNSS_In.write(cmd);
  GNSS_FLUSH();
  delay(250);
}

static bool goke_setup()
{
  /* There are reports that Air530 does not actually work with GALILEO yet */
  if (settings->band == RF_BAND_CN) {
    /* GPS + BEIDOU */
    goke_sendcmd("$PGKC115,1,0,1,0*2A\r\n");
  } else {
    /* GPS + GLONASS */
    goke_sendcmd("$PGKC115,1,1,0,0*2A\r\n");
  }

#if 0
  /* SBAS */
  goke_sendcmd("$PGKC239,1*3A\r\n");
#endif

#if defined(NMEA_TCP_SERVICE)
  /* RMC + GGA + GSA */
  goke_sendcmd("$PGKC242,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*36\r\n");
#else
  /* RMC + GGA */
//goke_sendcmd("$PGKC242,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*37\r\n");
  /* <<< why not GSA? what does TCP have to do with it? */
  goke_sendcmd("$PGKC242,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*36\r\n");
#endif

  if (get_pps_pin() != SOC_UNUSED_PIN) {
    /* Enable 3D fix 1PPS output */
    goke_sendcmd("$PGKC161,2,200,1000*04\r\n");
  }

  return true;
}

static void goke_loop()
{

}

static void goke_fini()
{
  goke_sendcmd("$PGKC051,0*37\r\n");
  // goke_sendcmd("$PGKC051,1*36\r\n");
  // goke_sendcmd("$PGKC105,4*33\r\n");
}

const gnss_chip_ops_t goke_ops = {
  goke_probe,
  goke_setup,
  goke_loop,
  goke_fini,
  185 /* GGA */, 265 /* RMC */
};
#endif /* EXCLUDE_GNSS_GOKE */

#if !defined(EXCLUDE_GNSS_AT65)
static gnss_id_t at65_probe()
{
  /* Firmware version request */
  return nmea_handshake("$PCAS06,0*1B\r\n", "$GPTXT,01,01,02", false) ?
                        GNSS_MODULE_AT65 : GNSS_MODULE_NMEA;
}

static bool at65_setup()
{
#if !defined(EXCLUDE_LOG_GNSS_VERSION)
  Serial_GNSS_Out.write("$PCAS06,0*1B\r\n");

  int i=0;
  char c;
  unsigned long start_time = millis();

  /* take response into buffer */
  while ((millis() - start_time) < 2000) {

    c = Serial_GNSS_In.read();

    if (isPrintable(c) || c == '\r' || c == '\n') {
      if (i >= sizeof(GNSSbuf) - 1) break;
      GNSSbuf[i++] = c;
    } else {
      /* ignore */
      continue;
    }

    if (c == '\n') break;
  }

  GNSSbuf[i] = 0;

  size_t len = strlen((char *) &GNSSbuf[0]);

  if (len > 19) {
    for (int i=19; i < len; i++) {
      if (GNSSbuf[i] == '*') {
        GNSSbuf[i] = 0;
      }
    }
    Serial.print(F("INFO: AT65 GNSS module FW version: "));
    Serial.println((char *) &GNSSbuf[19]);
  }

  delay(250);
#endif

  /* Assume that we deal with fake NEO module (AT6558 based) */
  if (hw_info.model == SOFTRF_MODEL_BADGE)
      Serial_GNSS_Out.write("$PCAS04,7*1E\r\n"); /* GPS + GLONASS + BEIDOU */
  else
      Serial_GNSS_Out.write("$PCAS04,5*1C\r\n"); /* GPS + GLONASS */     
    //Serial_GNSS_Out.write("$PCAS04,7*1E\r\n"); /* GPS + GLONASS + BEIDOU */
  delay(250);
#if defined(NMEA_TCP_SERVICE)
  /* GGA,RMC and GSA */
  Serial_GNSS_Out.write("$PCAS03,1,0,1,0,1,0,0,0,0,0,,,0,0*03\r\n"); delay(250);
#else
  /* GGA and RMC */
//Serial_GNSS_Out.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n"); delay(250);
  /* <<< why not GSA? what does TCP have to do with it? */
  Serial_GNSS_Out.write("$PCAS03,1,0,1,0,1,0,0,0,0,0,,,0,0*03\r\n"); delay(250);
#endif
  Serial_GNSS_Out.write("$PCAS11,6*1B\r\n"); /* Aviation < 2g */     delay(250);

  return true;
}

static void at65_loop()
{

}

static void at65_fini()
{

}

const gnss_chip_ops_t at65_ops = {
  at65_probe,
  at65_setup,
  at65_loop,
  at65_fini,
  70 /* GGA */, 135 /* RMC */
};
#endif /* EXCLUDE_GNSS_AT65 */

static bool GNSS_fix_cache = false;

bool leap_seconds_valid()
{
    static uint8_t leap_valid = 2;  // means not known whether valid
    // could start = 0 but risk 13 minutes of no-transmit if no response
    // - usually gets set to 0 in first check below (if no leap seconds)
    //    - before even the first fix, so no bad transmissions happen

    // this is only called if GNSS_fix_cache == true
    switch (hw_info.gnss) {
#if !defined(EXCLUDE_GNSS_UBLOX)
        case GNSS_MODULE_U6:
        case GNSS_MODULE_U7:
        case GNSS_MODULE_U8:
        case GNSS_MODULE_U9:
        case GNSS_MODULE_U10:
            static uint32_t next_check = 0;
            static uint8_t checks_count = 0;
            static uint8_t max_checks = 18;   // 13 minutes
            if (leap_valid!=1 && checks_count<max_checks && (checks_count==0 || millis()>next_check)) {
                if (ublox_query(0x01, 0x20, 2000) == true) {    // NAV-TIMEGPS
                    if ((GNSSbuf[11] & 0x04) == 0) {
                        leap_valid = 0;    // known invalid
                        Serial.println("UBX says leap seconds not known yet");
                    } else {
                        leap_valid = 1;    // known valid, no need to ask again
                        Serial.println("UBX says leap seconds known");
                    }
                } else {
                    // (query failed) - no change in leap_valid
                    Serial.println("No response to UBX leap seconds query");
                }
                next_check = millis() + 43000;
                ++checks_count;
                if (checks_count >= 18) {
                    leap_valid = 2;     // in case it never works - resume transmissions
                    max_checks = 36;    // keep trying for up to 13 more minutes
                }
            }
            break;
#endif
        case GNSS_MODULE_AT65:
        default:
            return true;    // on other models assume valid
            break;
    }
    return (leap_valid != 0);   // if not known invalid, assume valid
}

bool isValidGNSSFix()
{
  if (settings->debug_flags & DEBUG_FAKEFIX)
      return true;   // for testing
  if (! GNSS_fix_cache)
      return false;
  return true;
  //return leap_seconds_valid();
}

byte GNSS_setup() {

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 /* && hw_info.revision >= 8 */)
      is_prime_mk2 = true;

  gnss_id_t gnss_id = GNSS_MODULE_NONE;

  SoC->swSer_begin(SERIAL_IN_BR);
  delay(500);  // added to make sure swSer is ready

  if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
      hw_info.model == SOFTRF_MODEL_PRIME_MK3 ||
      hw_info.model == SOFTRF_MODEL_UNI       ||
      hw_info.model == SOFTRF_MODEL_BADGE     ||
      hw_info.model == SOFTRF_MODEL_LEGO)
  {
    // power on by wakeup call
    Serial_GNSS_Out.write((uint8_t) 0); GNSS_FLUSH(); delay(1000);
  }

#if !defined(EXCLUDE_GNSS_SONY)
  gnss_id = (gnss_id == GNSS_MODULE_NONE ?
              (gnss_chip = &sony_ops,         gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_SONY */

  gnss_id = (gnss_id == GNSS_MODULE_NONE ?
              (gnss_chip = &generic_nmea_ops, gnss_chip->probe()) : gnss_id);

  gnss_id = (gnss_id == GNSS_MODULE_NONE ?   // no NMEA sentences seen - try again
              (gnss_chip = &generic_nmea_ops, gnss_chip->probe()) : gnss_id);

  gnss_id = (gnss_id == GNSS_MODULE_NONE ?   // still no NMEA sentences seen - try again
              (gnss_chip = &generic_nmea_ops, gnss_chip->probe()) : gnss_id);

  if (gnss_id == GNSS_MODULE_NONE) {     // no NMEA sentences seen in 3 trials

#if !defined(EXCLUDE_GNSS_UBLOX) && defined(ENABLE_UBLOX_RFS)
    if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 ||
        hw_info.model == SOFTRF_MODEL_PRIME_MK3) {

      byte version = ublox_version();

      if (version == GNSS_MODULE_U6 ||
          version == GNSS_MODULE_U7 ||
          version == GNSS_MODULE_U8) {

        Serial.println(F("WARNING: Misconfigured UBLOX GNSS detected!"));
        Serial.print(F("Reset to factory default state: "));

        (void) ublox_factory_reset();

        gnss_id = generic_nmea_ops.probe();

        if (gnss_id == GNSS_MODULE_NONE) {
          Serial.println(F("FAILURE"));
          return (byte) gnss_id;
        }
        Serial.println(F("SUCCESS"));    // and fall through to probe() call below

      } else {
        Serial.println(F("WARNING: no NMEA and Ublox GNSS not detected!"));
        return (byte) gnss_id;    // GNSS_MODULE_NONE
      }

    } else   // not SOFTRF_MODEL_PRIME_MK*
#endif /* EXCLUDE_GNSS_UBLOX && ENABLE_UBLOX_RFS */

        return (byte) gnss_id;    // GNSS_MODULE_NONE
  }

#if !defined(EXCLUDE_GNSS_UBLOX)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &ublox_ops,  gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_UBLOX */
#if !defined(EXCLUDE_GNSS_MTK)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &mtk_ops,    gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_MTK */
#if !defined(EXCLUDE_GNSS_GOKE)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &goke_ops,   gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_GOKE */
#if !defined(EXCLUDE_GNSS_AT65)
  gnss_id = (gnss_id == GNSS_MODULE_NMEA ?
              (gnss_chip = &at65_ops,   gnss_chip->probe()) : gnss_id);
#endif /* EXCLUDE_GNSS_AT65 */

  Serial.print("GNSS type found: ");
  Serial.println(GNSS_name[gnss_id]);

  gnss_chip = (gnss_id == GNSS_MODULE_NMEA ? &generic_nmea_ops : gnss_chip);

  if (gnss_chip) gnss_chip->setup();

  uint8_t pps_pin = get_pps_pin();
  if (pps_pin != SOC_UNUSED_PIN
#if defined(ESP32)
   && ESP32_pin_reserved(pps_pin, false, "GNSS PPS") == false
#endif
   ) {
      pinMode(pps_pin, INPUT_PULLDOWN);
#if !defined(NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(pps_pin), SoC->GNSS_PPS_handler, RISING);
#else
      int interrupt_num = digitalPinToInterrupt(pps_pin);
      if (interrupt_num != NOT_AN_INTERRUPT)
        attachInterrupt(interrupt_num, SoC->GNSS_PPS_handler, RISING);
#endif
  } else {
      settings->ppswire = false;
  }

#if defined(USE_NMEALIB)
//Serial.println("GNSS_setup: using NMEALIB");
#else
//Serial.println("GNSS_setup: not using NMEALIB");
#endif

//#if defined(USE_NMEA_CFG)
  NMEA_Source = DEST_NONE;
//#endif /* USE_NMEA_CFG */

  return (byte) gnss_id;
}

void GNSS_loop()
{
#if defined(ESP32)
  if (gnss_needs_reset) {
      gnss_needs_reset = false;
      reset_gnss();
      delay(2000);
      Serial.println("Rebooting...");
      delay(2000);
      reboot();
  }
#endif

  PickGNSSFix();

  /*
   * Both GGA and RMC NMEA sentences are required.
   * No fix when any of them is missing or lost.
   * Valid date is critical for legacy protocol (only).
   */
  GNSS_fix_cache = gnss.location.isValid()               &&
                   gnss.altitude.isValid()               &&
                   gnss.date.isValid()                   &&
                  (gnss.location.age() <= NMEA_EXP_TIME) &&
                  (gnss.altitude.age() <= NMEA_EXP_TIME) &&
                  (gnss.date.age()     <= NMEA_EXP_TIME);

  GNSSTimeSync();

  if (gnss_chip) gnss_chip->loop();
}

void GNSS_fini()
{
  uint8_t pps_pin = get_pps_pin();
  if (pps_pin != SOC_UNUSED_PIN) {
#if !defined(NOT_AN_INTERRUPT)
    detachInterrupt(digitalPinToInterrupt(pps_pin));
#else
    int interrupt_num = digitalPinToInterrupt(pps_pin);
    if (interrupt_num != NOT_AN_INTERRUPT) {
      detachInterrupt(interrupt_num);
    }
#endif

  }

  if (gnss_chip) gnss_chip->fini();
}

/*
 * Sync with GNSS time every 60 seconds
 */
void GNSSTimeSync()
{
  if ((GNSSTimeSyncMarker == 0 || (millis() - GNSSTimeSyncMarker > 60000)) &&
       gnss.time.isValid()                                                 &&
       gnss.time.isUpdated()                                               &&
       gnss.date.year() >= FW_Build_Year                                   &&
      (gnss.time.age() <= 1000) /* 1s */ ) {
#if 0
    Serial.print("Valid: ");
    Serial.println(gnss.time.isValid());
    Serial.print("isUpdated: ");
    Serial.println(gnss.time.isUpdated());
    Serial.print("age: ");
    Serial.println(gnss.time.age());
#endif
    setTime(gnss.time.hour(),
            gnss.time.minute(),
            gnss.time.second(),
            gnss.date.day(),
            gnss.date.month(),
            gnss.date.year());
    GNSSTimeSyncMarker = millis();
  }
}

bool Try_GNSS_sentence() {
    int ndx;
    bool isValidSentence = gnss.encode(GNSSbuf[GNSS_cnt]);
    if (GNSSbuf[GNSS_cnt] == '\r' && isValidSentence) {
      NMEA_Source = DEST_NONE;
      if (settings->nmea_g || settings->nmea2_g) {
        for (ndx = GNSS_cnt - 4; ndx >= 0; ndx--) { // jump over CS and *
          if ((GNSSbuf[ndx] == '$') && (GNSSbuf[ndx+1] == 'G')) {
            size_t write_size = GNSS_cnt - ndx + 1;   // includes * and CS
          char *gb = (char *) &GNSSbuf[ndx];
#if 0
          if (!strncmp(gb+3, "GGA,", strlen("GGA,"))) {
            GGA_Stop_Time_Marker = millis();

            Serial.print("GGA Start: ");
            Serial.print(GGA_Start_Time_Marker);
            Serial.print(" Stop: ");
            Serial.print(GGA_Stop_Time_Marker);
            Serial.print(" gnss.time.age: ");
            Serial.println(gnss.time.age());

          }
#endif
            /*
             * Work around issue with "always 0.0,M" GGA geoid separation value
             * given by some Chinese GNSS chipsets
             */
            bool is_gga = (write_size>7 && !strncmp(gb+3, "GGA,", 4));
#if defined(USE_NMEALIB)
            if (hw_info.model == SOFTRF_MODEL_PRIME_MK2 && is_gga
                  && gnss.separation.meters() == 0.0) {
              NMEA_GGA();
              // GGA is output either directly (below) or indirectly (via NMEA_GGA()) but not both.
              // Observed on a T-Beam: NMEA_GGA() while no fix, then direct.
            }
            else
#endif
            {
              if (is_gga) {
                strncpy(GPGGA_Copy, gb, write_size);  // for traffic alarm logging
                GPGGA_Copy[write_size] = '\0';
              }
              NMEA_Outs(settings->nmea_g, settings->nmea2_g, gb, write_size, true);
            }
            return true;
          }
        }
      }
    }
    return false;
}

void PickGNSSFix()
{
  int c = -1;

  if (is_prime_mk2) {

    // only use the internal (or add-on serial) GNSS, leave other ports alone for data bridging
    while (true) {
      if (Serial_GNSS_In.available() > 0) {
       c = Serial_GNSS_In.read();
      } else {
        /* return back if no input data */
        return;
      }
      if (c == -1) {
        /* retry */
        continue;
      }
      if (isPrintable(c) || c == '\r' || c == '\n') {
        GNSSbuf[GNSS_cnt] = c;
      } else {
        /* ignore */
        continue;
      }
      (void) Try_GNSS_sentence();
      if (GNSSbuf[GNSS_cnt] == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
        GNSS_cnt = 0;
      } else {
        GNSS_cnt++;
        yield();
      }
    }        // infinite loop unless !Serial_GNSS_In.available() above

    return;
  }

  // other models:

  /*
   * Check SW/HW UARTs, USB and BT for data
   * WARNING! Make use only one input source at a time.
   */
   // - note there is nothing here to stop interleaving sentences from different sources!
  while (true) {
#if !defined(USE_NMEA_CFG)
    if (Serial_GNSS_In.available() > 0) {
      c = Serial_GNSS_In.read();
    } else if (Serial.available() > 0) {
      c = Serial.read();
    } else if (SoC->Bluetooth_ops && SoC->Bluetooth_ops->available() > 0) {
      c = SoC->Bluetooth_ops->read();
    } else {
      /* return back if no input data */
      break;
    }
      /*
       * Don't forget to disable echo:
       *
       * stty raw -echo -F /dev/rfcomm0
       *
       * GNSS input becomes garbled otherwise
       */

      // Serial.write((char) c);
      /* Ignore Bluetooth input for a while */
      // break;
#else
    /*
     * Give priority to control channels over default GNSS input source on
     * 'Dongle', 'Retro', 'Uni', 'Mini', 'Badge', 'Academy' and 'Lego' Editions
     */

    /* Bluetooth input is first */
    if (SoC->Bluetooth_ops && SoC->Bluetooth_ops->available() > 0) {
      c = SoC->Bluetooth_ops->read();

      NMEA_Source = DEST_BLUETOOTH;

    /* USB input is second */
    } else if (SoC->USB_ops && SoC->USB_ops->available() > 0) {
      c = SoC->USB_ops->read();

      NMEA_Source = DEST_USB;

#if defined(ARDUINO_NUCLEO_L073RZ)
      /* This makes possible to configure S76x's built-in SONY GNSS from aside */
      if (hw_info.model == SOFTRF_MODEL_DONGLE) {
        Serial_GNSS_Out.write(c);
      }
#endif

    /* Serial input is third */
    } else if (SerialOutput.available() > 0) {
      c = SerialOutput.read();

      NMEA_Source = DEST_UART;

#if 0
      /* This makes possible to configure HTCC-AB02S built-in GOKE GNSS from aside */
      if (hw_info.model == SOFTRF_MODEL_MINI) {
        Serial_GNSS_Out.write(c);
      }
#endif

    /* Built-in GNSS input */
    } else if (Serial_GNSS_In.available() > 0) {
      c = Serial_GNSS_In.read();
      NMEA_Source = DEST_NONE;
    } else {
      /* return back if no input data */
      break;
    }
#endif /* USE_NMEA_CFG */

    if (c == -1) {
      /* retry */
      continue;
    }

    if (isPrintable(c) || c == '\r' || c == '\n') {
      GNSSbuf[GNSS_cnt] = c;
    } else {
      /* ignore */
      continue;
    }

#if defined(ENABLE_GNSS_STATS)
    if ( (GNSS_cnt >= 5) &&
         (GNSSbuf[GNSS_cnt-5] == '$') &&
         (GNSSbuf[GNSS_cnt-4] == 'G') &&
        ((GNSSbuf[GNSS_cnt-3] == 'P') || (GNSSbuf[GNSS_cnt-3] == 'N'))) {
      if ( (GNSSbuf[GNSS_cnt-2] == 'G') &&
           (GNSSbuf[GNSS_cnt-1] == 'G') &&
           (GNSSbuf[GNSS_cnt-0] == 'A')) {
        gnss_stats.gga_time_ms = millis();
        gnss_stats.gga_count++;
      } else if ( (GNSSbuf[GNSS_cnt-2] == 'R') &&
                  (GNSSbuf[GNSS_cnt-1] == 'M') &&
                  (GNSSbuf[GNSS_cnt-0] == 'C')) {
        gnss_stats.rmc_time_ms = millis();
        gnss_stats.rmc_count++;
      }
    }
#endif /* ENABLE_GNSS_STATS */

    if (Try_GNSS_sentence()) {
#if defined(USE_NMEA_CFG)
      NMEA_Process_SRF_SKV_Sentences();
#endif /* USE_NMEA_CFG */
    }

#if defined(ENABLE_D1090_INPUT)
    if (GNSSbuf[GNSS_cnt]   == '\n' &&
        GNSS_cnt             >  1   &&
        GNSSbuf[GNSS_cnt-1] == '\r' &&
        GNSSbuf[GNSS_cnt-2] == ';')
    {
      int i=0;

      if (GNSS_cnt > 16 && GNSSbuf[GNSS_cnt-17] == '*') {
        for (i=0; i<14; i++) {
          if (!isxdigit(GNSSbuf[GNSS_cnt-16+i])) break;
        }
        if (i>=14) {
          D1090_Import(&GNSSbuf[GNSS_cnt-17]);
          GNSS_cnt -= 18;
        }
      } else if (GNSS_cnt > 30 && GNSSbuf[GNSS_cnt-31] == '*') {
        for (i=0; i<28; i++) {
          if (!isxdigit(GNSSbuf[GNSS_cnt-30+i])) break;
        }
        if (i>=28) {
          D1090_Import(&GNSSbuf[GNSS_cnt-31]);
          GNSS_cnt -= 32;
        }
      }
    }
#endif /* ENABLE_D1090_INPUT */

    if (GNSSbuf[GNSS_cnt] == '\n' || GNSS_cnt == sizeof(GNSSbuf)-1) {
      GNSS_cnt = 0;
    } else {
      GNSS_cnt++;
      yield();
    }
  }    // end of while()

}

#if !defined(EXCLUDE_EGM96)
/*
 *  Algorithm of EGM96 geoid offset approximation was taken from XCSoar
 */

static float AsBearing(float angle)
{
  float retval = angle;

  while (retval < 0)
    retval += 360.0;

  while (retval >= 360.0)
    retval -= 360.0;

  return retval;
}

int LookupSeparation(float lat, float lon)
{
  int ilat, ilon;

  ilat = round((90.0 - lat) / 2.0);
  ilon = round(AsBearing(lon) / 2.0);

  int offset = ilat * 180 + ilon;

  if (offset >= egm96s_dem_len)
    return 0;

  if (offset < 0)
    return 0;

  return (int) pgm_read_byte(&egm96s_dem[offset]) - 127;
}
#endif /* EXCLUDE_EGM96 */
