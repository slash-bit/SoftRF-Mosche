/*
 * IGC.cpp
 * Copyright (C) 2024 Moshe Braner
 *
 * Security-record code here borrowed from LK8000.
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

#include <SD.h>

#include "MD5.h"

// include this first, to get USE_SD_CARD
#include "../../system/SoC.h"

#if defined(USE_SD_CARD)

#include "../../driver/EEPROM.h"
#include "../../driver/GNSS.h"
#include "../../driver/Baro.h"
#include "NMEA.h"
#include "IGC.h"

#define INI_FILE              "/logs/IGC_CONF.TXT"

char FlightLogPath[28] = {'\0'};
File FlightLog;
bool FlightLogOpen = false;   // means log is in process, but the *file* may be closed
bool FlightLogFail = false;
uint32_t FlightLogClosed = 0;  // when a file was finalized, don't open a new one for a minute
size_t FlightLogPosition = 0;

static char brecord_template[] = "B1751494352910N07215306WA%05d%05d\r\n";
#define EXAMPLE_B_RECORD "B1751494352910N07215306WA0012300123\r\n"
//#define B_RECORD_SIZE 64    // chars in each pre-stored B-record
#define B_RECORD_SIZE sizeof(EXAMPLE_B_RECORD)
   // the null char at end is included in sizeof()
#define PRE_POS_NUM   8    // number of pre-stored B-records
//#define G_RECORD_SIZE (8 * (1+16+2))   // G+16+\r\n
#define DATA_BLOCK_SIZE 3000
static char *data_block_buf = NULL;    // will be malloc()ed into PSRAM
static int data_block_used = 0;
static char *pre_positions_buf = NULL;
static int num_pre_positions = 0;
static int pre_positions_head = 0;
static int pre_positions_next = 0;

// default config
static const char* CONFIG_DEFAULT_PILOT = "Chuck Yeager";
static const char* CONFIG_DEFAULT_TYPE  = "ASW20";
static const char* CONFIG_DEFAULT_REG   = "N12345";
static const char* CONFIG_DEFAULT_CS    = "XXX";
static const bool  CONFIG_DEFAULT_LIFTOFF_DETECT_ENABLE = true;
static const int   CONFIG_DEFAULT_LOG_INTERVAL = FLIGHT_LOG_INTERVAL;

static MD5_CTX md5_buf[8];   // or could malloc() it in PSRAM?
static MD5_CTX *md5_a, *md5_b, *md5_c, *md5_d;
static MD5_CTX *md5_a_copy, *md5_b_copy, *md5_c_copy, *md5_d_copy;

void FlightLog_setup()
{
    if (FlightLogFail)
        return;
    data_block_buf = (char *) malloc(DATA_BLOCK_SIZE + B_RECORD_SIZE*PRE_POS_NUM);
    // the combined block is intended to be stored in PSRAM if possible
    if (! data_block_buf) {
        FlightLogFail = true;
        return;
    }
    pre_positions_buf = data_block_buf + DATA_BLOCK_SIZE;
    //MD5_CTX *p = (MD5_CTX *) malloc(8*sizeof(MD5_CTX));
    //if (! data_block_buf) {
    //    free(data_block_buf);
    //    FlightLogFail = true;
    //    return;
    //}
    MD5_CTX *p = md5_buf;
    md5_a = p++;
    md5_b = p++;
    md5_c = p++;
    md5_d = p++;
    md5_a_copy = p++;
    md5_b_copy = p++;
    md5_c_copy = p++;
    md5_d_copy = p;
}

char clean_igc_char(char c)
{
    if (c >= 0x2D && c <= 0x5B)
      return c;        // for efficiency
    if (c >= 0x20 && c <= 0x7E && c != 0x24 &&
        c != 0x2A && c != 0x2C && c != 0x21 &&
        c != 0x5C && c != 0x5E && c != 0x7E) {
      return c;
    }
    return ' ';
}

void failFlightLog()
{
    FlightLog.close();
    FlightLogOpen = false;
    FlightLogFail = true;
}

void write_g_record(MD5_CTX *md5)
{
    if (FlightLogFail)
        return;
    //if (! FlightLogOpen)
    //    return;
    MD5::MD5Final(md5);
    char *digest = MD5::make_digest(md5);
//Serial.print("md5_digest=");
//Serial.println(md5->digest);
    if (strlen(digest) == 32) {
      char buf[40];
      buf[0] = 'G';
      strncpy(&buf[1], digest, 16);
      strcpy(&buf[17], "\r\nG");
      strncpy(&buf[20], digest+16, 16);
      strcpy(&buf[36], "\r\n");
      if (FlightLog.print(buf) == 0)
          failFlightLog();
    }
}

void reopenFlightLog()
{
    if (FlightLogFail)
        return;
    if (! FlightLogOpen)    // no log file in process
        return;
    if (!FlightLogPath[0])  // file name has not been generated
        return;
    FlightLog = SD.open(FlightLogPath, FILE_WRITE);
    if (!FlightLog) {
        Serial.println("Failed to re-open flight log for writing");
        FlightLogFail = true;
        return;
    }
    if (!FlightLog.seek(FlightLogPosition)) {
        Serial.println("Flight log seek() failed");
        FlightLog.close();
        FlightLogFail = true;
        return;
    }
}

void closeFlightLog()
{
    if (! FlightLogOpen)    // no log file in process
        return;
    if (!FlightLogFail)
        reopenFlightLog();
    if (FlightLogFail)
        return;
    if (data_block_used > 0) {
        if (FlightLog.write((const uint8_t*) data_block_buf, data_block_used) < data_block_used) {
            failFlightLog();
            return;
        }
    }
    //data_block_used = 0;
    //FlightLogPosition = 0;
    write_g_record(md5_a);
    write_g_record(md5_b);
    write_g_record(md5_c);
    write_g_record(md5_d);
    FlightLog.close();
    FlightLogOpen = false;
    FlightLogClosed = millis();
    //FlightLogPath[0] = '\0';   // leave intact for Web.cpp/flightlogfile()
    Serial.println("Flight log closed");
}

void MD5_update(const char *data, size_t size)
{
    MD5::MD5Update(md5_a, data, size);
    MD5::MD5Update(md5_b, data, size);
    MD5::MD5Update(md5_c, data, size);
    MD5::MD5Update(md5_d, data, size);
}

// common code to the functions below
void igc_file_append(const char *data, size_t size)
{
    if (data_block_used + size > DATA_BLOCK_SIZE) {
        // flush the block before writing the new data
        if (!FlightLogFail)
            reopenFlightLog();
        if (FlightLogFail)
            return;
        if (FlightLog.write((const uint8_t*) data_block_buf, data_block_used) < data_block_used) {
            failFlightLog();
            return;
        }
        FlightLogPosition += data_block_used;  // file position before the G-record
        data_block_used = 0;
        // temporarily close the file complete with a G-record
        // finalize a copy of the MD5 context - keep the main context un-finalized
        // this is based on the MD5 state *before* updating with the new data
        *md5_a_copy = *md5_a;
        *md5_b_copy = *md5_b;
        *md5_c_copy = *md5_c;
        *md5_d_copy = *md5_d;
        write_g_record(md5_a_copy);
        write_g_record(md5_b_copy);
        write_g_record(md5_c_copy);
        write_g_record(md5_d_copy);
        FlightLog.close();
        Serial.print("Flight log updated, size now: ");
        Serial.println(FlightLogPosition);
        yield();
    }
    strncpy(data_block_buf + data_block_used, data, size);
    data_block_used += size;
    //Serial.print("data_block_used = ");
    //Serial.println(data_block_used);
}

// assume chars are IGC valid, input must be const
void igc_file_append_const(const char *data)
{
    if (! FlightLogOpen)
        return;
    size_t size = strlen(data);
    if (size < 3)
        return;   // should not happen
    igc_file_append(data, size);         // include \r\n
    MD5_update(data, size-2);            // exclude \r\n
    yield();
}

// this checks and corrects the chars, they cannot be const
// by default commas are replaced with spaces in both the IGC file and MD5
void igc_file_append_nonconst(char *data, bool keepcommas=false, bool iscomment=false)
{
    if (! FlightLogOpen)
        return;
    char *p = data;
    size_t size = 0;
    for (; *p; ++p) {
        if (*p == '\r')
            break;
        if (!keepcommas || *p != 0x2C)     // if (keepcommas) leave commas in file
            *p = clean_igc_char(*p);
        ++size;
    }
    if (size == 0)
        return;   // should not happen
    igc_file_append(data, size+2);     // include \r\n
    if (iscomment)
        return;                       // skip MD5 calc
    if (keepcommas) {
        char *q = data;
        for (p=data; *p; ++p) {
            if (*p == '\r')
                break;
            if (*p != 0x2C) {
                *q++ = *p;             // *skip* the commas in MD5 calculation
            } else {
                --size;
                if (size == 0)
                    return;
            }
        }
    }
    MD5_update(data, size);            // exclude \r\n
    yield();
}

// this checks and corrects the chars, they cannot be const
// commas are kept in the file but skipped in the MD5 calc
void igc_file_append_commas(char *data)
{
    igc_file_append_nonconst(data, true, false);
}

// this checks and corrects the chars, they cannot be const
// this excludes the (LPLT) comment from the MD5 calculation
void igc_file_append_comment(char *data)
{
    igc_file_append_nonconst(data, true, true);
}


// Read one line from an open text file
// Handle possibly different types of line endings
// Used for reading a config file with pilot name etc
char *
getline(File file, char *buf, int limit)
{
    char *cp, *tp;
    char c;
    cp = buf;
    tp = cp + (limit-2);
    while (file.available() && cp<tp) {
        c = file.read();
        if (cp == buf) {
            if (c == '\r')  continue;
            if (c == '\n')  continue;
        }
        if (c == '\r')  break;
        if (c == '\n')  break;
        if (c == '\0')  break;
        *cp++ = c;
    }
    *cp = '\0';
    if (cp == buf)     // read nothing
        return (NULL);
    return (buf);
}


/*
Short file name style: YMDCXXXF.IGC
Y = Year; value 0 to 9, cycling every 10 years
M = Month; value 1 to 9 then A for 10, B=11, C=12.
D = Day; value 1 to 9 then A=10, B=11, C=12, ...
C = manufacturer's IGC code letter
XXX = unique FR Serial Number; 3 alphanumeric characters
F = Flight number of the day; 1 to 9 then A=10 through Z=35
*/

void makeFlightLogName()
{
    char buf[4];
    String basename = "/logs/";
    uint16_t year = gnss.date.year() - 2000;
    char c = ((uint8_t)'0' + (uint8_t)(year % 10));
    basename += c;
    uint8_t month = gnss.date.month();
    if (month < 10)
        c = ((uint8_t)'0' + month);
    else
        c = ((uint8_t)'A' + (month-10));
    basename += c;
    uint8_t day = gnss.date.day();
    if (day < 10)
        c = ((uint8_t)'0' + day);
    else
        c = ((uint8_t)'A' + (day-10));
    basename += c;
    basename += "X";
    String chip = String((SoC->getChipId() & 0x0FFF), HEX);
    strcpy(buf,chip.c_str());
    basename += strupr(buf);
    int flightnum = 1;
    char flightn = '1';
    String filename = basename;
    filename += flightn;
    filename += ".IGC";
    while (SD.exists(filename)) {
        Serial.print(filename);
        Serial.println(" already exists");
        ++flightnum;
        if (flightnum < 10) {
            ++flightn;
        } else if (flightnum < 36) {
            flightn = ('A' + (flightnum-10));
        } else {
            SD.remove(filename);   // overwrite this one
            break;
        }
        filename = basename;
        filename += flightn;
        filename += ".IGC";
    }
    strcpy(FlightLogPath, filename.c_str());
}

bool writeIGCHeader()
{
    if (! FlightLogOpen)
        return false;

    if (!SD.exists(INI_FILE)) {
        Serial.println(F("Creating IGC_CONF file..."));
        File file = SD.open(INI_FILE, FILE_WRITE);
        if (! file) {
            Serial.println(F("... failed"));
            return false;
        }
        file.println(CONFIG_DEFAULT_PILOT);
        file.println(CONFIG_DEFAULT_TYPE);
        file.println(CONFIG_DEFAULT_REG);
        file.println(CONFIG_DEFAULT_CS);
        file.close();
    }

    File conf = SD.open(INI_FILE, FILE_READ);
    if (! conf) {
        Serial.println(F("failed to open IGC_CONF file"));
        return false;
    }
    char buf[80];
    Serial.println(F("Writing IGC Header..."));
    snprintf(buf, 80, "AXLK%06X\r\n", (SoC->getChipId() & 0x00FFFFFF));
    strupr(buf);
    igc_file_append_nonconst(buf);    // 6 hex digits (only 3 in file name)
    int y = gnss.date.year() - 2000;
    snprintf(buf, 80, "HFDTE%02d%02d%02d\r\n", gnss.date.day(), gnss.date.month(), y);
    igc_file_append_nonconst(buf);
    igc_file_append_const("HFFXA035\r\n");
    char buf2[52];
    const char *cp;
    cp = getline(conf, buf2, sizeof(buf2));
    if (cp == NULL)
        cp = CONFIG_DEFAULT_PILOT;
    snprintf(buf, 80, "HFPLTPILOTINCHARGE: %s\r\n", cp);
    Serial.print(buf);
    igc_file_append_commas(buf);
    cp = getline(conf, buf2, sizeof(buf2));
    if (cp == NULL)
        cp = CONFIG_DEFAULT_TYPE;
    snprintf(buf, 80, "HFGTYGLIDERTYPE: %s\r\n", cp);
    Serial.print(buf);
    igc_file_append_commas(buf);
    cp = getline(conf, buf2, sizeof(buf2));
    if (cp == NULL)
        cp = CONFIG_DEFAULT_REG;
    snprintf(buf, 80, "HFGIDGLIDERID: %s\r\n", cp);
    Serial.print(buf);
    igc_file_append_commas(buf);
    cp = getline(conf, buf2, sizeof(buf2));
    if (cp == NULL)
        cp = CONFIG_DEFAULT_CS;
    snprintf(buf, 80, "HFCIDCOMPETITIONID: %s\r\n", cp);
    Serial.print(buf);
    igc_file_append_commas(buf);
    //printf("HFCCLCOMPETITIONCLASS: %s\r\n",____);
    conf.close();
    igc_file_append_const("HFFTYFRTYPE: SoftRF\r\n");
    snprintf(buf, 80, "HFRFWFIRMWAREVERSION: %s\r\n", SOFTRF_FIRMWARE_VERSION);
    //Serial.print(buf);
    igc_file_append_nonconst(buf);
    igc_file_append_const("HFRHWHARDWAREVERSION: T-Beam\r\n");
    //igc_file_append_const("HFGPSRECEIVER: Generic\r\n");
    if (baro_chip != NULL) {
        strcpy(buf, "HFPRSPRESSALTSENSOR: Bosch,BME280,9163m\r\n");
        igc_file_append_commas(buf);
    } else {
        igc_file_append_const("HFALPALTPRESSURE:ISA\r\n");
    }
    igc_file_append_const("HFALGALTGPS:GEO\r\n");     // for non IGC loggers
    igc_file_append_const("HFDTM100GPSDATUM: WGS-1984\r\n");
    igc_file_append_const("I00\r\n");   // >>> may add FXA etc later
    return true;  
}

void init_md5()
{
#define ul (unsigned long)
    // LK8000 keys:
    MD5::MD5Initialize(md5_a, ul 0x63e54c01, ul 0x25adab89, ul 0x44baecfe, ul 0x60f25476);
    MD5::MD5Initialize(md5_b, ul 0x41e24d03, ul 0x23b8ebea, ul 0x4a4bfc9e, ul 0x640ed89a);
    MD5::MD5Initialize(md5_c, ul 0x61e54e01, ul 0x22cdab89, ul 0x48b20cfe, ul 0x62125476);
    MD5::MD5Initialize(md5_d, ul 0xc1e84fe8, ul 0x21d1c28a, ul 0x438e1a12, ul 0x6c250aee);
#undef ul
}

// XCsoar keys:
//  { 0x1C80A301,0x9EB30b89,0x39CB2Afe,0x0D0FEA76 },
//  { 0x48327203,0x3948ebea,0x9a9b9c9e,0xb3bed89a },
//  { 0x67452301,0xefcdab89,0x98badcfe,0x10325476 },
//  { 0xc8e899e8,0x9321c28a,0x438eba12,0x8cbe0aee },

void openFlightLog()
{
    if (FlightLogFail)
        return;
    if (FlightLogOpen)
        return;
    if (GNSSTimeMarker == 0)
        return;
    if (settings->logflight == FLIGHT_LOG_ALWAYS) {
        // pause before starting another log, to allow shutdown instead
        if (FlightLogClosed != 0 && millis() < FlightLogClosed + 60000)
            return;
    }
    makeFlightLogName();
    FlightLog = SD.open(FlightLogPath, FILE_WRITE);
    if (FlightLog) {
        FlightLog.close();      // will be reopen()ed later
        FlightLogOpen = true;
        Serial.print("New flight log: ");
        Serial.println(FlightLogPath);
        data_block_used = 0;
        FlightLogPosition = 0;
        init_md5();
        writeIGCHeader();       // into PSRAM block
        return;
    }
    Serial.println("Failed to open flight log for writing");
    FlightLogFail = true;
}

bool logFlightPosition()
{
    if (! FlightLogOpen && settings->logflight == FLIGHT_LOG_ALWAYS)
        openFlightLog();
    // if FLIGHT_LOG_AIRBORNE then Wind.cpp/this_airborne() will open the file on takeoff 

    // >>> testing: simulate takeoff 90 seconds after GPS fix
    //if (! FlightLogOpen && settings->logflight == FLIGHT_LOG_AIRBORNE
    //               && millis() > GNSSTimeMarker + 60000) {
    //    openFlightLog();
    //    Serial.println("90s - open log");
    //}

    static bool prepositioning = false;
    if (! FlightLogOpen) {
        // waiting for takeoff
        // populate a ring buffer of positions
        // to be used to start the flight log once it is opened
        if (FlightLogFail)
            return false;
        if (settings->logflight != FLIGHT_LOG_AIRBORNE
         && settings->logflight != FLIGHT_LOG_TRAFFIC)
            return false;
        prepositioning = true;
    } else if (prepositioning) {   // but now FlightLogOpen
        // write the stored positions to the newly opened file
        for (int i=0; i<num_pre_positions; i++) {
//if (i==0) {
//Serial.print("incl pre-pos from: [");
//Serial.print(pre_positions_head);
//Serial.print("]: ");
//Serial.print(pre_positions_buf + (B_RECORD_SIZE * pre_positions_head));
//}
            igc_file_append_nonconst(pre_positions_buf + (B_RECORD_SIZE * pre_positions_head));
            pre_positions_head = (pre_positions_head + 1) % PRE_POS_NUM;
        }
        prepositioning = false;
        num_pre_positions = 0;     // in case a new flight log is started later
        pre_positions_head = 0;
        pre_positions_next = 0;
    }

    // B1751494352910N07215306WA0021000210\r\n
    // Number of time/lat/lon digits in GGA varies between GNSS module types:
    // $GPGGA,192054.00,4521.2812,N,07547.2398,W,...
    // $GPGGA,235317.00,4003.90395,N,10512.57934,W,...
    // $GNGGA,144935.000,4521.30100,N,07547.22870,W,...
    char *gp = &GPGGA_Copy[7];   // after the "$GPGGA,", start of timestamp
    //if (strlen(gp) < 35) {
    if (*gp == '\0') {           // set in GNSS.cpp
        //Serial.print("Bad GGA skipped");
        //Serial.println(GPGGA_Copy);
        return false;
    }
    char *bp = &brecord_template[1];
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    while (*gp++ != ',') { if (gp > &GPGGA_Copy[20]) break; }   // seek to start of lat
    *bp++ = *gp++;   // degrees
    *bp++ = *gp++;
    *bp++ = *gp++;   // minutes
    *bp++ = *gp++;
    if (*gp != '.') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return false;
    }
    ++gp;            // skip the period
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    while (*gp++ != ',') { if (gp > &GPGGA_Copy[32]) break; }   // seek to N or S
    if (*gp != 'N' && *gp != 'S') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return false;
    }
    *bp++ = *gp++;
    ++gp;            // skip the comma, to start of lon
    *bp++ = *gp++;   // degrees
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;   // minutes
    *bp++ = *gp++;
    if (*gp != '.') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return false;
    }
    ++gp;            // skip the period
    *bp++ = *gp++;
    *bp++ = *gp++;
    *bp++ = *gp++;
    while (*gp++ != ',') { if (gp > &GPGGA_Copy[NMEA_BUFFER_SIZE-2]) break; }   // seek to E or W
    if (*gp != 'E' && *gp != 'W') {
        Serial.print("Bad GGA: ");
        Serial.println(GPGGA_Copy);
        return false;
    }
    *bp++ = *gp;

    int galt = (int) ThisAircraft.altitude;
    int palt;
    if (baro_chip != NULL)
        palt = (int) ThisAircraft.pressure_altitude;
    else
        palt = galt;

    char buf[B_RECORD_SIZE];
    char *pp = buf;
    if (prepositioning)  pp = pre_positions_buf + (B_RECORD_SIZE * pre_positions_next);
    snprintf(pp, B_RECORD_SIZE, brecord_template, palt, galt);

    if (prepositioning) {
//Serial.print("pre-position[");
//Serial.print(pre_positions_next);
//Serial.print("]: ");
//Serial.print(pp);
        pre_positions_next = (pre_positions_next + 1) % PRE_POS_NUM;
        if (num_pre_positions < PRE_POS_NUM)
            ++num_pre_positions;
        else  // table was already full, overwrote the old head
            pre_positions_head = (pre_positions_head + 1) % PRE_POS_NUM;
    } else {
        igc_file_append_nonconst(buf);
        //igc_file_append_const(pp);  if can skip the checking of chars
    }

    return true;
}

// must be given a null-terminated string
void FlightLogComment(const char *data)
{
    if (FlightLogFail)
        return;
    if (! FlightLogOpen)
        return;
    char buf[80];
    strcpy(buf, "LPLT");
    size_t len = strlen(data);
    if (len > 72)  len = 72;   // IGC format limits lines to 76 chars
    strncpy(buf+4, data, len);
    len += 4;
    if (buf[len-2] != '\r') {
        buf[len++] = '\r';
        buf[len++] = '\n';
    }
    buf[len] = '\0';
    //if (secure)
    //    igc_file_append_commas(buf)
    //else
    igc_file_append_comment(buf);
}

// run a test of MD5 machinery
// this processes the text as-is, including commas
void test_final(MD5_CTX *md5)
{
    MD5::MD5Final(md5);
    char *digest = MD5::make_digest(md5);
    char buf[40];
    buf[0] = 'G';
    strncpy(&buf[1], digest, 16);
    strcpy(&buf[17], "\r\nG");
    strncpy(&buf[20], digest+16, 16);
    strcpy(&buf[36], "\r\n");
    Serial.print(buf);
}
void MD5_test()
{
    if (! (settings->debug_flags & DEBUG_SIMULATE))  return;
    Serial.println("MD5 test:");
    //MD5::make_hash(md5_a, "abcdefghijklmnopqrstuvwxyz");
    //Serial.println("MD5 should be:   c3fcd3d76192e4007dfb496cca67e13b");
    //Serial.print("MD5 computed as: ");
    //Serial.println(MD5::make_digest(md5_a));
    init_md5();
    File file = SD.open("/logs/TEST.TXT", FILE_READ);
    if (! file)  return;
    const char *cp;
    char buf[80];
    while (cp = getline(file, buf, sizeof(buf))) {
        MD5_update(cp, strlen(cp));
        yield();
    }
    file.close();
    test_final(md5_a);
    test_final(md5_b);
    test_final(md5_c);
    test_final(md5_d);
}

#endif
