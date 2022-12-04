/*
 * Platform_ESP32.h
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
#if defined(ESP32)

#ifndef PLATFORM_ESP32_H
#define PLATFORM_ESP32_H

#if defined(ARDUINO)
#include <Arduino.h>
#endif /* ARDUINO */

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

/* Maximum of tracked flying objects is now SoC-specific constant */
#define MAX_TRACKING_OBJECTS  9

#define SerialInput           Serial1

/* Peripherals */

#define LEDC_CHANNEL_BUZZER     0
#define LEDC_RESOLUTION_BUZZER  8

#define SOC_GPIO_PIN_GNSS_RX  21
#define SOC_GPIO_PIN_GNSS_TX  22

//#define SOC_GPIO_PIN_BUZZER   14   // used this on the T-Beam

/* should be OK for most ESP32 boards: */
#define SOC_GPIO_PIN_STROBE    25
#define SOC_GPIO_PIN_BUZZER    26
#define SOC_GPIO_PIN_BUZZER2   27
#define SOC_GPIO_PIN_DCBUZZ    33

#define SOC_BUTTON_MODE_DEF     0

/* The EZSBC ESP32_SW board has LEDs on these pins */
/* On other boards using these pins is probably harmless */
#define EZSBC
#ifdef EZSBC
#define SOC_GPIO_PIN_LED_RED   16   // independent RGB LED
#define SOC_GPIO_PIN_LED_GREEN 17
#define SOC_GPIO_PIN_LED_BLUE  18
#define SOC_GPIO_PIN_LED_BLUE2 19   // part of the USB RGB LED
#else
#define SOC_GPIO_PIN_LED_RED   SOC_UNUSED_PIN
#define SOC_GPIO_PIN_LED_GREEN SOC_UNUSED_PIN
#define SOC_GPIO_PIN_LED_BLUE  SOC_UNUSED_PIN
#define SOC_GPIO_PIN_LED_BLUE2 SOC_UNUSED_PIN
#endif

/* Boya Microelectronics Inc. */
#define BOYA_ID               0x68
#define BOYA_BY25Q32AL        0x4016

/* ST / SGS/Thomson / Numonyx / XMC(later acquired by Micron) */
#define ST_ID                 0x20
#define XMC_XM25QH128C        0x4018

#define MakeFlashId(v,d)      ((v  << 16) | d)
#define CCCC(c1, c2, c3, c4)  ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

#define MAX_FILENAME_LEN      64
#define WAV_FILE_PREFIX       "/Audio/"

#define POWER_SAVING_WIFI_TIMEOUT 600000UL /* 10 minutes */

/* these are data structures to process wav file */
typedef enum headerState_e {
    HEADER_RIFF, HEADER_FMT, HEADER_DATA, DATA
} headerState_t;

typedef struct wavRiff_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint32_t format;
} wavRiff_t;

typedef struct wavProperties_s {
    uint32_t chunkID;
    uint32_t chunkSize;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
} wavProperties_t;

extern bool loopTaskWDTEnabled;

extern WebServer server;

#endif /* PLATFORM_ESP32_H */

#endif /* ESP32 */
