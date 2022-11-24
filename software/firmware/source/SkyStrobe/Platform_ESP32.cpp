/*
 * Platform_ESP32.cpp
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

#include <SPI.h>
#include <esp_err.h>
#include <esp_wifi.h>
#include <soc/rtc_cntl_reg.h>
#include <rom/spi_flash.h>
#include <soc/adc_channel.h>
#include <flashchips.h>

#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "WiFiHelper.h"
#include "BluetoothHelper.h"
#include "Sound.h"

#include "Platform_ESP32.h"

#include "SkyStrobe.h"

#include <battery.h>

// #include <sqlite3.h>
// #include <SD.h>

#include "driver/i2s.h"

#include <esp_wifi.h>
#include <esp_bt.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  28        /* Time ESP32 will go to sleep (in seconds) */

WebServer server ( 80 );

/*
 * TTGO-T5S. Pin definition

#define BUSY_PIN        4
#define CS_PIN          5
#define RST_PIN         16
#define DC_PIN          17
#define SCK_PIN         18
#define MOSI_PIN        23

P1-1                    21
P1-2                    22 (LED)

I2S MAX98357A           26
                        25
                        19

I2S MIC                 27
                        32
                        33

B0                      RST
B1                      38
B2                      37
B3                      39

SD                      2
                        13
                        14
                        15

P2                      0
                        12
                        13
                        RXD
                        TXD
                        34
                        35 (BAT)
 */


static union {
  uint8_t efuse_mac[6];
  uint64_t chipmacid;
};


RTC_DATA_ATTR int bootCount = 0;

static uint32_t ESP32_getFlashId()
{
  return g_rom_flashchip.device_id;
}

static void ESP32_fini()
{
  int mode_button_pin = SOC_BUTTON_MODE_DEF;

  esp_wifi_stop();
  esp_bt_controller_disable();

  /*
   * manually apply this fix onto Arduino Core for ESP32:
   * https://github.com/espressif/arduino-esp32/pull/4272
   * to put SD card into idle state
   *
   *  SkyView EZ sleep current (from 3.7V battery source):
   *  ---------------------------------------------------
   *  SD card in  -            0.2 mA
   *  SD card out -            0.1 mA
   */
  esp_sleep_enable_ext1_wakeup(1ULL << mode_button_pin, ESP_EXT1_WAKEUP_ALL_LOW);

//  Serial.println("Going to sleep now");
//  Serial.flush();

  esp_deep_sleep_start();
}

static void ESP32_setup()
{
  esp_err_t ret = ESP_OK;
  uint8_t null_mac[6] = {0};

  ++bootCount;

  ret = esp_efuse_mac_get_custom(efuse_mac);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Get base MAC address from BLK3 of EFUSE error (%s)", esp_err_to_name(ret));
    /* If get custom base MAC address error, the application developer can decide what to do:
     * abort or use the default base MAC address which is stored in BLK0 of EFUSE by doing
     * nothing.
     */

    ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
    chipmacid = ESP.getEfuseMac();
  } else {
    if (memcmp(efuse_mac, null_mac, 6) == 0) {
      ESP_LOGI(TAG, "Use base MAC address which is stored in BLK0 of EFUSE");
      chipmacid = ESP.getEfuseMac();
    }
  }

  uint32_t flash_id = ESP32_getFlashId();

  /*
   *    Board         |   Module   |  Flash memory IC
   *  ----------------+------------+--------------------
   *  DoIt ESP32      | WROOM      | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.0   | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T3  V2.1.6 | PICO-D4 IC | GIGADEVICE_GD25Q32
   *  TTGO T22 V06    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V08    |            | WINBOND_NEX_W25Q32_V
   *  TTGO T22 V11    |            | BOYA_BY25Q32AL
   *  TTGO T8  V1.8   | WROVER     | GIGADEVICE_GD25LQ32
   *  TTGO T5S V1.9   |            | WINBOND_NEX_W25Q32_V
   *  TTGO T5S V2.8   |            | BOYA_BY25Q32AL
   *  TTGO T5  4.7    | WROVER-E   | XMC_XM25QH128C
   *  TTGO T-Watch    |            | WINBOND_NEX_W25Q128_V
   */

#if 0
  if (psramFound()) {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25LQ32):
      /* ESP32-WROVER module */
      hw_info.revision = HW_REV_T8_1_8;
      break;
    case MakeFlashId(ST_ID, XMC_XM25QH128C):
      /* custom ESP32-WROVER-E module with 16 MB flash */
      hw_info.revision = HW_REV_T5_1;
      break;
    default:
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  } else {
    switch(flash_id)
    {
    case MakeFlashId(GIGADEVICE_ID, GIGADEVICE_GD25Q32):
      hw_info.revision = HW_REV_DEVKIT;
      break;
    case MakeFlashId(WINBOND_NEX_ID, WINBOND_NEX_W25Q32_V):
      hw_info.revision = HW_REV_T5S_1_9;
      break;
    case MakeFlashId(BOYA_ID, BOYA_BY25Q32AL):
      hw_info.revision = HW_REV_T5S_2_8;
      break;
    default:
      hw_info.revision = HW_REV_UNKNOWN;
      break;
    }
  }
#endif

  ledcSetup(LEDC_CHANNEL_BUZZER, 0, LEDC_RESOLUTION_BUZZER);
}

static uint32_t ESP32_getChipId()
{
  return (uint32_t) efuse_mac[5]        | (efuse_mac[4] << 8) | \
                   (efuse_mac[3] << 16) | (efuse_mac[2] << 24);
}

static bool ESP32_EEPROM_begin(size_t size)
{
  return EEPROM.begin(size);
}

static void ESP32_Sound_tone(int hz, uint8_t volume)
{
  if (SOC_GPIO_PIN_BUZZER != SOC_UNUSED_PIN && volume != BUZZER_OFF) {
    if (hz > 0) {
      ledcAttachPin(SOC_GPIO_PIN_BUZZER, LEDC_CHANNEL_BUZZER);

      ledcWriteTone(LEDC_CHANNEL_BUZZER, hz);
      ledcWrite(LEDC_CHANNEL_BUZZER, volume == BUZZER_VOLUME_FULL ? 0xFF : 0x07);
    } else {
      ledcWriteTone(LEDC_CHANNEL_BUZZER, 0); // off

      ledcDetachPin(SOC_GPIO_PIN_BUZZER);
      pinMode(SOC_GPIO_PIN_BUZZER, INPUT_PULLDOWN);
    }
  }
}

static const int8_t ESP32_dB_to_power_level[21] = {
  8,  /* 2    dB, #0 */
  8,  /* 2    dB, #1 */
  8,  /* 2    dB, #2 */
  8,  /* 2    dB, #3 */
  8,  /* 2    dB, #4 */
  20, /* 5    dB, #5 */
  20, /* 5    dB, #6 */
  28, /* 7    dB, #7 */
  28, /* 7    dB, #8 */
  34, /* 8.5  dB, #9 */
  34, /* 8.5  dB, #10 */
  44, /* 11   dB, #11 */
  44, /* 11   dB, #12 */
  52, /* 13   dB, #13 */
  52, /* 13   dB, #14 */
  60, /* 15   dB, #15 */
  60, /* 15   dB, #16 */
  68, /* 17   dB, #17 */
  74, /* 18.5 dB, #18 */
  76, /* 19   dB, #19 */
  78  /* 19.5 dB, #20 */
};

static void ESP32_WiFi_setOutputPower(int dB)
{
  if (dB > 20) {
    dB = 20;
  }

  if (dB < 0) {
    dB = 0;
  }

  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(ESP32_dB_to_power_level[dB]));
}

static bool ESP32_WiFi_hostname(String aHostname)
{
  return WiFi.setHostname(aHostname.c_str());
}

static void ESP32_swSer_begin(unsigned long baud)
{
  SerialInput.begin(baud, SERIAL_8N1, SOC_GPIO_PIN_GNSS_RX, SOC_GPIO_PIN_GNSS_TX);
  SerialInput.setRxBufferSize(baud / 10); /* 1 second */
}

static void ESP32_swSer_enableRx(boolean arg)
{

}

static uint32_t ESP32_maxSketchSpace()
{
  return 0x1E0000;
}

static void ESP32_WiFiUDP_stopAll()
{
/* not implemented yet */
}

static void ESP32_Battery_setup()
{
#if 0
  calibrate_voltage(settings->adapter == ADAPTER_TTGO_T5S ?
                    ADC1_GPIO35_CHANNEL : ADC1_GPIO36_CHANNEL);
#endif
}

static float ESP32_Battery_voltage()
{
  float voltage = ((float) read_voltage()) * 0.001 ;

#if 0
  /* T5 has voltage divider 100k/100k on board */
  return (settings->adapter == ADAPTER_TTGO_T5S    ||
          settings->adapter == ADAPTER_TTGO_T5_4_7 ?
          2 * voltage : voltage);
#else
  return voltage;
#endif
}


static size_t ESP32_WiFi_Receive_UDP(uint8_t *buf, size_t max_size)
{
  return WiFi_Receive_UDP(buf, max_size);
}

static int ESP32_WiFi_clients_count()
{
  WiFiMode_t mode = WiFi.getMode();

  switch (mode)
  {
  case WIFI_AP:
    wifi_sta_list_t stations;
    ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

    tcpip_adapter_sta_list_t infoList;
    ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

    return infoList.num;
  case WIFI_STA:
  default:
    return -1; /* error */
  }
}

#if !defined(EXCLUDE_BUTTONS)

#include <AceButton.h>
using namespace ace_button;

AceButton button_mode(SOC_BUTTON_MODE_T5S);
AceButton button_up  (SOC_BUTTON_UP_T5S);
AceButton button_down(SOC_BUTTON_DOWN_T5S);

// The event handler for the button.
void handleEvent(AceButton* button, uint8_t eventType,
    uint8_t buttonState) {

#if 0
  // Print out a message for all events.
  if        (button == &button_mode) {
    Serial.print(F("MODE "));
  } else if (button == &button_up) {
    Serial.print(F("UP   "));
  } else if (button == &button_down) {
    Serial.print(F("DOWN "));
  }

  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);
#endif

  switch (eventType) {
    case AceButton::kEventPressed:
      break;
    case AceButton::kEventReleased:
      if (button == &button_mode) {
        //EPD_Mode();
      } else if (button == &button_up) {
        //EPD_Up();
      } else if (button == &button_down) {
        //EPD_Down();
      }
      break;
    case AceButton::kEventLongPressed:
      if (button == &button_mode) {
        shutdown("NORMAL OFF");
        Serial.println(F("This will never be printed."));
      }
      break;
  }
}

/* Callbacks for push button interrupt */
void onModeButtonEvent() {
  button_mode.check();
}

void onUpButtonEvent() {
  button_up.check();
}

void onDownButtonEvent() {
  button_down.check();
}

static void ESP32_Button_setup()
{
  int mode_button_pin = settings->adapter == ADAPTER_TTGO_T5S ?
                        SOC_BUTTON_MODE_T5S : SOC_BUTTON_MODE_DEF;

  // Button(s) uses external pull up resistor.
  pinMode(mode_button_pin, INPUT);

  button_mode.init(mode_button_pin);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.
  ButtonConfig* ModeButtonConfig = button_mode.getButtonConfig();
  ModeButtonConfig->setEventHandler(handleEvent);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureClick);
  ModeButtonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  ModeButtonConfig->setDebounceDelay(15);
  ModeButtonConfig->setClickDelay(100);
  ModeButtonConfig->setDoubleClickDelay(1000);
  ModeButtonConfig->setLongPressDelay(2000);

  attachInterrupt(digitalPinToInterrupt(mode_button_pin), onModeButtonEvent, CHANGE );

  if (settings->adapter == ADAPTER_TTGO_T5S) {

    // Button(s) uses external pull up resistor.
    pinMode(SOC_BUTTON_UP_T5S,   INPUT);
    pinMode(SOC_BUTTON_DOWN_T5S, INPUT);

    ButtonConfig* UpButtonConfig = button_up.getButtonConfig();
    UpButtonConfig->setEventHandler(handleEvent);
    UpButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    UpButtonConfig->setDebounceDelay(15);
    UpButtonConfig->setClickDelay(100);
    UpButtonConfig->setDoubleClickDelay(1000);
    UpButtonConfig->setLongPressDelay(2000);

    ButtonConfig* DownButtonConfig = button_down.getButtonConfig();
    DownButtonConfig->setEventHandler(handleEvent);
    DownButtonConfig->setFeature(ButtonConfig::kFeatureClick);
    DownButtonConfig->setDebounceDelay(15);
    DownButtonConfig->setClickDelay(100);
    DownButtonConfig->setDoubleClickDelay(1000);
    DownButtonConfig->setLongPressDelay(2000);

    attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_UP_T5S),   onUpButtonEvent,   CHANGE );
    attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_DOWN_T5S), onDownButtonEvent, CHANGE );
  }
}

static void ESP32_Button_loop()
{
  button_mode.check();
}

static void ESP32_Button_fini()
{

}
#endif

static void ESP32_WDT_setup()
{
  enableLoopWDT();
}

static void ESP32_WDT_fini()
{
  disableLoopWDT();
}

const SoC_ops_t ESP32_ops = {
  SOC_ESP32,
  "ESP32",
  ESP32_setup,
  ESP32_fini,
  ESP32_getChipId,
  ESP32_EEPROM_begin,
  ESP32_WiFi_setOutputPower,
  ESP32_WiFi_hostname,
  ESP32_swSer_begin,
  ESP32_swSer_enableRx,
  ESP32_maxSketchSpace,
  ESP32_WiFiUDP_stopAll,
  ESP32_Battery_setup,
  ESP32_Battery_voltage,
  ESP32_WiFi_Receive_UDP,
  ESP32_WiFi_clients_count,
#if defined(EXCLUDE_BUTTONS)
  NULL, NULL, NULL,
#else
  ESP32_Button_setup,
  ESP32_Button_loop,
  ESP32_Button_fini,
#endif
  ESP32_WDT_setup,
  ESP32_WDT_fini,
  ESP32_Sound_tone,
  &ESP32_Bluetooth_ops
};

#endif /* ESP32 */
