#include "../system/SoC.h"
#if defined(USE_SD_CARD)

#include <SPI.h>
#include <SD.h>
#include <FS.h>

#include "SDcard.h"
#include "EEPROM.h"
#include "OLED.h"
#include "../protocol/data/IGC.h"

SPIClass SD_VSPI = SPIClass(VSPI);  // for the SD card sharing the bus with LORA
SPIClass SD_HSPI = SPIClass(HSPI);  // for a separate SPI bus for the SD card

static uint8_t ss_pin;
static uint8_t cardType = CARD_NONE;
static File SDfile;
static bool SDfileOpen = false;

// check for firmware update file
// if found, do update, then delete the file and reboot
void performUpdate()
{
    if (! SD.exists("/firmware")) {
        SD.mkdir("/firmware");
        //return;
    }

    File updateBin = SD.open("/firmware/SoftRF.bin", FILE_READ);
    size_t updateSize = updateBin.size();
    if (updateSize == 0) {
        Serial.println("No firmware found on SD card");
        return;
    }

    Serial.println("firmware update...");
    blue_LED_1hz();
    OLED_msg("UPDATE", "...");
    bool success = false;
    if (Update.begin(updateSize)) {      
        size_t written = Update.writeStream(updateBin);
        if (written == updateSize) {
            Serial.print("Written: ");
            Serial.print(written);
            Serial.println(" successfully");
            if (Update.end()) {
                if (Update.isFinished()) {
                    success = true;
                }
            }
        }
    }
    updateBin.close();
    if (success) {
        Serial.println("update success");
        blue_LED_on();
        OLED_msg("UPDATE", "SUCCESS");
        SD.remove("/firmware/SoftRF.bin");
        delay(2000);               
        Serial.println("rebooting...");
        delay(1000);
        SoC->reset();         
    } else {
        Serial.println("update failed");
        Update.printError(Serial);
        blue_LED_4hz();
        OLED_msg("UPDATE", "FAILED");
        delay(2000);
    }
}

void SD_setup() {
  if (settings->sd_card == SD_CARD_NONE)
      return;
  if (settings->sd_card == SD_CARD_13_25) {
      Serial.println("starting SD as HSPI on pins 13, 25, 2, 0");
      if (ESP32_pin_reserved(SD1_SCK,  false, "SD SCK"))  return;
      if (ESP32_pin_reserved(SD1_MISO, false, "SD MISO")) return;
      if (ESP32_pin_reserved(SD1_MOSI, false, "SD MOSI")) return;
      if (ESP32_pin_reserved(SD1_SS,   false, "SD SS"))   return;
      ss_pin = SD1_SS;
      SD_HSPI.begin(SD1_SCK, SD1_MISO, SD1_MOSI, ss_pin);
  } else if (settings->sd_card == SD_CARD_13_VP) {
      uint8_t miso_pin;
      if (hw_info.revision >= 8) {
          Serial.println("starting SD as HSPI on pins 13, VP, 2, 0");
          if (ESP32_pin_reserved(SD2_MISO, false, "SD MISO")) return;
          miso_pin = SD2_MISO;
      } else {
          Serial.println("starting SD as HSPI on pins 13, 4, 2, 0");
          if (ESP32_pin_reserved(SD2_MISO_V7, false, "SD MISO")) return;
          miso_pin = SD2_MISO_V7;
      }
      if (ESP32_pin_reserved(SD2_MOSI, false, "SD MOSI")) return;
      if (ESP32_pin_reserved(SD2_SCK,  false, "SD SCK"))  return;
      if (ESP32_pin_reserved(SD2_SS,   false, "SD SS"))   return;
      ss_pin = SD2_SS;
      SD_HSPI.begin(SD2_SCK, miso_pin, SD2_MOSI, ss_pin);
  } else {   // (settings->sd_card == SD_CARD_LORA)
      Serial.println("starting SD sharing VSPI with LORA, SS on pin 0");
      if (ESP32_pin_reserved(SD3_SS, false, "SD SS")) return;
      ss_pin = SD3_SS;
      ESP32_pin_reserved(SD3_SCK,  true, "SD SCK");
      ESP32_pin_reserved(SD3_MISO, true, "SD MISO");
      ESP32_pin_reserved(SD3_MOSI, true, "SD MOSI");
      SD_VSPI.begin(SD3_SCK, SD3_MISO, SD3_MOSI, ss_pin);
      digitalWrite(SOC_GPIO_PIN_SS, HIGH);     // LORA not selected
      pinMode(SOC_GPIO_PIN_SS, OUTPUT);
  }
  digitalWrite(ss_pin, HIGH);             // rest state: not selected
  pinMode(ss_pin, OUTPUT);
  bool mounted;
  if (settings->sd_card == SD_CARD_LORA)
      mounted = SD.begin(ss_pin,SD_VSPI);
  else
      mounted = SD.begin(ss_pin,SD_HSPI);
  if(!mounted){
      Serial.println("SD mount failed");
      return;
  }
  cardType = SD.cardType();
  if(cardType == CARD_NONE){
      Serial.println("No SD card detected");
      return;
  }
  uint64_t cardSize = SD.cardSize();
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }
  cardSize >>= 20;
  Serial.printf("SD Card Size: %dMB\n", (uint32_t)cardSize);

  // check for firmware update file
  performUpdate();

  if (! SD.exists("/logs"))
      SD.mkdir("/logs");

  if (! SD.exists("/logs/old"))
      SD.mkdir("/logs/old");

  // keep a copy of the existing log.txt (if non-empty)
  // so can download it after a reboot
  size_t logsize = 0;
  SDfile = SD.open("/logs/log.txt", FILE_READ);
  if (SDfile) {
    logsize = SDfile.size();
    SDfile.close();
  }
  if (logsize > 0) {
    SD.remove("/logs/oldlog.txt");
    SD.rename("/logs/log.txt", "/logs/oldlog.txt");
  }

  SDfile = SD.open("/logs/log.txt", FILE_WRITE);
  if (SDfile)
      SDfileOpen = true;
  else
      Serial.println("Failed to open SD/logs/log.txt for writing");
}

void SD_log(const char * message)
{
  if (SDfileOpen) {
    bool success = SDfile.print(message);
    if (success) {
        Serial.println("- Message appended to SD log.txt");
    } else {
        Serial.println("- Append to SD log.txt failed");
    }
    yield();
  }
}

void closeSDlog()
{
  if (SDfileOpen) {
      //digitalWrite(ss_pin, LOW);
      SDfile.close();
      //digitalWrite(ss_pin, HIGH);
      SDfileOpen = false;
  }
}

#endif /* USE_SD_CARD */
