
/*
 * BluetoothHelper.cpp
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

/*
    BLE code is based on Neil Kolban example for IDF:
      https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini    
    HM-10 emulation and adaptation for SoftRF is done by Linar Yusupov.
*/

#if defined(ESP32)

#include "sdkconfig.h"

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "BluetoothHelper.h"
#include "BatteryHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"
#include "Sound.h"

#include "SkyStrobe.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

#include <BluetoothSerial.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// this line wasn't in SoftRF code:
#include "BluetoothSerial.h"

#include "esp_gap_bt_api.h"

#include "WiFiHelper.h"   // HOSTNAME

#include <core_version.h>

#if 1   //--------------------- from SoftRF v1.2:

BLEServer* pServer = NULL;
BLECharacteristic* pUARTCharacteristic = NULL;
BLECharacteristic* pBATCharacteristic  = NULL;

BLECharacteristic* pModelCharacteristic         = NULL;
BLECharacteristic* pSerialCharacteristic        = NULL;
BLECharacteristic* pFirmwareCharacteristic      = NULL;
BLECharacteristic* pHardwareCharacteristic      = NULL;
BLECharacteristic* pSoftwareCharacteristic      = NULL;
BLECharacteristic* pManufacturerCharacteristic  = NULL;

bool deviceConnected    = false;
bool oldDeviceConnected = false;

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

static uint32_t BLE_Notify_TimeMarker = 0;
static uint32_t BLE_Advertising_TimeMarker = 0;

BLEDescriptor UserDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
//Serial.println("BLE-bridge connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      BLE_Advertising_TimeMarker = millis();
//Serial.println("BLE-bridge disconnected");
    }
};

class UARTCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pUARTCharacteristic) {
      std::string rxValue = pUARTCharacteristic->getValue();

      if (rxValue.length() > 0) {
        BLE_FIFO_RX->write(rxValue.c_str(),
                      (BLE_FIFO_RX->room() > rxValue.length() ?
                      rxValue.length() : BLE_FIFO_RX->room()));
      }
    }
};

#endif   //--------------------- stuff from SoftRF v1.2

BluetoothSerial SerialBT;
String BT_name = HOSTNAME;

Bluetooth_ctl_t ESP32_BT_ctl = {
  .mutex   = portMUX_INITIALIZER_UNLOCKED,
  .command = BT_CMD_NONE,
  .status  = BT_STATUS_NC
};

/* LE */
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* AppDevice;
static BLEClient* pClient;

static BLEUUID  serviceUUID(SERVICE_UUID);
static BLEUUID  charUUID(CHARACTERISTIC_UUID);

// cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

static uint32_t BT_TimeMarker = 0;

static void AppNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (length > 0) {
      BLE_FIFO_RX->write((char *) pData, (BLE_FIFO_RX->room() > length ?
                                          length : BLE_FIFO_RX->room()));
    }
}

class AppClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    ESP32_BT_ctl.status = BT_STATUS_NC;

    Serial.println(F("BLE: disconnected from Server."));
    red_LED(true);
  }
};

class AppAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

  void onResult(BLEAdvertisedDevice advertisedDevice) {

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();

      if (AppDevice) {
        AppDevice->~BLEAdvertisedDevice();
      }

      AppDevice = new BLEAdvertisedDevice(advertisedDevice);
      ESP32_BT_ctl.command = BT_CMD_CONNECT;
    }
  }
};

static void ESP32_BT_SPP_Connection_Manager(void *parameter)
{
  int command;
  int status;

  while (true) {

    portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
    command = ESP32_BT_ctl.command;
    portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

    switch (command)
    {
    case BT_CMD_CONNECT:
        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        status = ESP32_BT_ctl.status;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

        if (status == BT_STATUS_CON) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.command = BT_CMD_NONE;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
          break;
        }

        // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
        // to resolve name to address first, but it allows to connect to different devices with the same name.
        // Set CoreDebugLevel to Info to view devices bluetooth address and device names

        if (SerialBT.connect(settings->server)) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.status = BT_STATUS_CON;
          ESP32_BT_ctl.command = BT_CMD_NONE;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
          blue_LED(true);
          Serial.print(F("BT SPP: Connected to "));
        } else {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.status = BT_STATUS_NC;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
          red_LED(true);
          Serial.print(F("BT SPP: Unable to connect to "));
        }
        Serial.println(settings->server);
        break;

    case BT_CMD_DISCONNECT:
        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        status = ESP32_BT_ctl.status;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

        if (status != BT_STATUS_CON) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.command = BT_CMD_NONE;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
          break;
        }

        // disconnect() may take upto 10 secs max
        if (SerialBT.disconnect()) {
          Serial.print(F("BT SPP: Disconnected from "));
        } else {
          Serial.print(F("BT SPP: Unable to disconnect from "));
        }
        Serial.println(settings->server);

        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        ESP32_BT_ctl.status = BT_STATUS_NC;
        ESP32_BT_ctl.command = BT_CMD_NONE;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
        break;

    case BT_CMD_SHUTDOWN:
        vTaskDelete(NULL);
        break;
    default:
        break;
    }

    delay(1000);
  }
}

static bool ESP32_BLEConnectToServer() {
    pClient->connect(AppDevice);

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print(F("BLE: Failed to find our service UUID: "));
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print(F("BLE: Failed to find our characteristic UUID: "));
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(AppNotifyCallback);

    ESP32_BT_ctl.status = BT_STATUS_CON;
    return true;
}

static void ESP32_Bluetooth_setup()
{
  BT_name += String(SoC->getChipId() & 0x00FFFFFFU, HEX);

  if (settings->connection == CON_BLUETOOTH_SPP)
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      //SerialBT.setPin(settings->key);            // >>> why use a PIN?
      //SerialBT.begin(BT_name.c_str(), true);
      SerialBT.begin(BT_name.c_str());

      xTaskCreate(ESP32_BT_SPP_Connection_Manager, "BT SPP ConMgr Task", 1024, NULL, tskIDLE_PRIORITY, NULL);

      BT_TimeMarker = millis();
    }

  else if (settings->bridge == BRIDGE_BT_SPP)
    {
      esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

      SerialBT.begin(BT_name.c_str());
    }

  else if (settings->connection == CON_BLUETOOTH_LE)
    {
      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_LARGE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SMALL);

      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

      BLEDevice::init("");

      pClient = BLEDevice::createClient();
      pClient->setClientCallbacks(new AppClientCallback());

      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new AppAdvertisedDeviceCallbacks());
      pBLEScan->setInterval(1349);
      pBLEScan->setWindow(449);
      pBLEScan->setActiveScan(true);
      pBLEScan->start(3, false);

      BLE_Notify_TimeMarker = millis();
    }

  else if (settings->bridge == BRIDGE_BT_LE)
    {

#if 1   //--------------------- from SoftRF v1.2:

      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SMALL);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_LARGE);

      // this was absent from v1.2 unless ESP32Sx:
      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

      // Create the BLE Device
      BLEDevice::init((BT_name+"-LE").c_str());

      /*
       * Set the MTU of the packets sent,
       * maximum is 500, Apple needs 23 apparently.
       */
      // BLEDevice::setMTU(23);

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(BLEUUID(UART_SERVICE_UUID));

      // Create a BLE Characteristic
      pUARTCharacteristic = pService->createCharacteristic(
                              BLEUUID(UART_CHARACTERISTIC_UUID),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY |
                              BLECharacteristic::PROPERTY_WRITE_NR
                            );

      UserDescriptor.setValue("HMSoft");
      pUARTCharacteristic->addDescriptor(&UserDescriptor);
      pUARTCharacteristic->addDescriptor(new BLE2902());

      pUARTCharacteristic->setCallbacks(new UARTCallbacks());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_BATTERY));

      // Create a BLE Characteristic
      pBATCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_BATTERY_LEVEL),
                              BLECharacteristic::PROPERTY_READ   |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
      pBATCharacteristic->addDescriptor(new BLE2902());

      // Start the service
      pService->start();

      // Create the BLE Service
      pService = pServer->createService(BLEUUID(UUID16_SVC_DEVICE_INFORMATION));

      // Create BLE Characteristics
      pModelCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MODEL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSerialCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SERIAL_NUMBER_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pFirmwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_FIRMWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pHardwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_HARDWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pSoftwareCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_SOFTWARE_REVISION_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );
      pManufacturerCharacteristic = pService->createCharacteristic(
                              BLEUUID(UUID16_CHR_MANUFACTURER_NAME_STRING),
                              BLECharacteristic::PROPERTY_READ
                            );

      const char *Model = "SkyStrobe";
      char SerialNum[9];
      snprintf(SerialNum, sizeof(SerialNum), "%08X", SoC->getChipId());

      const char *Firmware = "Arduino Core ESP32 " ARDUINO_ESP32_RELEASE;

      char Hardware[9];
      snprintf(Hardware, sizeof(Hardware), "%08X", hw_info.revision);

      const char *Manufacturer  = SKYSTROBE_IDENT;
      const char *Software      = SKYSTROBE_FIRMWARE_VERSION;

      pModelCharacteristic->       setValue((uint8_t *) Model,        strlen(Model));
      pSerialCharacteristic->      setValue((uint8_t *) SerialNum,    strlen(SerialNum));
      pFirmwareCharacteristic->    setValue((uint8_t *) Firmware,     strlen(Firmware));
      pHardwareCharacteristic->    setValue((uint8_t *) Hardware,     strlen(Hardware));
      pSoftwareCharacteristic->    setValue((uint8_t *) Software,     strlen(Software));
      pManufacturerCharacteristic->setValue((uint8_t *) Manufacturer, strlen(Manufacturer));

      // Start the service
      pService->start();

      // Start advertising
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(BLEUUID(UART_SERVICE_UUID));
      pAdvertising->addServiceUUID(BLEUUID(UUID16_SVC_BATTERY));
      pAdvertising->setScanResponse(true);
      pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      pAdvertising->setMaxPreferred(0x12);
      BLEDevice::startAdvertising();

      BLE_Advertising_TimeMarker = millis();

#endif   //--------------------- stuff from SoftRF v1.2

    }
}

static void ESP32_Bluetooth_loop()
{
  bool hasData = false;

  if (settings->connection == CON_BLUETOOTH_SPP)
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int command = ESP32_BT_ctl.command;
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_NC && command == BT_CMD_NONE) {
        portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
        ESP32_BT_ctl.command = BT_CMD_CONNECT;
        portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);
      } else {
        switch (settings->protocol)
        {
        case PROTOCOL_GDL90:
          hasData = GDL90_isConnected();
          break;
        case PROTOCOL_NMEA:
        default:
          hasData = NMEA_isConnected();
          break;
        }

        if (hasData) {
          BT_TimeMarker = millis();
        } else if (millis() - BT_TimeMarker > BT_NODATA_TIMEOUT &&
                   command == BT_CMD_NONE) {
          portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
          ESP32_BT_ctl.command = BT_CMD_DISCONNECT;
          portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

          BT_TimeMarker = millis();
        }
      }
    }

  else if (settings->connection == CON_BLUETOOTH_LE)
    {
      if (ESP32_BT_ctl.command == BT_CMD_CONNECT) {
        if (ESP32_BLEConnectToServer()) {
          Serial.println(F("BLE: connected to Server."));
          blue_LED(true);
        }
        ESP32_BT_ctl.command = BT_CMD_NONE;
      }

      switch (settings->protocol)
      {
      case PROTOCOL_GDL90:
        hasData = GDL90_isConnected();
        break;
      case PROTOCOL_NMEA:
      default:
        hasData = NMEA_isConnected();
        break;
      }

      if (hasData) {
        BT_TimeMarker = millis();
      } else if (millis() - BT_TimeMarker > BT_NODATA_TIMEOUT) {

        Serial.println(F("BLE: attempt to (re)connect..."));
        red_LED(true);

        if (pClient) {
          if (pClient->isConnected()) {
            pClient->disconnect();
          }
        }

        BLEDevice::getScan()->start(3, false);

#if 0
        /* approx. 170 bytes memory leak still remains */
        Serial.print("Free Heap: ");
        Serial.println(ESP.getFreeHeap());
#endif

        BT_TimeMarker = millis();
      }

      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (ESP32_BT_ctl.status == BT_STATUS_CON &&
          (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = (BLE_FIFO_TX->available() < BLE_MAX_WRITE_CHUNK_SIZE ?
                         BLE_FIFO_TX->available() : BLE_MAX_WRITE_CHUNK_SIZE);

          if (size > 0) {
            BLE_FIFO_TX->read((char *) chunk, size);

            pRemoteCharacteristic->writeValue(chunk, size);

            BLE_Notify_TimeMarker = millis();
          }
      }
    }

    else if (settings->bridge == BRIDGE_BT_LE)
    {
      // code from SoftRF v1.2:

      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (deviceConnected && (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          static uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];   // >>> MB added "static"
          size_t size = BLE_FIFO_TX->available();
          size = size < BLE_MAX_WRITE_CHUNK_SIZE ? size : BLE_MAX_WRITE_CHUNK_SIZE;

          if (size > 0) {
//Serial.print("BLE-bridge loop: ");
//Serial.println(size);
            BLE_FIFO_TX->read((char *) chunk, size);

            pUARTCharacteristic->setValue(chunk, size);
            pUARTCharacteristic->notify();
          }

          BLE_Notify_TimeMarker = millis();
      }
      // disconnecting
      if (!deviceConnected && oldDeviceConnected && (millis() - BLE_Advertising_TimeMarker > 500) ) {
          // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          oldDeviceConnected = deviceConnected;
          BLE_Advertising_TimeMarker = millis();
          //Serial.println("BLE disconnected");
          //red_LED(true);
      }
      // connecting
      if (deviceConnected && !oldDeviceConnected) {
          // do stuff here on connecting
          oldDeviceConnected = deviceConnected;
          //Serial.println("BLE reconnected");
          //blue_LED(true);
      }

#if 0   // not applicable to SkyStrobe - will never run on ESP32's battery
      if (deviceConnected && isTimeToBattery()) {
        uint8_t battery_level = Battery_charge();

        pBATCharacteristic->setValue(&battery_level, 1);
        pBATCharacteristic->notify();
      }
#endif
    }
}

static void ESP32_Bluetooth_fini()
{
  if (settings->connection == CON_BLUETOOTH_SPP
       || settings->bridge == BRIDGE_BT_SPP)
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      ESP32_BT_ctl.command = BT_CMD_SHUTDOWN;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      delay(100);

      SerialBT.end();
    }

  else if (settings->connection == CON_BLUETOOTH_LE
            || settings->bridge == BRIDGE_BT_LE)
    {
      BLEDevice::deinit();
    }
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  if (settings->connection == CON_BLUETOOTH_SPP)
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_CON) {
        rval = SerialBT.available();
      }
    }

  else if (settings->connection == CON_BLUETOOTH_LE)
    {
      rval = BLE_FIFO_RX->available();
    }

  else if (settings->bridge == BRIDGE_BT_LE)
    {
      rval = BLE_FIFO_RX->available();
    }
  
  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  if (settings->connection == CON_BLUETOOTH_SPP)
    {
      portENTER_CRITICAL(&ESP32_BT_ctl.mutex);
      int status = ESP32_BT_ctl.status;
      portEXIT_CRITICAL(&ESP32_BT_ctl.mutex);

      if (status == BT_STATUS_CON) {
        rval = SerialBT.read();
      }
    }

  else if (settings->connection == CON_BLUETOOTH_LE)
    {
      rval = BLE_FIFO_RX->read();
    }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  int rval = 0;

  if (settings->bridge == BRIDGE_BT_SPP)
    {
        rval = SerialBT.write(buffer, size);
    }

  else if (settings->bridge == BRIDGE_BT_LE)
    {
      rval = BLE_FIFO_TX->write((char *) buffer,
                          (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    }

  return rval;
}

Bluetooth_ops_t ESP32_Bluetooth_ops = {
  "ESP32 Bluetooth",
  ESP32_Bluetooth_setup,
  ESP32_Bluetooth_loop,
  ESP32_Bluetooth_fini,
  ESP32_Bluetooth_available,
  ESP32_Bluetooth_read,
  ESP32_Bluetooth_write
};

#endif /* ESP32 */

