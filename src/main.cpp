/******************************************************************************

 * esp32_BLEtoMQTTBridge
 * Leonardo Bispo
 * April, 2020
 * https://github.com/ldab/esp32_BLEtoMQTTBridge

 * Distributed as-is; no warranty is given.

******************************************************************************/

#include <Arduino.h>

/***************************************************************************************************/
/* 
  Example for ROHM BH1750FVI Ambient Light Sensor library

  Power supply voltage: 2.4 - 3.6v
  Default range:        1 - 65'535 lux
  Measurement accuracy: ±20%, possible to calibrate
  Peak wave length:     560nm, yellow-green

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This chip uses I2C bus to communicate, specials pins are required to interface
  Board:                                    SDA                    SCL                    Level
  Uno, Mini, Pro, ATmega168, ATmega328..... A4                     A5                     5v
  Mega2560................................. 20                     21                     5v
  Due, SAM3X8E............................. 20                     21                     3.3v
  Leonardo, Micro, ATmega32U4.............. 2                      3                      5v
  Digistump, Trinket, ATtiny85............. 0/physical pin no.5    2/physical pin no.7    5v
  Blue Pill, STM32F103xxxx boards.......... PB7                    PB6                    3.3v/5v
  ESP8266 ESP-01........................... GPIO0/D5               GPIO2/D3               3.3v/5v
  NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2               GPIO5/D1               3.3v/5v
  ESP32.................................... GPIO21/D21             GPIO22/D22             3.3v

  Frameworks & Libraries:
  ATtiny Core           - https://github.com/SpenceKonde/ATTinyCore
  ESP32 Core            - https://github.com/espressif/arduino-esp32
  ESP8266 Core          - https://github.com/esp8266/Arduino
  ESP8266 I2C lib fixed - https://github.com/enjoyneering/ESP8266-I2C-Driver
  STM32 Core            - https://github.com/rogerclarkmelbourne/Arduino_STM32

  GNU GPL license, all text above must be included in any redistribution, see link below for details:
  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <BH1750FVI.h>
#include "Adafruit_HTU21DF.h"
#include "Adafruit_CCS811.h"

#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID     ""
#define WIFI_PASSWORD ""

#define MQTT_CLIENT ""
#define MQTT_HOST IPAddress(192, 168, 0, 84)
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASS ""

#define SLEEP_TIME_uS   15 * 60 * 1000 * 1000         // 15 min
#define SCAN_TIME_SEC   5
#define SENSOR_TIME_SEC 5 * 60

String knownDevices[] = { "cc:5e:9b:96:0a:79" };

bool isBoot = true;

BLEScan*        pBLEScan;
AsyncMqttClient mqttClient;
TimerHandle_t   mqttReconnectTimer;
TimerHandle_t   wifiReconnectTimer;
TimerHandle_t   readSensorTimer;
Adafruit_CCS811 ccs;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
      int8_t rssi;
      bool known_found = false;
      String _mac = advertisedDevice.getAddress().toString().c_str();

      for(int i = 0; i < (sizeof(knownDevices)/sizeof(knownDevices[0])); i++)
      {
        if( knownDevices[0].equalsIgnoreCase(_mac) )
        {
          rssi = advertisedDevice.getRSSI();
          known_found = true;
        }
      }

      if ( known_found )
      {
        String manuf_data  = advertisedDevice.getManufacturerData().c_str();

        // Remove manufacturer ID 0xFFFF
        manuf_data  = manuf_data.substring(2);
        float temp  = manuf_data.substring(0, manuf_data.indexOf("@")).toFloat() / 100.0;
        float humi  = manuf_data.substring(manuf_data.indexOf("@") + 1, manuf_data.lastIndexOf("@")).toFloat() / 100.0;
        int8_t batt = manuf_data.substring(manuf_data.lastIndexOf("@") + 1).toInt();

        char out_json[64];
        sprintf(out_json, "{\"Temp\": %.01f,\"Humi\": %.01f,\"Batt\": %d, \"RSSI\": %d}", temp, humi, batt, rssi); 

        Serial.println(out_json);

        String _topic = "/ble/" + _mac;
        mqttClient.publish(_topic.c_str(), 1, false, out_json);
      }
    }
};

BH1750FVI myBH1750(BH1750_DEFAULT_I2CADDR, BH1750_ONE_TIME_HIGH_RES_MODE, BH1750_SENSITIVITY_DEFAULT, BH1750_ACCURACY_DEFAULT);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		    xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void read_sensors(float *temp, float *humi, float *lux, uint16_t *co2, uint16_t *tvoc)
{

  *temp = htu.readTemperature();
  *humi = htu.readHumidity();
  *lux  = myBH1750.readLightLevel();
  
  myBH1750.powerDown();

  uint8_t compensate_humi = (uint8_t)*humi;
  double  compensate_temp = (double)*temp;

  ccs.setEnvironmentalData(compensate_humi, compensate_temp);

  if(ccs.available())
  {
    if(!ccs.readData())
    {
      *co2  = ccs.geteCO2();
      *tvoc = ccs.getTVOC();
    }
    else
      Serial.println(F("Failed to read sensor"));
  }
  else
    Serial.println(F("Failed to read sensor"));
  

}

void pubSensorData( void )
{

  float lux;

  float temp;
  float hum;

  uint16_t co2;
  uint16_t tvoc;

  char out_buf[12] = "";

  read_sensors(&temp, &hum, &lux, &co2, &tvoc);

  Serial.printf("T%.01f H%.01f L%.01f C%d TV%d\n", temp, hum, lux, co2, tvoc);
  
  sprintf(out_buf, "%.02f", lux);
  mqttClient.publish("logger/lux", 1, false, out_buf);

  sprintf(out_buf, "%.02f", temp);
  mqttClient.publish("logger/temp", 1, false, out_buf);

  sprintf(out_buf, "%.02f", hum);
  mqttClient.publish("logger/hum", 1, false, out_buf);

  sprintf(out_buf, "%d", WiFi.RSSI());
  mqttClient.publish("logger/RSSI", 1, false, out_buf);

  if( !isBoot )
  {
    sprintf(out_buf, "%d", co2);
    mqttClient.publish("logger/co2", 1, false, out_buf);

    sprintf(out_buf, "%d", tvoc);
    mqttClient.publish("logger/tvoc", 1, false, out_buf);
  }
  else
    isBoot = false;
  

}

void onMqttConnect(bool sessionPresent)
{
  pubSensorData( );

  Serial.printf("It took: %lu ms", millis());
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
    xTimerStart(mqttReconnectTimer, 0);
}

void setup()
{
  Serial.begin(115200);
  Serial.println(); 

  esp_sleep_enable_timer_wakeup(SLEEP_TIME_uS);
  //esp_deep_sleep_start();

  //Wire.begin(2, 15);

  while (myBH1750.begin() != true && millis() < 10000 )
  {
    Serial.println(F("ROHM BH1750FVI is not present")); //(F()) saves string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("ROHM BH1750FVI is present"));

  if( !ccs.begin() )
    Serial.println("Failed to start CCs811! Please check your wiring.");
  else
    Serial.println("CCS811 is present");
  
  // Wait for the sensor to be ready -> avoid infinite loop
  while(!ccs.available() && millis() < 10000 );

  if (!htu.begin())
    Serial.println("Couldn't find sensor!");
  else
    Serial.printf("Found HRU21D on 0x%02x\n", HTU21DF_I2CADDR);  

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  //mqttClient.onMessage(onMqttMessage);
  //mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);

  connectToWifi();
}

uint16_t i  = SENSOR_TIME_SEC / ( SCAN_TIME_SEC + 2);
uint16_t _i = i;

void loop()
{
  BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME_SEC, false);
  Serial.println(F("BLE-> Scan"));
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  delay(2000);

  _i--;
  if( _i == 0 )
  {
    pubSensorData();
    _i = i;
  }
}