#define DEBUG_HARDWARE_SERIAL
#define SERIAL_SPEED 115200
#define CODE_VERSION 1.2

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "credentials.h"                                                        //ignored by git to keep the network details private
/* credentials.h example:
const char* ssid = "network_name";
const char* password = "password";
const IPAddress ip(10,10,10,101);                                               //ip address of the unit
 */

 #include <ArduinoOTA.h>
//   |--------------|-------|---------------|--|--|--|--|--|
//   ^              ^       ^               ^     ^
//   Sketch    OTA update   File system   EEPROM  WiFi config (SDK)

void setup() {
  #ifdef DEBUG_HARDWARE_SERIAL
    Serial.begin(SERIAL_SPEED);
    //Compilation info
    Serial.println(" ");  Serial.println(" ");  Serial.println(" ");
    Serial.println("---------------------------------------");
    Serial.println("Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
    Serial.print("Code version: "); Serial.println(CODE_VERSION);
    Serial.println("---------------------------------------");
    Serial.println("ESP Info: ");
    Serial.print( F("Heap: ") ); Serial.println(system_get_free_heap_size());
    Serial.print( F("Boot Vers: ") ); Serial.println(system_get_boot_version());
    Serial.print( F("CPU: ") ); Serial.println(system_get_cpu_freq());
    Serial.print( F("SDK: ") ); Serial.println(system_get_sdk_version());
    Serial.print( F("Chip ID: ") ); Serial.println(system_get_chip_id());
    Serial.print( F("Flash ID: ") ); Serial.println(spi_flash_get_id());
    Serial.print( F("Flash Size: ") ); Serial.println(ESP.getFlashChipRealSize());
    Serial.printf("Sketch size: %u\n", ESP.getSketchSize());
    Serial.printf("Free size: %u\n", ESP.getFreeSketchSpace());
    Serial.print( F("Vcc: ") ); Serial.println(ESP.getVcc());
    Serial.println(" ");
    Serial.println("Starting Setup");
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.println("Connection Failed! Rebooting...");
    #endif
    delay(5000);
    ESP.restart();
  }

  #ifdef DEBUG_HARDWARE_SERIAL
    Serial.print("unit MAC address: "); Serial.println(WiFi.macAddress());
    Serial.print("assigned IP address: "); Serial.println(WiFi.localIP());
  #endif

// --------------------------- OTA ---------------------------------------------
  ArduinoOTA.onStart([]() {
  #ifdef DEBUG_HARDWARE_SERIAL
    Serial.println("Uploading...");
  #endif
  });
  ArduinoOTA.onEnd([]() {
    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.println("\nEnd");
    #endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });
  ArduinoOTA.onError([](ota_error_t error) {
    #ifdef DEBUG_HARDWARE_SERIAL
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    #endif
  });
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
}
