#define DEBUG_HARDWARE_SERIAL
#define SERIAL_SPEED 115200
#define CODE_VERSION 1.1

#include <Arduino.h>
#include <ESP8266WiFi.h>

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
}

void loop() {
  // put your main code here, to run repeatedly:
}
