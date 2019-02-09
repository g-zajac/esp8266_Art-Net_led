#define DEBUG_HARDWARE_SERIAL                                                   // if commented, not defined, serial debug info will be off
#define SERIAL_SPEED 115200
#define CODE_VERSION 1.9
#define HOSTNAME "costume"                                                      //costumeXXX - XXX last octet of IP address
#define UNIVERSE 0                                                              //set for 0 with Max MSP, 1 for lighting desk
#define LED_OUT  13
#define ADCINPUT A0                                                             //battery monitoring level - connected via resistors devider

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "credentials.h"                                                        //ignored by git to keep the network details in separate file private
/* credentials.h template:
const char* ssid = "network_name";
const char* password = "password";
IPAddress gateway(10,0,100,1);
IPAddress subnet(255,255,255,0);
const IPAddress remoteIP(xxx,xxx,xxx,xxx);        // remote IP of your computer
const unsigned int destPort = xxxx;          // remote port to receive OSC
const unsigned int localPort = xxxx;        // local port to listen for OSC packets (actually not used for sending)
 */

 #include <ArduinoOTA.h>
//   |--------------|-------|---------------|--|--|--|--|--|
//   ^              ^       ^               ^     ^
//   Sketch    OTA update   File system   EEPROM  WiFi config (SDK)

extern "C"{
 #include "user_interface.h"                                                    //NOTE needed for esp_system_info Since the include file from SDK is a plain C not a C++
}
#include "devices.h"                                                            //list of MAC addresses of devices for self assigning static IPs
IPAddress deviceip;
int unitID;
char unitName[16];

//----------------------------------------- ArtNet -----------------------------
#include <ArtnetWifi.h>   //cloned from https://github.com/rstephan/ArtnetWifi.git
ArtnetWifi artnet;

//----------------------------------------- OSC --------------------------------
#include <OSCBundle.h>
#include <OSCData.h>
WiFiUDP Udp;
OSCErrorCode error;
unsigned long runningTime, previousMillisGlobal;
int reportInterval = 3000;                                                      // send OSC report every 3 sec

//------------------------ functions -------------------------------------------
void blink(int tOn, int tOff){                                                  // for LEDs testing
static int timer=tOn;
  static long previousMillis;
  if ((millis() - previousMillis) >= timer) {
    if (digitalRead(LED_OUT) == HIGH) {
      timer = tOff;
    } else {
      timer = tOn;
    }
digitalWrite(LED_OUT, !digitalRead(LED_OUT));
     previousMillis = millis();
  }
}

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){

  if (universe == UNIVERSE) {

    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.print("DMX: Univ: ");
      Serial.print(universe, DEC);
      Serial.print(", Seq: ");
      Serial.print(sequence, DEC);
      Serial.print(", Data (");
      Serial.print(length, DEC);
      Serial.print("), Address: ");
      Serial.print(deviceip[3]);
      Serial.print(", Value: ");
      Serial.print(data[deviceip[3]-1]);
      Serial.println("");
    #endif

    int value = data[unitID-1];                                            //artnet address = unit IP last octet, starting from 0 -> -1
    if (value == 0) analogWrite(LED_OUT, value);
    if (value > 0 and value <=255) analogWrite(LED_OUT, value);
    if (value > 255){
      analogWrite(LED_OUT, 0);    //disable PWM
      digitalWrite(LED_OUT, HIGH);
      }
    }
}

void sendOSCmessage(char* name, float value){
  char message_osc_header[20];
  message_osc_header[0] = {0};
  strcat(message_osc_header, unitName);
  strcat(message_osc_header, name);
  OSCMessage message(message_osc_header);
  message.add(value);
  Udp.beginPacket(remoteIP, destPort);
  message.send(Udp);
  Udp.endPacket();
  message.empty();
}

void sendReport(){
  sendOSCmessage("/ver", CODE_VERSION);
  sendOSCmessage("/rssi", WiFi.RSSI());
  sendOSCmessage("/channel", WiFi.channel());
  sendOSCmessage("/time", (millis()/1000));               //running time in secs
  float v = analogRead(ADCINPUT * 10.65); // ((30000 + 3000)/3000)calibration based on voltage divider 30k - 3k, calibrated on workbench
  float voltage = v / 1024;
  sendOSCmessage("/voltage", voltage);                                          //TODO test AD input
}

//------------------------------------------------------------------------------

void setup() {
  pinMode(LED_OUT, OUTPUT);
  analogWrite(LED_OUT, 0);

  #ifdef DEBUG_HARDWARE_SERIAL
    Serial.begin(SERIAL_SPEED);
    Serial.println(" ");  Serial.println(" ");  Serial.println(" ");            //Some compilation and other info
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

//---------------------------- WiFi --------------------------------------------
// determine IP address based on devices.h definitions, each device has exactly the same code and address itself accordingly to mac table
  int chip_id = ESP.getChipId();
  const device_details *device = devices;
  for (; device->esp_chip_id != 0; device++) {
    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.printf("chip_id %X = %X?\n", chip_id, device->esp_chip_id);
    #endif
    if (device->esp_chip_id == chip_id)
      break;
  }
  if (device->esp_chip_id == 0) {
    while(1) {
      #ifdef DEBUG_HARDWARE_SERIAL
        Serial.println("Could not obtain a chipId we know. Means we dont know what id/IP address to asign. Fail");
        Serial.printf("This ESP8266 Chip id = 0x%08X\n", chip_id);
      #endif
    }
  }
  deviceip = IPAddress(gateway);
  unitID = device->id;
  #ifdef DEBUG_HARDWARE_SERIAL
    Serial.print("found in devices list ID: "); Serial.println(unitID);
  #endif

  deviceip[3] = unitID;

  WiFi.mode(WIFI_STA);
  WiFi.config(deviceip, gateway, subnet);
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

  // build unit name
  unitName[0] = {0};
  snprintf(unitName, 30, "%s%i", HOSTNAME, unitID);

// --------------------------- OTA ---------------------------------------------
  ArduinoOTA.setHostname(unitName);
  #ifdef DEBUG_HARDWARE_SERIAL
    Serial.print("Hostname: "); Serial.println(unitName);
  #endif

  ArduinoOTA.setHostname(HOSTNAME);

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
    digitalWrite(LED_OUT, HIGH);
    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
    digitalWrite(LED_OUT, LOW);
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

// -----------------------------------------------------------------------------

  //initialize artnet
  artnet.begin();

  // this will be called for each packet received
  artnet.setArtDmxCallback(onDmxFrame);

  //OSC receiving, might be use for remote triggering report instead of fixed time slots
  //Udp.begin(localPort);
}

void loop() {
  ArduinoOTA.handle();
  //blink(500,500);                                                             // for LEDs testing
  artnet.read();

  //send OSC report
  if (millis() - previousMillisGlobal >= reportInterval){
    #ifdef DEBUG_HARDWARE_SERIAL
      Serial.println("sending OSC report");
    #endif
    sendReport();
    previousMillisGlobal = millis();
  }

}
