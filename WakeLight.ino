#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "WakeLight.h"

const char * ssid = "the wifi";
const char*  pswd = "the wifi password";

const long ntpOffset = 3600;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", ntpOffset);

void setup() {
 WiFi.begin(ssid,pswd);
 timeClient.begin();
 Serial.begin(9600);
}

void loop() {
  
  
}
