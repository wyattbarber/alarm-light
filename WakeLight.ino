#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "WakeLight.h"

const char * ssid = "the wifi";
const char*  pswd = "the wifi password";

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const long ntpOffset = 3600;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", ntpOffset);

void setup() {
 WiFi.begin(ssid,pswd);
 timeClient.begin();
 Serial.begin(9600);
}

void loop() {
  if(WiFi.status()==WL_CONNECTED) {
    Serial.println("Connected to network");
    
    timeClient.update();
    Serial.print(daysOfTheWeek[timeClient.getDay()]);
    Serial.print(", ");
    Serial.print(timeClient.getHours());
    Serial.print(":");
    Serial.print(timeClient.getMinutes());
    Serial.print(":");
    Serial.println(timeClient.getSeconds());
  }
  else {
    Serial.println("Not connected");
  }
  delay(1000);
}
