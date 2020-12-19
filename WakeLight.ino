#include <ESP8266WiFi.h>

//#include "WakeLight.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeLib.h>

#define BUT_PIN         D1
#define FIVE_V_PIN      D2
#define TWELVE_V_PIN    D3
#define RED_PIN         D4
#define GRN_PIN         D5
#define BLU_PIN         D6
#define WHT_PIN         D7
#define BUZ_PIN         D8
#define MAT_PIN         A0

const char *ssid = "the wifi";
const char *pswd = "the wifi password";

long ntpOffset = -18000;
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", ntpOffset);

void setup()
{
    Serial.begin(115200);
    WiFi.begin(ssid, pswd);
    while(WiFi.status() != WL_CONNECTED )
    {
        Serial.print(".");
        delay(500);
    }
    Serial.print('\n');
    timeClient.begin();
    setSyncProvider(getNtpTime);
    setSyncInterval((time_t)5);
}

void loop()
{
    time_t t = now();
    Serial.println(String(t)+" = "+timeClient.getFormattedTime());
    delay(1000);
}

time_t getNtpTime()
{
    timeClient.update();
    return (time_t)(timeClient.getEpochTime()); 
}