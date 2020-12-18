#include <ESP8266WiFi.h>

#include "WakeLight.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

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

const long ntpOffset = -18000;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpOffset);
Alarm::WakeLight alarmClock;

int* rgbw = (int*)malloc(4*sizeof(int));

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
}

void loop()
{
    timeClient.update();
    if((millis() % 5000)==0)
    {
        alarmClock.setBaseTime(Alarm::Time((Alarm::DaysOfWeekNum)timeClient.getDay(),   
                                            timeClient.getHours(),
                                            timeClient.getMinutes(),
                                            timeClient.getSeconds()));
    }
    alarmClock.update();
    Serial.println(alarmClock.getStatus().time.toString()+'\n');
    delay(1000);
}
