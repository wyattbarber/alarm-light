#include <ESP8266WiFi.h>

#include "AlarmLight.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeLib.h>

#define BUT_PIN D1
#define TWELVE_V_PIN D3
#define RED_PIN D4
#define GRN_PIN D5
#define BLU_PIN D6
#define WHT_PIN D7
#define BUZ_PIN D8
#define MAT_PIN A0

// Wifi access data
const char *ssid = "the wifi";
const char *pswd = "the wifi password";

// NTP client data
long ntpOffset = -18000;
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", ntpOffset);
TimeElements jetzt;

// Alarm
SmartHome::AlarmLight *alarm;

// Trigger rising edge on beginning of each day for alarm reset
SmartHome::Helpers::RTrig newDay([jetzt](){return (jetzt.Hour == 0);});

// Button edge detect
SmartHome::Helpers::RTrig button([](){return digitalRead(BUT_PIN);});

void setup()
{
    // Set pins
    pinMode(TWELVE_V_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(BLU_PIN, OUTPUT);
    pinMode(GRN_PIN, OUTPUT);
    pinMode(WHT_PIN, OUTPUT);

    // Begin WiFi connection and NTP client
    WiFi.begin(ssid, pswd);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
    }
    timeClient.begin();
    setSyncProvider(getNtpTime); // timeClient.update() called in getNtpTime()
    setSyncInterval((time_t)60); // getNtpTime() called every 60 seconds

    // Set alarm max illumination
    int maxLights[] = {1023, 1023, 1023, 1023};
    alarm = new SmartHome::AlarmLight(maxLights);
    TimeElements a = jetzt;
    a.Hour      = 6;
    a.Minute    = 0;
    a.Second    = 0;
    alarm->set(makeTime(a));

    Serial.begin(57600);
}

void loop()
{
    Serial.print("Looping... ");
    Serial.print(alarm->getStatus().toString());
    Serial.print('\n');

    // Update alarm status
    alarm->update();
    
    // Set light and buzzer outputs
    int* rgbw = alarm->getLights();
    if(
        (rgbw[0] != 0)
    ||  (rgbw[1] != 0)
    ||  (rgbw[2] != 0)
    ||  (rgbw[3] != 0)
    )
    {
        digitalWrite(TWELVE_V_PIN, HIGH);
    }
    else
    {
        digitalWrite(TWELVE_V_PIN, LOW);
    }
    
    Serial.println("point 3");

    analogWrite(RED_PIN, rgbw[0]);
    analogWrite(BLU_PIN, rgbw[1]);
    analogWrite(GRN_PIN, rgbw[2]);
    analogWrite(WHT_PIN, rgbw[3]);
    digitalWrite(BUZ_PIN, alarm->getBuzzer());

    // Check button
    if(button.Q())
    {
        alarm->acknowledge();
    }

    // Set new alarm if needed
    if(newDay.Q())
    {
        TimeElements a = jetzt;
        a.Hour      = 6;
        a.Minute    = 0;
        a.Second    = 0;
        alarm->set(makeTime(a));
    }

    delay(5000);
}

time_t getNtpTime()
{
    timeClient.update();
    time_t t = timeClient.getEpochTime();
    breakTime(t, jetzt);
    return (time_t)(t);
}