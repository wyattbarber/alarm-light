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
long ntpOffset = -14400;
WiFiUDP udp;
NTPClient timeClient(udp);
TimeElements jetzt;

// Alarm, max illumination
SmartHome::AlarmLight *alarm;    
int maxLights[] = {1023, 1023, 1023, 1023};

// Button edge detect
SmartHome::Helpers::RTrig button([](){return digitalRead(BUT_PIN);});

void setup()
{
    // Start serial for debugging
    Serial.begin(57600);

    // Construct alarm
    alarm = new SmartHome::AlarmLight(maxLights);

    // Set pin modes
    pinMode(TWELVE_V_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(BLU_PIN, OUTPUT);
    pinMode(GRN_PIN, OUTPUT);
    pinMode(WHT_PIN, OUTPUT);

    // Begin WiFi
    WiFi.begin(ssid, pswd);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connecting... ");
        delay(1000);
    }

    // Begin NTP client
    timeClient.begin();
    timeClient.setTimeOffset(0);
    timeClient.setUpdateInterval(60 * SECS_PER_HOUR);
    Serial.println("TimeClient started.");
    setSyncProvider(getNtpTime); // timeClient.update() called in getNtpTime()
    setSyncInterval((time_t)60); // getNtpTime() called every minute

    // Set alarm time
    while (timeStatus() != timeSet)
    {
        Serial.println("NTP client connecting... ");
        delay(1000);
    }

    Serial.println("Setting alarm.");
    TimeElements a;
    a.Hour      = 6;
    a.Minute    = 0;
    a.Second    = 0;
    alarm->setAlarm(a);

}

void loop()
{
    Serial.println("Looping... ");

    // Update alarm status
    alarm->update();
    int* rgbw = alarm->getLights();

    // Set 12V relay output
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
    
    // Set light outputs
    analogWrite(RED_PIN, rgbw[0]);
    analogWrite(BLU_PIN, rgbw[1]);
    analogWrite(GRN_PIN, rgbw[2]);
    analogWrite(WHT_PIN, rgbw[3]);

    // Set buzzer output
    digitalWrite(BUZ_PIN, alarm->getBuzzer());

    // Check button
    if(button.Q())
    {
        alarm->acknowledge();
    }

    // Log times status for debugging
    Serial.print("Current time: ");
    Serial.print(now());
    Serial.print(". Alarm at: ");
    Serial.print(alarm->getAlarm());
    Serial.print(". Time to go: ");
    Serial.print(alarm->getAlarm() - now());
    Serial.print('\n');
    Serial.print("Lights: ");
    Serial.print(rgbw[0]);
    Serial.print(", ");
    Serial.print(rgbw[1]);
    Serial.print(", ");
    Serial.print(rgbw[2]);
    Serial.print(", ");
    Serial.print(rgbw[3]);
    Serial.print(". \n");

    delay(1000);
}

time_t getNtpTime()
{
    timeClient.update();
    time_t t = timeClient.getEpochTime();
    breakTime(t, jetzt);
    return (time_t)(t + ntpOffset);
}