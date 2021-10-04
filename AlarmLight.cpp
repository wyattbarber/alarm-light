#include "AlarmLight.h"
#include <algorithm>
#include <functional>

using SmartHome::AlarmLight;

AlarmLight::AlarmLight(const int *maxRGBW)
{
    this->maxRGBW[0] = maxRGBW[0];
    this->maxRGBW[1] = maxRGBW[1];
    this->maxRGBW[2] = maxRGBW[2];
    this->maxRGBW[3] = maxRGBW[3];
    this->state = WAIT_ALARM;
}
AlarmLight::~AlarmLight()
{
    free((void*)rgbw);
    free((void*)maxRGBW);
}

void AlarmLight::update()
{
    time_t t = now();

    int day_alm = day();

    Serial.println("Point 1");

    switch (state)
    {
    case WAIT_DAY:
    {
        if (day() != day_alm) 
        {
            state = WAIT_ALARM;
        }
        break;
    }
    case WAIT_ALARM:
    {
        Serial.println("WAIT");
        rgbw[0] = 0;
        rgbw[1] = 0;
        rgbw[2] = 0;
        rgbw[3] = 0;

        buzzer = false;

        if (t >= (alarm - LIGHTUP_TIME))
        {
            state = LIGHTUP;
        }
        break;
    }
    case LIGHTUP:
    {
        Serial.println("LIGHTUP");
        // Increase each color channel linearly to the maximum
        float progress = 1.0 - ((float)(alarm - t) / (float)LIGHTUP_TIME);
        rgbw[0] = (int)(progress * maxRGBW[0]);
        rgbw[1] = (int)(progress * maxRGBW[1]);
        rgbw[2] = (int)(progress * maxRGBW[2]);
        rgbw[3] = (int)(progress * maxRGBW[3]);

        buzzer = false;

        if (t >= alarm)
        {
            state = ALARM;
        }
        break;
    }

    case ALARM:
    {
        Serial.println("ALARM");
        // Alarm ended by call to acknowledge()
        // Blink at 0.5 Hz
        buzzer = (t % 2);

        Serial.println("Point 1.1");

        rgbw[0] = maxRGBW[0] * (int)buzzer;
        rgbw[1] = maxRGBW[1] * (int)buzzer;
        rgbw[2] = maxRGBW[2] * (int)buzzer;
        rgbw[3] = maxRGBW[3] * (int)buzzer;

        Serial.println("Point 1.2");

        // Run for a maximum of 20 minutes
        if (t >= (alarm + (MAX_ALARM_TIME)))
        {
            Serial.println("Point 1.3");
            day_alm = day();
            state = WAIT_DAY;
        }
        break;
    }
    }
    Serial.println("Point 2");
}

void AlarmLight::set(time_t alarm) { this->alarm = alarm; }

void AlarmLight::acknowledge()
{
    if((state == LIGHTUP) || (state == ALARM))
    {
        state = WAIT_DAY;
    }
}

/*
void AlarmLight::clear()
{
    Serial.println(alarms.size());
    Serial.println("Clear()");
    if (alarms.size() > 0) 
    {
        int idx_min = 0;
        for (int idx = 0; idx < alarms.size(); idx++)
        {
            Serial.println("Clear().1");

            if (alarms.at(idx) < alarms.at(idx_min))
            {
                Serial.println("Clear().2");

                idx_min = idx;
            }
        }
        Serial.println("Clear().3");
        alarms.erase(alarms.begin() + idx_min);
    }
    Serial.println("Clear().end");
}
*/

int *AlarmLight::getLights() { return rgbw; }

bool AlarmLight::getBuzzer() { return buzzer; }

/*
time_t AlarmLight::getNext()
{
    int idx_min = 0;
    for (int idx = 0; idx < alarms.size(); idx++)
    {
        if (alarms.size() != 0)
        {
            if (alarms.at(idx) < alarms.at(idx_min))
            {
                idx_min = idx;
            }
        }
    }
    return alarms.at(idx_min);
}
*/

SmartHome::AlarmStatus AlarmLight::getStatus()
{
    return SmartHome::AlarmStatus(state, now(), alarm);
}
