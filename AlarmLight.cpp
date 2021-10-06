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
}
AlarmLight::~AlarmLight()
{
    free((void*)rgbw);
    free((void*)maxRGBW);
}

void AlarmLight::update()
{
    time_t t = now();

    if (day_of_alarm != day())
    {
        alarmActive = true;
        incrementAlarm();
    }

    if (now() < light_time) // Alarm hasnt happened
    {
        rgbw[0] = 0;
        rgbw[1] = 0;
        rgbw[2] = 0;
        rgbw[3] = 0;
        buzzer = false;
    }
    else if (now() < next_alarm) // Light-up stage
    {
        float progress = (now() - light_time) / LIGHTUP_TIME;
        rgbw[0] = (int)(maxRGBW[0] * progress);
        rgbw[1] = (int)(maxRGBW[1] * progress);
        rgbw[2] = (int)(maxRGBW[2] * progress);
        rgbw[3] = (int)(maxRGBW[3] * progress);
        buzzer = false;
    }
    else if(now() < end_alarm) // Alarm stage
    {
        rgbw[0] = maxRGBW[0];
        rgbw[1] = maxRGBW[1];
        rgbw[2] = maxRGBW[2];
        rgbw[3] = maxRGBW[3];
        buzzer = true;

        day_of_alarm = day();
    }
    else // Alarm has passed
    {
        rgbw[0] = 0;
        rgbw[1] = 0;
        rgbw[2] = 0;
        rgbw[3] = 0;
        buzzer = false;
    }
}

void AlarmLight::setAlarm(TimeElements alarm) { this->alarm = alarm; }
time_t AlarmLight::getAlarm(){ return next_alarm; }
bool AlarmLight::getBuzzer(){ return buzzer; }
int* AlarmLight::getLights(){ return rgbw; }

void AlarmLight::acknowledge()
{
    if (now() > light_time){ alarmActive = false; }
}

time_t AlarmLight::incrementAlarm()
{
    TimeElements tm;
    breakTime(now(), tm);

    tm.Second   = alarm.Second;
    tm.Minute   = alarm.Minute;
    tm.Hour     = alarm.Hour;

    next_alarm = makeTime(tm);
    return next_alarm;
}
