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
        int progress = ((now() - light_time) * 100)/ LIGHTUP_TIME;
        rgbw[0] = map(progress, 0, 100, 0, maxRGBW[0]);
        rgbw[1] = map(progress, 0, 100, 0, maxRGBW[1]);
        rgbw[2] = map(progress, 0, 100, 0, maxRGBW[2]);
        rgbw[3] = map(progress, 0, 100, 0, maxRGBW[3]);
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

void AlarmLight::setAlarm(TimeElements alarm) { 
    this->alarm = alarm; 
    incrementAlarm();
}

time_t AlarmLight::getAlarm(){ return next_alarm; }

bool AlarmLight::getBuzzer(){ return buzzer; }

int* AlarmLight::getLights(){ return rgbw; }

void AlarmLight::acknowledge()
{
    if (now() > light_time){ alarmActive = false; }
}

time_t AlarmLight::incrementAlarm()
{
    time_t midnight = now() - ((hour()*3600) + (minute()*60) + second());
    next_alarm = midnight;
    next_alarm += alarm.Hour * 3600;
    next_alarm += alarm.Minute * 60;
    next_alarm += alarm.Second;
    light_time = next_alarm - LIGHTUP_TIME;
    end_alarm = next_alarm + MAX_ALARM_TIME;
    return next_alarm;
}
