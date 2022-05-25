#include "AlarmLight.h"
#include <algorithm>
#include <functional>
#include <Arduino.h>

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

void AlarmLight::On() 
{
    on = true;
    time_on = now();
    alarm_time = time_on + LIGHTUP_TIME;
    end_alarm = alarm_time + MAX_ALARM_TIME;
}

void AlarmLight::Off() {on = false;}

void AlarmLight::update()
{
    if (on) {
        if (now() < alarm_time) // Light-up stage
        {
            int progress = ((now() - time_on) * 100)/ LIGHTUP_TIME;
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
    else { // Alarm is not on
        rgbw[0] = 0;
        rgbw[1] = 0;
        rgbw[2] = 0;
        rgbw[3] = 0;
        buzzer = false;
    }
}

bool AlarmLight::getBuzzer(){ return buzzer; }

int* AlarmLight::getLights(){ return rgbw; }
