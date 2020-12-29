#include "AlarmLight.h"
#include <algorithm>
#include <functional>

using SmartHome::AlarmLight;


AlarmLight::AlarmLight(int* maxRGBW, TimeElements* defaults){
    this->maxRGBW   = maxRGBW;
    this->defaults  = defaults;
    this->setDefaults();
    std::function<bool()> f = [this](){return (this->tm.Wday == Sunday);};
    this->resetDefaults = new Logic::RTrig(f);
    this->state     = WAIT;
}

void AlarmLight::update(){
    // internal data updates
    breakTime(now(), tm);
    resetDefaults->update();
    if(resetDefaults->Q())
    {
        setDefaults();
    }
    time_t nextAlarm = getNextAlarm();

    switch(state)
    {
        case WAIT:

        break;
        case PREALARM:

        break;  
        case ALARM:

        break;
        case ERROR:

        break;
    }
}

int* AlarmLight::getLights(){return rgbw;}

SmartHome::AlarmStatus AlarmLight::getStatus()
{
    return SmartHome::AlarmStatus(state, now(), getNextAlarm());
}

time_t AlarmLight::clearDefault()
{
    time_t t = defaultAlarms.back();
    defaultAlarms.pop_back();
    return t;
}

time_t AlarmLight::clearTemp()
{
    time_t t = tempAlarms.back();
    tempAlarms.pop_back();
    return t;
}

time_t AlarmLight::clearNext()
{
    getNextAlarm();
    if(usingTemp)
    {
        return clearTemp();
    }
    else
    {
        return clearDefault();
    }   
}

time_t AlarmLight::getNextAlarm()
{
    std::sort(defaultAlarms.begin(), defaultAlarms.end());
    std::reverse(defaultAlarms.begin(), defaultAlarms.end());
    std::sort(tempAlarms.begin(), tempAlarms.end());
    std::reverse(tempAlarms.begin(), tempAlarms.end());

    time_t nextTemp = tempAlarms.back();
    time_t nextDefault = defaultAlarms.back();

    if(nextTemp > nextDefault)
    {
        usingTemp = true;
        return nextTemp;
    }
    else
    {
        usingTemp = false;
        return nextDefault;
    }
}

time_t AlarmLight::getNextWday(TimeElements tn)
{
    TimeElements t;
    breakTime(now(), t);

    time_t diff = (tn.Wday - t.Wday) * 24 * 60 * 60;
    time_t tn_t = now() + diff;

    TimeElements out;
    breakTime(tn_t, out);
    out.Hour    = tn.Hour;
    out.Minute  = tn.Minute;
    out.Second  = tn.Second;

    return makeTime(out);
}

void AlarmLight::setDefaults()
{
    for(int i = 0; i < (sizeof(defaults)/sizeof(TimeElements)); i++)
    {
        defaultAlarms.push_back(getNextWday(defaults[i]));
    }
}